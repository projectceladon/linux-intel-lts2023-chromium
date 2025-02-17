#include "i915_gpu_work.h"
#include <linux/pid.h>
#include <linux/errno.h>
#include <linux/jiffies.h>

#include "gt/intel_context.h"
#include "gt/intel_engine.h"
#include "gem/i915_gem_context.h"

#define CREATE_TRACE_POINTS
#include "intel_power_gpu_work_period_trace.h"

static inline u32 get_stats_uid(s32 key, struct i915_work_stats *stats)
{
    struct i915_work_stats *stat = &stats[key];
    u32 uid = 0;

    spin_lock(&stat->lock);
    uid = READ_ONCE(stat->uid);
    spin_unlock(&stat->lock);
    return uid;
}

static s32 get_uid_ctx(struct intel_context *ce)
{
    struct i915_gem_context *ctx = NULL;
    struct task_struct *task = NULL;
    const struct cred *cred = NULL;
    s32 ret;

    rcu_read_lock();
    ctx = rcu_dereference(ce->gem_context);
    /* ctx could be freed from right under our nose,
     * so check first if we are able to get a reference
     */
    if (ctx && !kref_get_unless_zero(&ctx->ref))
        ctx = NULL;
    rcu_read_unlock();

    if (!ctx) {
        ret = -EINVAL;
        goto out;
    }

    task = get_pid_task(ctx->pid, PIDTYPE_PID);
    if (!task) {
        ret = -EINVAL;
        goto put_ctx; 
    }

    cred = get_task_cred(task);
    if (!cred) {
        ret = -EINVAL;
        goto put_task;
    }

    const unsigned int uid = cred->euid.val;
    ret = (s32)uid;

    put_cred(cred);
put_task:
    put_task_struct(task);
put_ctx:
    i915_gem_context_put(ctx);
out:
    return ret;
}

static void emit_work_period_event(struct i915_work_stats *stat)
{
    struct intel_context *ce = NULL, *ce2 = NULL;

    GEM_BUG_ON(!stat->uid);

    lockdep_assert_held(&stat->lock);

    trace_gpu_work_period(stat->gpu_id, stat->uid,
        stat->start_time_ns, stat->end_time_ns,
        stat->active_duration_ns);

    /* clean up the slate */
    stat->uid = 0;
    stat->start_time_ns = 0;
    stat->end_time_ns = 0;
    stat->active_duration_ns = 0;
    stat->jiffies = 0;

    /* Remove all the contexts associated with this uid and drop their
     * reference
     */
    list_for_each_entry_safe(ce, ce2, &stat->ctx_list, work_stat_link) {
        list_del_init(&ce->work_stat_link);
        intel_context_put(ce);
    }
    smp_mb();
}

static inline u32 get_cur_dt(struct intel_context* ce)
{
    struct intel_context_stats *stats = &ce->stats;
    s32 dt = stats->runtime.dt;
    WRITE_ONCE(stats->runtime.dt, 0);
    /* Make this visible to lrc_update_runtime */
    smp_wmb();

    if (unlikely(dt < 0)) {
        return 0;
    }
    return dt;
}

static u64 get_active_duration_ns(struct intel_context* ce)
{
    u64 dur = get_cur_dt(ce);
	if (ce->ops->flags & COPS_RUNTIME_CYCLES)
		dur *= ce->engine->gt->clock_period_ns;
    return dur;
}

static int handle_collision(s32 key, struct i915_engine_work *ew,
                            u32 uid)
{
    struct i915_work_stats * const stats = &ew->stats[0];
    u32 count = 0;

    GEM_BUG_ON(KEY_INVALID(key));

    while (get_stats_uid(key, stats) != uid) {
        if (unlikely(count >= I915_ENGINE_WORK_STATS_COUNT)) {
            return -ENOENT;
        }
        key++;
        if (key == I915_ENGINE_WORK_STATS_COUNT)
            key = 0;
        count++;
    }
    return key;
}

static int find_next_available_slot(int key, struct i915_engine_work *ew)
{
    return handle_collision(key, ew, 0);
}

void i915_gpu_work_process_ctx(struct intel_context *ce,
                     struct i915_engine_work *ew)
{
    struct i915_work_stats *stat = NULL;
    s32 key = 0, uid = 0, cur_uid = 0;

    // TODO: Handle this carefully
    if (!ew->enabled)
        return;

    uid = get_uid_ctx(ce);
    if (uid < 0)
        return;

    key = HASH_MAP(uid);
    cur_uid = get_stats_uid(key, ew->stats);

    if (unlikely(cur_uid && cur_uid != uid)) {
        /*
         * We have encountered a hash collision.
         * First check if the uid is already present in another
         * slot by doing a linear search
         */
        key = handle_collision(key, ew, uid);
        /*
         * We couldn't find the uid in the stats array
         * this means this is the first occurence of this
         * uid. So we find the next available slot
         */
        if (KEY_INVALID(key))
            key = find_next_available_slot(key, ew);

                    /*
         * This can only happen if all the slots in our stats
         * array are occupied. Emit the event and evict one slot.
         */
        if (KEY_INVALID(key)) {
            u32 idx = HASH_MAP(uid);
            stat = &ew->stats[idx];
            spin_lock(&stat->lock);
            emit_work_period_event(stat);
            spin_unlock(&stat->lock);
            key = idx;
        }
    }
    stat = &ew->stats[key];

    /*
     * If the uid at our hash index is empty (zero)
     * this implies that our ctx is processed first
     * time since we last emitted the events and
     * subsequently evicted all the slots.
     * So, we set the start time to the last time this
     * ctx was put into the active queue. We also set
     * the end time and the total active duration to
     * the current runtime of this ctx
     */
    spin_lock(&stat->lock);

    GEM_BUG_ON(stat->uid && (stat->uid != uid));
    if (!stat->uid) {
        stat->uid = uid;
        stat->start_time_ns = READ_ONCE(ce->start_time_ns);
        stat->active_duration_ns =
                    get_active_duration_ns(ce);
        stat->end_time_ns = ktime_get_raw_ns();
        atomic_inc(&ew->num_entries);

        goto list_add;
    }

    /*
     * We set the endtime to the current time this ctx
     * is being processed and accumulate the current
     * runtime to the total active duration
     */
    stat->end_time_ns = ktime_get_raw_ns();
    stat->active_duration_ns +=
                get_active_duration_ns(ce);

    unsigned long delta = jiffies - stat->jiffies;
    if (jiffies_to_msecs(delta) >=
                GPU_WORK_PERIOD_EVENT_TIMEOUT)
    {
        emit_work_period_event(stat);
        stat->jiffies = jiffies;
        goto out;
    }

list_add:
    if (list_empty(&ce->work_stat_link)) {
        /* This implies the context wasn't being tracked
         * until this point. Get a reference and add this
         * to the list to mark it as being tracked.
         */
        if (intel_context_get(ce))
            list_add(&ce->work_stat_link, &stat->ctx_list);
    }
out:
    spin_unlock(&stat->lock);
}

void i915_gpu_work_stats_init(struct intel_engine_cs *engine)
{
    struct i915_engine_work *ew = &engine->gpu_work;

    atomic_set(&ew->num_entries, 0);

    /* Initalize the slots */
    for (int i = 0; i < I915_ENGINE_WORK_STATS_COUNT; i++) {
        struct i915_work_stats *stat = &ew->stats[i];

        stat->gpu_id = engine->id;
        stat->uid = 0;
        stat->start_time_ns = 0;
        stat->end_time_ns = 0;
        stat->active_duration_ns = 0;
        stat->jiffies = 0;

        spin_lock_init(&stat->lock);
        INIT_LIST_HEAD(&stat->ctx_list);
    }

    /* Enable gpu work period */
    WRITE_ONCE(ew->enabled, true);
    smp_wmb();
}

void i915_gpu_work_stats_fini(struct intel_engine_cs *engine)
{
    struct i915_engine_work *ew = &engine->gpu_work;

    WRITE_ONCE(ew->enabled, false);
    smp_wmb();

    if (!atomic_read(&ew->num_entries))
        return;

    for (int i = 0; i < I915_ENGINE_WORK_STATS_COUNT; i++) {
        struct i915_work_stats *stat = &ew->stats[i];

        if (!get_stats_uid(i, stat))
            continue;

        spin_lock(&stat->lock);
        emit_work_period_event(stat);
        spin_unlock(&stat->lock);

        if (atomic_dec_and_test(&ew->num_entries))
            break;
    }
}
