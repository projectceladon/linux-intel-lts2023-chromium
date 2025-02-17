#include "i915_gpu_work.h"
#include <linux/pid.h>
#include <linux/errno.h>
#include <linux/jiffies.h>

#include "gt/intel_context.h"
#include "gt/intel_engine.h"
#include "gt/intel_lrc_reg.h"
#include "gem/i915_gem_context.h"

#define CREATE_TRACE_POINTS
#include "intel_power_gpu_work_period_trace.h"

static inline u32 get_stats_uid(s32 key, struct i915_work_stats *stats)
{
    struct i915_work_stats *stat = &stats[key];
    return READ_ONCE(stat->uid);
}

static s32 get_uid_ctx(struct intel_context *ce)
{
    struct i915_gem_context *ctx = NULL;
    struct task_struct *task = NULL;
    const struct cred *cred = NULL;
    s32 ret;

    rcu_read_lock();
    ctx = rcu_dereference(ce->gem_context);
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

static void __emit_work_period_event(struct i915_work_stats *stat, bool discard)
{
    struct intel_context *ce = NULL, *ce2 = NULL;

    GEM_BUG_ON(!stat->uid);

    lockdep_assert_held(&stat->lock);

    if (STAT_INVALID(stat))
        discard = true;

    if (!discard) {
        trace_gpu_work_period(stat->gpu_id, stat->uid,
            stat->start_time_ns, stat->end_time_ns,
            stat->active_duration_ns);
    }

    /* clean up the slate */
    /* We keep the uid and the end time intact since we
       may encounter the same uid again soon */
    stat->start_time_ns = 0;
    stat->active_duration_ns = 0;
    stat->jiffies = 0;

    /* Remove all the contexts associated with this uid and drop their
     * reference
     */
    list_for_each_entry_safe(ce, ce2, &stat->contexts, record.ws_link) {
        list_del_init(&ce->record.ws_link);
        intel_context_put(ce);
    }
    smp_mb();
}

static void emit_work_period_event(struct i915_work_stats *stat)
{
    lockdep_assert_held(&stat->lock);

    u64 start_time = stat->start_time_ns;
    u64 end_time = stat->end_time_ns;

    /* Google requirement restricts the interval between end time
     * and start time to be at most 1 second
     */
    bool discard = ((end_time - start_time) >
                        GPU_WORK_TIME_GAP_LIMIT_NS);

    __emit_work_period_event(stat, discard);
}

static void emit_event_and_evict_slot(struct i915_work_stats *stat)
{
    lockdep_assert_held(&stat->lock);

    u64 start_time = stat->start_time_ns;
    u64 end_time = stat->end_time_ns;

    /* Google requirement restricts the interval between end time
     * and start time to be at most 1 second
     */
    bool discard = ((end_time - start_time) >
                        GPU_WORK_TIME_GAP_LIMIT_NS);
    stat->uid = 0;
    stat->end_time_ns = 0;
    __emit_work_period_event(stat, discard);

}

static inline u32 get_cur_dt(struct intel_context* ce)
{
    spin_lock(&ce->record.lock);
    u32 ts = READ_ONCE(ce->lrc_reg_state[CTX_TIMESTAMP]);
    s32 dt = ts - ce->record.last_ts;
    ce->record.last_ts = ts;
    spin_unlock(&ce->record.lock);

    if (unlikely(dt < 0))
        dt = 0;
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

    // Put this in caller
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
            emit_event_and_evict_slot(stat);
            spin_unlock(&stat->lock);
            key = idx;
        }
    }
    stat = &ew->stats[key];
    GEM_BUG_ON(stat->uid && (stat->uid != uid));
    u64 ctx_start_time =
        atomic64_read(&ce->record.start_time_ns);

    /*
     * If the uid at our hash index is empty (zero)
     * this implies that our ctx is processed for
     * the first time.
     *
     * So, we set the start time to the last time this
     * ctx was put into the active queue after emitting
     * its event. We also set the total active duration to
     * the current runtime of this ctx
     */
    spin_lock(&stat->lock);
    if (!stat->uid) {
        stat->uid = uid;
        stat->start_time_ns = ctx_start_time;
        stat->active_duration_ns =
                    get_active_duration_ns(ce);
        stat->end_time_ns = ktime_get_raw_ns();

        atomic_inc(&ew->num_entries);
        goto list_add;
    }

    /* Google requirement prohibits next start time to
     * overlap with previous end time for a given uid.
     * Skip the reuqests that don't match the requirement
     * until we get the desired new start time
     */
    u64 prev_start_time = stat->start_time_ns;
    u64 prev_end_time = stat->end_time_ns;
    if (!prev_start_time && ctx_start_time <= prev_end_time)
        goto out;

    /*
     * We set the endtime to the current time this ctx
     * is being processed and accumulate the current
     * runtime to the total active duration
     */
    stat->start_time_ns = prev_start_time?: ctx_start_time;
    stat->end_time_ns = ktime_get_raw_ns();
    stat->active_duration_ns +=
                get_active_duration_ns(ce);

    /* We limit the frequency of events to 10ms */
    unsigned long delta = jiffies - stat->jiffies;
    if (jiffies_to_msecs(delta) >=
                GPU_WORK_PERIOD_EVENT_TIMEOUT_MS)
    {
        emit_work_period_event(stat);
        stat->jiffies = jiffies;
        goto out;
    }

list_add:
    GEM_BUG_ON(stat->start_time_ns <= prev_end_time);
    if (list_empty(&ce->record.ws_link)) {
        /* This implies the context wasn't being tracked
         * until this point. Get a reference and add this
         * to the list to mark it as being tracked.
         */
        if (intel_context_get(ce)) {
            list_add(&ce->record.ws_link, &stat->contexts);
        }
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

        stat->gpu_id = engine->class;
        stat->uid = 0;
        stat->start_time_ns = 0;
        stat->end_time_ns = 0;
        stat->active_duration_ns = 0;
        stat->jiffies = 0;

        spin_lock_init(&stat->lock);
        INIT_LIST_HEAD(&stat->contexts);
    }

    /* Enable gpu work period */
    ew->enabled = true;
}

void i915_gpu_work_stats_fini(struct intel_engine_cs *engine)
{
    struct i915_engine_work *ew = &engine->gpu_work;

    ew->enabled = false;
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
