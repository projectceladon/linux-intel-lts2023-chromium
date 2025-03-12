#ifndef __I915_GPU_WORK_H__
#define __I915_GPU_WORK_H__

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/list.h>

#define I915_ENGINE_WORK_STATS_COUNT (256)
#define GPU_WORK_PERIOD_EVENT_TIMEOUT_MS (10)
#define SEC_IN_NSEC (1000000000)
#define GPU_WORK_TIME_GAP_LIMIT_NS (SEC_IN_NSEC)

#define HASH_MAP(x) (x & (I915_ENGINE_WORK_STATS_COUNT - 1))
#define KEY_INVALID(key) (key < 0 || key >= I915_ENGINE_WORK_STATS_COUNT)
#define STAT_INVALID(stat) (!stat->start_time_ns || \
                            stat->start_time_ns >= stat->end_time_ns || \
                            !stat->active_duration_ns || \
                            (stat->end_time_ns - stat->start_time_ns) < \
                            stat->active_duration_ns)

struct intel_context;
struct intel_engine_cs;

struct i915_work_stats {
    u32 gpu_id;
    u32 uid;
    u64 start_time_ns;
    u64 end_time_ns;
    u64 active_duration_ns;

    /*
     * Rather than having a giant lock for the entire stats array
     * we keep a per slot lock to minimise lock contention
     */
    spinlock_t lock;
    /* List of contexts currently contributing to this uid */
    struct list_head contexts;
    /* Number of jiffies since we last emitted event for this uid */
    unsigned long jiffies;
};

struct i915_engine_work {
    /* Indicates if gpu work period is enabled */
    bool enabled;
    /* number of entries currently in work stats */
    atomic_t num_entries;
    /* work period stats record per engine */
    struct i915_work_stats stats[I915_ENGINE_WORK_STATS_COUNT];
};

void i915_gpu_work_process_ctx(struct intel_context *ctx, struct i915_engine_work *ew);

void i915_gpu_work_stats_init(struct intel_engine_cs *engine);
void i915_gpu_work_stats_fini(struct intel_engine_cs *engine);
#endif /*__I915_GPU_WORK_H__*/