/*
 * litmus/sched_zsrm.c
 *
 * Implementation of Rate monotonic Scheduling.
 * Based on P-FP.
 */

#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <litmus/litmus.h>
#include <litmus/wait.h>
#include <litmus/jobs.h>
#include <litmus/preempt.h>
#include <litmus/fp_common.h>
#include <litmus/binheap.h>
#include <litmus/sched_plugin.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>
#include <litmus/budget.h>
#include <litmus/zeroslack_timer.h>

/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>
#include <linux/uaccess.h>

typedef struct {
        rt_domain_t             domain;
        struct fp_prio_queue    ready_queue;
        int                     cpu;
        int                     global_criticality;
        struct bheap            rm_heap;
        struct bheap            zs_heap;
        struct task_struct*     scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock

} zsrm_domain_t;

DEFINE_PER_CPU(zsrm_domain_t, zsrm_domains);

zsrm_domain_t* zsrm_doms[NR_CPUS];

#define local_zsrm               (&__get_cpu_var(zsrm_domains))
#define remote_dom(cpu)         (&per_cpu(zsrm_domains, cpu).domain)
#define remote_zsrm(cpu) (&per_cpu(zsrm_domains, cpu))
#define task_dom(task)          remote_dom(get_partition(task))
#define task_zsrm(task)          remote_zsrm(get_partition(task))


#ifdef CONFIG_LITMUS_LOCKING
DEFINE_PER_CPU(uint64_t,fmlp_timestamp1);
#endif

int rm_ready_order(struct bheap_node* a, struct bheap_node* b)
{ 
  struct task_struct *first_task;
  struct task_struct *second_task;

  first_task = bheap2task(a);
  second_task = bheap2task(b);

  if (get_rt_period(first_task) <= get_rt_period(second_task))
    return 1;
  else
    return 0;
}

int zs_ready_order(struct bheap_node* a, struct bheap_node* b)
{ 
  struct task_struct *first_task;
  struct task_struct *second_task;

  first_task = bheap2task(a);
  second_task = bheap2task(b);

  if ((get_rt_relative_zero_slack(first_task) + get_release(first_task)) <= (get_rt_relative_zero_slack(second_task) + get_release(second_task)))
    return 1;
  else
    return 0;
}

static int zsrm_get_criticality(void) 
{
    zsrm_domain_t*   zsrm = local_zsrm;
    return zsrm->global_criticality;
}

static void zsrm_set_criticality(int mode) 
{
    zsrm_domain_t*   zsrm = local_zsrm;
    struct task_struct* t;
    struct bheap_node* hn;
    
    if(!bheap_empty(&zsrm->zs_heap))
    {
        hn = bheap_take(zs_ready_order, &zsrm->zs_heap);
        t = bheap2task(hn);
        if((is_job_running(t))){
            TRACE_TASK(t,"Criticality mode changed to %d\n", mode);
            zsrm->global_criticality = mode;    
        }
        else{
            TRACE_TASK(t,"Job completed before zero_slack_instant\n");
        }
        bheap_node_free(hn);
    }
}

static lt_t zsrm_get_zero_slack_instant(void)
{
    zsrm_domain_t*   zsrm = local_zsrm;
    struct task_struct* t;
    struct bheap_node* hn;

    if(!bheap_empty(&zsrm->zs_heap))
    {
        hn = bheap_peek(zs_ready_order, &zsrm->zs_heap);
        t = bheap2task(hn);
        return (get_rt_relative_zero_slack(t) + get_release(t));
    }
    else
        return 0;
}

static void update_mode_to_userspace(struct task_struct* t)
{   
    struct control_page* cp;
    zsrm_domain_t*   zsrm = local_zsrm;

    if(has_control_page(t)){
        cp  = get_control_page(t);
        cp->active_crit = zsrm->global_criticality;
        if((zsrm->global_criticality) && (get_criticality(t) == HIGH)){
            get_exec_cost(t) = get_exec_cost_hi(t);
            TRACE_TASK(t,"get_exec_cost_hi = %llu\n", get_exec_cost(t));
        }
    }
    else{
        TRACE_TASK(t,"control_page set error\n");
    }
}

/* we assume the lock is being held */
static void preempt(zsrm_domain_t *zsrm)
{
        preempt_if_preemptable(zsrm->scheduled, zsrm->cpu);
}

static unsigned int priority_index(struct task_struct* t)
{
#ifdef CONFIG_LITMUS_LOCKING
        if (unlikely(t->rt_param.inh_task))
                /* use effective priority */
                t = t->rt_param.inh_task;

        if (is_priority_boosted(t)) {
                /* zero is reserved for priority-boosted tasks */
                return 0;
        } else
#endif
                return get_priority(t);
}

static void zsrm_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
        zsrm_domain_t *zsrm = container_of(rt, zsrm_domain_t, domain);
        unsigned long flags;
        struct task_struct* t;
        struct bheap_node* hn;

        raw_spin_lock_irqsave(&zsrm->slock, flags);

        while (!bheap_empty(tasks)) {
            hn = bheap_take(fp_ready_order, tasks);
            t = bheap2task(hn);
            if (zsrm->global_criticality <= get_criticality(t)){
                    TRACE_TASK(t, "released (part:%d prio:%u time_rel: %llu zero_slack: %llu\n",
                       get_partition(t), get_priority(t), get_release(t), get_rt_relative_zero_slack(t));
                    fp_prio_add(&zsrm->ready_queue, t, priority_index(t));
                    tsk_rt(t)->zs_node = bheap_node_alloc(GFP_ATOMIC);
                    if (likely(tsk_rt(t)->zs_node)) {
                        bheap_node_init(&tsk_rt(t)->zs_node, (void*)(t));
                        bheap_insert(zs_ready_order, &zsrm->zs_heap, tsk_rt(t)->zs_node);
                        get_job_completed(t) = 0;
                    }
            }
            else{
                /*Active low criticality tasks has been blocked, while adding to ready or release queue*/
            }
        }

        /* do we need to preempt? */
        if (fp_higher_prio(fp_prio_peek(&zsrm->ready_queue), zsrm->scheduled)) {
                TRACE_CUR("preempted by new release\n");
                preempt(zsrm);
        }

        raw_spin_unlock_irqrestore(&zsrm->slock, flags);
}

static void zsrm_preempt_check(zsrm_domain_t *zsrm)
{
        if (fp_higher_prio(fp_prio_peek(&zsrm->ready_queue), zsrm->scheduled))
                preempt(zsrm);
}

static void zsrm_domain_init(zsrm_domain_t* zsrm,
                               int cpu)
{
        fp_domain_init(&zsrm->domain, NULL, zsrm_release_jobs);
        zsrm->cpu                = cpu;
        zsrm->scheduled          = NULL;
        fp_prio_queue_init(&zsrm->ready_queue);
        zsrm->global_criticality = LOW;
        bheap_init(&zsrm->rm_heap);
        bheap_init(&zsrm->zs_heap);
}

static void requeue(struct task_struct* t, zsrm_domain_t *zsrm)
{
        BUG_ON(!is_running(t));
        tsk_rt(t)->completed = 0;
        if (zsrm->global_criticality <= get_criticality(t)) {
            if (is_released(t, litmus_clock())) {
                    fp_prio_add(&zsrm->ready_queue, t, priority_index(t));
            }
            else{
                    add_release(&zsrm->domain, t); /* it has got to wait */
            }
        }
        else{
            /*Active low criticality tasks has been blocked, while adding to ready or release queue*/
        }
}

static void job_completion(struct task_struct* t, int forced)
{
        sched_trace_task_completion(t,forced);

        tsk_rt(t)->completed = 0;
        get_job_completed(t) = 1;
        TRACE_TASK(t, "job_completion() %llu execution time = %llu\n", litmus_clock(), get_exec_time(t));
        prepare_for_next_period(t);
        if (is_released(t, litmus_clock()))
                sched_trace_task_release(t);
}

static struct task_struct* zsrm_schedule(struct task_struct * prev)
{
        zsrm_domain_t*   zsrm = local_zsrm;
        struct task_struct*     next;

        int out_of_time, sleep, preempt, np, exists, blocks, resched, migrate, mode, flag;

        raw_spin_lock(&zsrm->slock);

        /* sanity checking
         * differently from gedf, when a task exits (dead)
         * zsrm->schedule may be null and prev _is_ realtime
         */
        BUG_ON(zsrm->scheduled && zsrm->scheduled != prev);
        BUG_ON(zsrm->scheduled && !is_realtime(prev));

        /* (0) Determine state */
        exists      = zsrm->scheduled != NULL;
        blocks      = exists && !is_running(zsrm->scheduled);
        out_of_time = exists && budget_enforced(zsrm->scheduled) &&
                                budget_exhausted(zsrm->scheduled);
        np          = exists && is_np(zsrm->scheduled);
        sleep       = exists && is_completed(zsrm->scheduled);
        migrate     = exists && get_partition(zsrm->scheduled) != zsrm->cpu;
        preempt     = !blocks && (migrate || fp_preemption_needed(&zsrm->ready_queue, prev));
        mode        = zsrm->global_criticality;

        /* If we need to preempt do so.
         * The following checks set resched to 1 in case of special
         * circumstances.
         */
        resched = preempt;

        /* If a task blocks we have no choice but to reschedule.
         */
        if (blocks)
                resched = 1;

        /* Request a sys_exit_np() call if we would like to preempt but cannot.
         * Multiple calls to request_exit_np() don't hurt.
         */
        if (np && (out_of_time || preempt || sleep))
                request_exit_np(zsrm->scheduled);

        /* Any task that is preemptable and either exhausts its execution
         * budget or wants to sleep completes. We may have to reschedule after
         * this.
         */
        
        if (!np && (out_of_time || sleep) && !blocks && !migrate) {
                job_completion(zsrm->scheduled, !sleep);
                resched = 1;
        }

        if (exists && !np && mode && (mode > get_criticality(zsrm->scheduled))) {
            TRACE_TASK(zsrm->scheduled, "LOW criticality job is moved to complete state\n");
            job_completion(zsrm->scheduled, !sleep);
            resched = 1;
        }

        /* The final scheduling decision. Do we need to switch for some reason?
         * Switch if we are in RT mode and have no task or if we need to
         * resched.
         */
        next = NULL;
        if ((!np || blocks) && (resched || !exists)) {
                /* When preempting a task that does not block, then
                 * re-insert it into either the ready queue or the
                 * release queue (if it completed). requeue() picks
                 * the appropriate queue.
                 */
                if (zsrm->scheduled && !blocks  && !migrate)
                        requeue(zsrm->scheduled, zsrm);
                flag = 1;
                while(flag){
                    next = fp_prio_take(&zsrm->ready_queue);
                    if(next){
                        if(mode && get_criticality(next) == LOW){
                            TRACE_TASK(next, "LOW critic job dropped 1\n"); 
                        }
                        else{
                            update_mode_to_userspace(next);
                            flag = 0;
                        }
                    }
                    else
                        flag = 0;
                }
                if (next == prev) {
                        struct task_struct *t = fp_prio_peek(&zsrm->ready_queue);
                        TRACE_TASK(next, "next==prev sleep=%d oot=%d np=%d preempt=%d migrate=%d "
                                   "boost=%d empty=%d prio-idx=%u prio=%u\n",
                                   sleep, out_of_time, np, preempt, migrate,
                                   is_priority_boosted(next),
                                   t == NULL,
                                   priority_index(next),
                                   get_priority(next));
                        if (t)
                                TRACE_TASK(t, "waiter boost=%d prio-idx=%u prio=%u\n",
                                           is_priority_boosted(t),
                                           priority_index(t),
                                           get_priority(t));
                }
                /* If preempt is set, we should not see the same task again. */
                BUG_ON(preempt && next == prev);
                /* Similarly, if preempt is set, then next may not be NULL,
                 * unless it's a migration. */
                BUG_ON(preempt && !migrate && next == NULL);
        }
        else {
            /* Only override Linux scheduler if we have a real-time task
             * scheduled that needs to continue.
             */
            if (exists)
                next = prev;
        }

        if (next) {
                TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
        } else {
                //TRACE("becoming idle at %llu\n", litmus_clock());
        }

        zsrm->scheduled = next;
        sched_state_task_picked();
        raw_spin_unlock(&zsrm->slock);

        return next;
}

#ifdef CONFIG_LITMUS_LOCKING

/* prev is no longer scheduled --- see if it needs to migrate */
static void zsrm_finish_switch(struct task_struct *prev)
{
        zsrm_domain_t *to;

        if (is_realtime(prev) &&
            is_running(prev) &&
            get_partition(prev) != smp_processor_id()) {
                TRACE_TASK(prev, "needs to migrate from P%d to P%d\n",
                           smp_processor_id(), get_partition(prev));

                to = task_zsrm(prev);

                raw_spin_lock(&to->slock);

                TRACE_TASK(prev, "adding to queue on P%d\n", to->cpu);
                requeue(prev, to);
                if (fp_preemption_needed(&to->ready_queue, to->scheduled))
                        preempt(to);

                raw_spin_unlock(&to->slock);

        }
}

#endif

/*      Prepare a task for running in RT mode
 */
static void zsrm_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
        zsrm_domain_t*   zsrm = task_zsrm(t);
        unsigned long           flags;

        TRACE_TASK(t, "P-ZSRM: task new, cpu = %d\n",
                   t->rt_param.task_params.cpu);

        /* setup job parameters */
        release_at(t, litmus_clock());

        /*Add task to RM priority assignment - binary heap*/
        bheap_add(rm_ready_order, &zsrm->rm_heap, (void *)t, GFP_ATOMIC);
        update_mode_to_userspace(t);

        raw_spin_lock_irqsave(&zsrm->slock, flags);
        if (is_scheduled) {
                /* there shouldn't be anything else running at the time */
                BUG_ON(zsrm->scheduled);
                zsrm->scheduled = t;
        } else if (is_running(t)) {
                requeue(t, zsrm);
                /* maybe we have to reschedule */
                zsrm_preempt_check(zsrm);
        }
        raw_spin_unlock_irqrestore(&zsrm->slock, flags);
}

static void zsrm_task_wake_up(struct task_struct *task)
{
        unsigned long           flags;
        zsrm_domain_t*           zsrm = task_zsrm(task);
        lt_t                    now;
        struct task_struct *t;
        struct bheap_node* rm_hn;
        unsigned long long prio = 1;

        while (!bheap_empty(&zsrm->rm_heap)) {
                rm_hn = bheap_take(rm_ready_order, &zsrm->rm_heap);
                t = bheap2task(rm_hn);
                get_priority(t) = prio;
                prio++;
                bheap_node_free(rm_hn);
                TRACE_TASK(t, "changing prio:%d\n", get_priority(t));
        }

        TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
        raw_spin_lock_irqsave(&zsrm->slock, flags);

#ifdef CONFIG_LITMUS_LOCKING
        /* Should only be queued when processing a fake-wake up due to a
         * migration-related state change. */
        if (unlikely(is_queued(task))) {
                TRACE_TASK(task, "WARNING: waking task still queued. Is this right?\n");
                goto out_unlock;
        }
#else
        BUG_ON(is_queued(task));
#endif
        now = litmus_clock();
        if (is_sporadic(task) && is_tardy(task, now)
#ifdef CONFIG_LITMUS_LOCKING
        /* We need to take suspensions because of semaphores into
         * account! If a job resumes after being suspended due to acquiring
         * a semaphore, it should never be treated as a new job release.
         */
            && !is_priority_boosted(task)
#endif
                ) {
                /* new sporadic release */
                release_at(task, now);
                sched_trace_task_release(task);
        }

        /* Only add to ready queue if it is not the currently-scheduled
         * task. This could be the case if a task was woken up concurrently
         * on a remote CPU before the executing CPU got around to actually
         * de-scheduling the task, i.e., wake_up() raced with schedule()
         * and won. Also, don't requeue if it is still queued, which can
         * happen under the DPCP due wake-ups racing with migrations.
         */
        if (zsrm->scheduled != task) {
                requeue(task, zsrm);
                zsrm_preempt_check(zsrm);
        }

#ifdef CONFIG_LITMUS_LOCKING
out_unlock:
#endif
        raw_spin_unlock_irqrestore(&zsrm->slock, flags);
        TRACE_TASK(task, "wake up done\n");
}

static void zsrm_task_block(struct task_struct *t)
{
        /* only running tasks can block, thus t is in no queue */
        TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

        BUG_ON(!is_realtime(t));

        /* If this task blocked normally, it shouldn't be queued. The exception is
         * if this is a simulated block()/wakeup() pair from the pull-migration code path.
         * This should only happen if the DPCP is being used.
         */
#ifdef CONFIG_LITMUS_LOCKING
        if (unlikely(is_queued(t)))
                TRACE_TASK(t, "WARNING: blocking task still queued. Is this right?\n");
#else
        BUG_ON(is_queued(t));
#endif
}

static void zsrm_task_exit(struct task_struct * t)
{
        unsigned long flags;
        zsrm_domain_t*   zsrm = task_zsrm(t);
        rt_domain_t*            dom;

        raw_spin_lock_irqsave(&zsrm->slock, flags);
        if (is_queued(t)) {
                BUG(); /* This currently doesn't work. */
                /* dequeue */
                dom  = task_dom(t);
                remove(dom, t);
        }
        if (zsrm->scheduled == t) {
                zsrm->scheduled = NULL;
                preempt(zsrm);
        }
        TRACE_TASK(t, "RIP, now reschedule\n");

        raw_spin_unlock_irqrestore(&zsrm->slock, flags);
}

static long zsrm_admit_task(struct task_struct* tsk)
{
        if (task_cpu(tsk) == tsk->rt_param.task_params.cpu &&
#ifdef CONFIG_RELEASE_MASTER
            /* don't allow tasks on release master CPU */
            task_cpu(tsk) != remote_dom(task_cpu(tsk))->release_master &&
#endif
            litmus_is_valid_fixed_prio(get_priority(tsk)))
                return 0;
        else
                return -EINVAL;
}

static struct domain_proc_info zsrm_domain_proc_info;
static long zsrm_get_domain_proc_info(struct domain_proc_info **ret)
{
        *ret = &zsrm_domain_proc_info;
        return 0;
}

static void zsrm_setup_domain_proc(void)
{
        int i, cpu;
        int release_master =
#ifdef CONFIG_RELEASE_MASTER
                atomic_read(&release_master_cpu);
#else
                NO_CPU;
#endif
        int num_rt_cpus = num_online_cpus() - (release_master != NO_CPU);
        struct cd_mapping *cpu_map, *domain_map;

        memset(&zsrm_domain_proc_info, sizeof(zsrm_domain_proc_info), 0);
        init_domain_proc_info(&zsrm_domain_proc_info, num_rt_cpus, num_rt_cpus);
        zsrm_domain_proc_info.num_cpus = num_rt_cpus;
        zsrm_domain_proc_info.num_domains = num_rt_cpus;
        for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
                if (cpu == release_master)
                        continue;
                cpu_map = &zsrm_domain_proc_info.cpu_to_domains[i];
                domain_map = &zsrm_domain_proc_info.domain_to_cpus[i];

                cpu_map->id = cpu;
                domain_map->id = i; /* enumerate w/o counting the release master */
                cpumask_set_cpu(i, cpu_map->mask);
                cpumask_set_cpu(cpu, domain_map->mask);
                ++i;
        }
}

static long zsrm_activate_plugin(void)
{
#if defined(CONFIG_RELEASE_MASTER) || defined(CONFIG_LITMUS_LOCKING)
        int cpu;
#endif

#ifdef CONFIG_RELEASE_MASTER
        for_each_online_cpu(cpu) {
                remote_dom(cpu)->release_master = atomic_read(&release_master_cpu);
        }
#endif

#ifdef CONFIG_LITMUS_LOCKING
        for_each_online_cpu(cpu) {
                zsrm_doms[cpu] = remote_zsrm(cpu);
        }
#endif

        zsrm_setup_domain_proc();

        return 0;
}

static long zsrm_deactivate_plugin(void)
{
        destroy_domain_proc_info(&zsrm_domain_proc_info);
        return 0;
}

/*      Plugin object   */
static struct sched_plugin zsrm_plugin __cacheline_aligned_in_smp = {
        .plugin_name            = "P-ZSRM",
        .task_new               = zsrm_task_new,
        .complete_job           = complete_job,
        .task_exit              = zsrm_task_exit,
        .schedule               = zsrm_schedule,
        .task_wake_up           = zsrm_task_wake_up,
        .task_block             = zsrm_task_block,
        .admit_task             = zsrm_admit_task,
        .activate_plugin        = zsrm_activate_plugin,
        .deactivate_plugin      = zsrm_deactivate_plugin,
        .get_domain_proc_info   = zsrm_get_domain_proc_info,
#ifdef CONFIG_LITMUS_LOCKING
        .finish_switch          = zsrm_finish_switch,
        .get_global_criticality = zsrm_get_criticality,
        .set_global_criticality = zsrm_set_criticality,
        .get_zero_slack_instant = zsrm_get_zero_slack_instant,
#endif
};

static int __init init_zsrm(void)
{
        int i;

        /* We do not really want to support cpu hotplug, do we? ;)
         * However, if we are so crazy to do so,
         * we cannot use num_online_cpu()
         */
        for (i = 0; i < num_online_cpus(); i++) {
                zsrm_domain_init(remote_zsrm(i), i);
        }
        return register_sched_plugin(&zsrm_plugin);
}

module_init(init_zsrm);
