#include <linux/sched.h>
#include <linux/percpu.h>
#include <linux/hrtimer.h>

#include <litmus/litmus.h>
#include <litmus/preempt.h>
#include <litmus/sched_plugin.h>
#include <litmus/zeroslack_timer.h>

struct slack_enforcement_timer {
	/* The slack_enforcement_timer is used to monitor zero slack instant */
	struct hrtimer		timer;
	int			armed;
};

DEFINE_PER_CPU(struct slack_enforcement_timer, slack_timer);

static enum hrtimer_restart on_zero_slack_instant(struct hrtimer *timer)
{
	struct slack_enforcement_timer* et = container_of(timer,
						    struct slack_enforcement_timer,
						    timer);
	unsigned long flags;

	local_irq_save(flags);
	et->armed = 0;

	TRACE("on_zero_slack_instant at %llu\n", litmus_clock());
	/* activate scheduler */
	litmus->set_global_criticality(HIGH);
	if (litmus->get_global_criticality() == HIGH)
		litmus_reschedule_local();
	local_irq_restore(flags);

	return  HRTIMER_NORESTART;
}

/* assumes called with IRQs off */
static void cancel_slack_enforcement_timer(struct slack_enforcement_timer* et)
{
	int ret;

	TRACE("cancelling slack instant enforcement timer at %llu\n", litmus_clock());

	/* Since interrupts are disabled and et->armed is only
	 * modified locally, we do not need any locks.
	 */

	if (et->armed) {
		ret = hrtimer_try_to_cancel(&et->timer);
		/* Should never be inactive. */
		BUG_ON(ret == 0);
		/* Should never be running concurrently. */
		BUG_ON(ret == -1);

		et->armed = 0;
	}
}

/* assumes called with IRQs off */
static void arm_slack_enforcement_timer(struct slack_enforcement_timer* et,
				  struct task_struct* t)
{
	lt_t when_to_fire;

	WARN_ONCE(!hrtimer_is_hres_active(&et->timer),
		KERN_ERR "WARNING: no high resolution timers available!?\n");
	/* __hrtimer_start_range_ns() cancels the timer
	 * anyway, so we don't have to check whether it is still armed */

	if (likely(!is_np(t)) && (litmus->get_zero_slack_instant())) {
		when_to_fire = litmus->get_zero_slack_instant();
		TRACE_TASK(t, "when_to_fire at arm_slack_enforcement_timer %llu\n", when_to_fire);
		__hrtimer_start_range_ns(&et->timer,
					 ns_to_ktime(when_to_fire),
					 0 /* delta */,
					 HRTIMER_MODE_ABS_PINNED,
					 0 /* no wakeup */);
		et->armed = 1;
		TRACE_TASK(t, "arming slack instant enforcement timer at %llu\n", litmus_clock());
	}
}

/* expects to be called with IRQs off */
void update_slack_enforcement_timer(struct task_struct* t)
{
	struct slack_enforcement_timer* et = &__get_cpu_var(slack_timer);

	if (t && (litmus->get_global_criticality() == LOW) && !(et->armed)) {
		/* Make sure we call into the scheduler when this slack
		 * expires. */
		arm_slack_enforcement_timer(et, t);
	}
}


static int __init init_slack_enforcement(void)
{
	int cpu;
	struct slack_enforcement_timer* et;

	for (cpu = 0; cpu < NR_CPUS; cpu++)  {
		et = &per_cpu(slack_timer, cpu);
		hrtimer_init(&et->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
		et->timer.function = on_zero_slack_instant;
	}
	return 0;
}

module_init(init_slack_enforcement);
