#ifndef _LITMUS_ZERO_SLACK_H_
#define _LITMUS_ZERO_SLACK_H_

/* Update the per-processor enforcement timer (arm/reproram/cancel) for
 * the next task. */
void update_slack_enforcement_timer(struct task_struct* );

#endif
