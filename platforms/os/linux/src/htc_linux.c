#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/wakelock.h>
#include <linux/android_alarm.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>

struct alarm htc_linux_alarm;
struct wake_lock htc_linux_wake_lock;
struct workqueue_struct *htc_linux_wqueue;
struct delayed_work htc_linux_work;

DEFINE_MUTEX(htc_linux_lock);//Protect htc_linux_periodic_wakup_started
int htc_linux_periodic_wakup_started = 0;

int scan_interval[] = {1*60,1*70,1*80,2*60,2*70,2*80,4*60,4*70,4*80,8*60,8*70,8*80,15*60};
int scan_interval_size;

#define SCAN_INTERVAL_MAX 15*60

int htc_linux_scan_counter = 0;

void htc_linux_alarm_handler(struct alarm *alarm)
{
	unsigned long flags;

	// prevent suspend before holding wake lock
	local_irq_save(flags);
	wake_lock(&htc_linux_wake_lock);
	local_irq_restore(flags);
	printk("[htc_linux] Locking\n");

	htc_linux_scan_counter++;

	queue_delayed_work(htc_linux_wqueue, &htc_linux_work, HZ*20);
}

void htc_linux_work_handler(struct work_struct *work)
{
	ktime_t interval;
	ktime_t slack = ktime_set(20, 0);
	ktime_t next_alarm;

	if (htc_linux_scan_counter >= scan_interval_size) {
		interval = ktime_set(SCAN_INTERVAL_MAX, 0);
		//printk("[htc_linux] next interval: %d\n", SCAN_INTERVAL_MAX);
	} else {
		interval = ktime_set(scan_interval[htc_linux_scan_counter], 0);
		//printk("[htc_linux] next interval: %d\n", scan_interval[htc_linux_scan_counter]);
	}

	next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
	alarm_start_range(&htc_linux_alarm, next_alarm, ktime_add(next_alarm, slack));

	printk("[htc_linux] Unlocking\n");
	wake_unlock(&htc_linux_wake_lock);
}

int htc_linux_periodic_wakeup_start(void)
{
	unsigned long flags;

	mutex_lock(&htc_linux_lock);
	if (htc_linux_periodic_wakup_started == 0) {
		printk("[htc_linux] Starting\n");
		htc_linux_periodic_wakup_started = 1;
	} else {
		mutex_unlock(&htc_linux_lock);
		return 0;
	}

	// prevent suspend before holding wake lock
	local_irq_save(flags);
	wake_lock(&htc_linux_wake_lock);
	local_irq_restore(flags);
	printk("[htc_linux] Locking\n");

	queue_delayed_work(htc_linux_wqueue, &htc_linux_work, HZ*20);

	alarm_init(&htc_linux_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP, htc_linux_alarm_handler);

	mutex_unlock(&htc_linux_lock);
	return 0;
}

void htc_linux_periodic_wakeup_stop(void)
{
	mutex_lock(&htc_linux_lock);
	if (htc_linux_periodic_wakup_started == 1) {
		printk("[htc_linux] Stoping\n");
		htc_linux_periodic_wakup_started = 0;
		htc_linux_scan_counter = 0;
	} else {
		mutex_unlock(&htc_linux_lock);
		return;
	}

	alarm_cancel(&htc_linux_alarm);

	cancel_delayed_work_sync(&htc_linux_work);

	/*Additional unlocking for unfinished work inside work queue.*/
	printk("[htc_linux] Unlocking for unfinished work\n");
	wake_unlock(&htc_linux_wake_lock);

	mutex_unlock(&htc_linux_lock);
	return;
}

int htc_linux_periodic_wakeup_init(void)
{
	mutex_init(&htc_linux_lock);

	wake_lock_init(&htc_linux_wake_lock, WAKE_LOCK_SUSPEND, "htc_linux_alarm");

	INIT_DELAYED_WORK(&htc_linux_work, htc_linux_work_handler);
	htc_linux_wqueue = create_singlethread_workqueue("htc_linux_wqueue");
	if (!htc_linux_wqueue) {
		printk("[htc_linux] Creating htc_linux wqueue failed\n");
		return -ENOMEM;
	}

	scan_interval_size = sizeof(scan_interval)/sizeof(int);
	printk("[htc_linux] scan_interval_size=%d\n", scan_interval_size);

	return 0;
}

void htc_linux_periodic_wakeup_exit(void)
{
	mutex_lock(&htc_linux_lock);
	htc_linux_periodic_wakup_started = -1;
	mutex_unlock(&htc_linux_lock);

	destroy_workqueue(htc_linux_wqueue);

	wake_lock_destroy(&htc_linux_wake_lock);

	mutex_destroy(&htc_linux_lock);
}
