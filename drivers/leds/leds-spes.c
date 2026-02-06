// SPDX-License-Identifier: GPL-2.0
/*
 * Smooth flashlight brightness driver for Xiaomi spes (Redmi Note 11)
 *
 * Hardware: SGM3785-style flash LED driver IC, ENM pin (tlmm 85):
 *   Steady high = full torch; PWM = duty-proportional torch current.
 *   SoC GPIO — non-sleeping, safe in IRQ context.
 *
 * PWM on ENM via hrtimer at 10 kHz — sub-µs accuracy, zero CPU between edges.
 *
 * Brightness mapping (0–255):
 *   0   — off
 *   1–254 — PWM on ENM, duty = brightness/255
 *   255 — ENM steady on (max torch)
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>

#define PWM_PERIOD_NS	100000		/* 10 kHz (100 µs) */
#define MIN_PULSE_NS	5000		/* 5 µs floor */

static int gpio_enm = -1;
static atomic_t target_brightness = ATOMIC_INIT(0);

/* --- PWM via hrtimer (runs in hard IRQ context) --- */
static struct hrtimer pwm_timer;
static int pwm_phase;
static s64 pwm_on_ns, pwm_off_ns;

static enum hrtimer_restart pwm_timer_cb(struct hrtimer *timer)
{
	if (pwm_phase) {
		gpio_set_value(gpio_enm, 0);
		pwm_phase = 0;
		hrtimer_forward_now(timer,
				    ns_to_ktime(READ_ONCE(pwm_off_ns)));
	} else {
		gpio_set_value(gpio_enm, 1);
		pwm_phase = 1;
		hrtimer_forward_now(timer,
				    ns_to_ktime(READ_ONCE(pwm_on_ns)));
	}
	return HRTIMER_RESTART;
}

/* --- Brightness state machine (workqueue, process context) --- */
static struct work_struct update_work;

static void update_work_fn(struct work_struct *work)
{
	int br;
	s64 on_ns, off_ns;
	bool timer_was_active;

restart:
	br = atomic_read(&target_brightness);

	/* --- OFF --- */
	if (br == 0) {
		hrtimer_cancel(&pwm_timer);
		gpio_set_value(gpio_enm, 0);
		goto check_race;
	}

	/* --- MAX --- */
	if (br >= 255) {
		hrtimer_cancel(&pwm_timer);
		gpio_set_value(gpio_enm, 1);
		goto check_race;
	}

	/* --- PWM (1–254) --- */
	on_ns = (s64)PWM_PERIOD_NS * br / 255;
	if (on_ns < MIN_PULSE_NS)
		on_ns = MIN_PULSE_NS;
	off_ns = PWM_PERIOD_NS - on_ns;
	if (off_ns < MIN_PULSE_NS)
		off_ns = MIN_PULSE_NS;

	timer_was_active = hrtimer_active(&pwm_timer);

	WRITE_ONCE(pwm_on_ns, on_ns);
	WRITE_ONCE(pwm_off_ns, off_ns);

	if (!timer_was_active) {
		gpio_set_value(gpio_enm, 1);
		pwm_phase = 1;
		hrtimer_start(&pwm_timer, ns_to_ktime(on_ns),
			      HRTIMER_MODE_REL);
	}

check_race:
	if (atomic_read(&target_brightness) != br)
		goto restart;
}

/* --- LED class interface --- */

static void flashlight_brightness_set(struct led_classdev *led_cdev,
				       enum led_brightness brightness)
{
	atomic_set(&target_brightness, brightness);
	schedule_work(&update_work);
}

static struct led_classdev flashlight_led = {
	.name		= "led:torch",
	.max_brightness	= 255,
	.brightness_set	= flashlight_brightness_set,
};

/* --- init / exit --- */

static int __init flashlight_init(void)
{
	struct device_node *np;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "qcom,camera-flash");
	if (!np) {
		pr_err("flashlight: camera-flash DT node not found\n");
		return -ENODEV;
	}

	gpio_enm = of_get_named_gpio(np, "qcom,flash-gpios", 1);
	if (!gpio_is_valid(gpio_enm)) {
		pr_err("flashlight: invalid ENM GPIO (%d)\n", gpio_enm);
		return -EINVAL;
	}

	ret = gpio_request(gpio_enm, "flashlight_enm");
	if (ret) {
		pr_err("flashlight: ENM GPIO request failed (%d)\n", ret);
		return ret;
	}
	gpio_direction_output(gpio_enm, 0);

	hrtimer_init(&pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pwm_timer.function = pwm_timer_cb;
	INIT_WORK(&update_work, update_work_fn);

	ret = led_classdev_register(NULL, &flashlight_led);
	if (ret) {
		pr_err("flashlight: LED class register failed (%d)\n", ret);
		gpio_free(gpio_enm);
		return ret;
	}

	pr_info("flashlight: ready (hrtimer PWM @ 10 kHz)\n");
	return 0;
}

static void __exit flashlight_exit(void)
{
	led_classdev_unregister(&flashlight_led);
	hrtimer_cancel(&pwm_timer);
	cancel_work_sync(&update_work);
	gpio_set_value(gpio_enm, 0);
	gpio_free(gpio_enm);
}

late_initcall(flashlight_init);
module_exit(flashlight_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Smooth flashlight brightness via hrtimer PWM");
