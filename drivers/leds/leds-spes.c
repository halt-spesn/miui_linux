// SPDX-License-Identifier: GPL-2.0
/*
 * Smooth flashlight brightness driver for Xiaomi spes (Redmi Note 11)
 *
 * Hardware: SGM3785-style flash LED driver IC with two control pins:
 *   ENF (pm6125_gpios 2) — Flash enable: rising edge triggers timed
 *                           high-current flash pulse (~500ms), then decays.
 *                           PMIC GPIO — requires sleeping I/O (SPMI bus).
 *   ENM (tlmm 85)        — Torch/movie enable: steady = full torch,
 *                           PWM input = duty-proportional torch current.
 *                           SoC GPIO — non-sleeping, safe in IRQ context.
 *
 * Implementation:
 *   - PWM on ENM via hrtimer: hardware timer interrupt toggles the GPIO
 *     at 10 kHz with sub-µs accuracy and zero CPU burn between edges.
 *   - ENF burst via workqueue: PMIC GPIO needs gpio_set_value_cansleep(),
 *     so re-trigger is handled in process context by a delayed work item.
 *
 * Brightness mapping (0–255):
 *   0       — off
 *   1–249   — PWM on ENM, duty = brightness/249
 *   250–255 — BURST: ENM on + ENF re-triggered every ~350ms
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>

#define PWM_PERIOD_NS	  100000	/* 10 kHz (100 µs) */
#define MIN_PULSE_NS	  5000		/* 5 µs floor */
#define BURST_THRESHOLD	  250
#define BURST_RETRIG_MS	  350		/* re-trigger before ~500ms decay */
#define BURST_LOW_MS	  10		/* 10ms low for IC reset + SPMI */

static int gpio_enf = -1;
static int gpio_enm = -1;
static bool enf_available;
static atomic_t target_brightness = ATOMIC_INIT(0);

/* --- PWM via hrtimer (runs in hard IRQ context) --- */
static struct hrtimer pwm_timer;
static int pwm_phase;		/* 1 = high phase, 0 = low phase */
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

/* --- Burst re-trigger (workqueue, process context) --- */
static struct delayed_work burst_work;

static void burst_work_fn(struct work_struct *work)
{
	int br;

	if (!enf_available)
		return;

	br = atomic_read(&target_brightness);
	if (br < BURST_THRESHOLD)
		return;

	/* Pull ENF low for IC reset */
	gpio_set_value_cansleep(gpio_enf, 0);
	msleep(BURST_LOW_MS);

	/* Re-check — brightness may have changed during sleep */
	br = atomic_read(&target_brightness);
	if (br < BURST_THRESHOLD)
		return;		/* leave ENF low — correct for non-burst */

	/* Rising edge triggers new flash pulse */
	gpio_set_value_cansleep(gpio_enf, 1);

	schedule_delayed_work(&burst_work,
			      msecs_to_jiffies(BURST_RETRIG_MS));
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
		cancel_delayed_work(&burst_work);
		gpio_set_value(gpio_enm, 0);
		if (enf_available)
			gpio_set_value_cansleep(gpio_enf, 0);
		goto check_race;
	}

	/* --- BURST (250–255) --- */
	if (br >= BURST_THRESHOLD) {
		hrtimer_cancel(&pwm_timer);
		gpio_set_value(gpio_enm, 1);
		if (enf_available) {
			cancel_delayed_work(&burst_work);
			gpio_set_value_cansleep(gpio_enf, 1);
			schedule_delayed_work(&burst_work,
					      msecs_to_jiffies(BURST_RETRIG_MS));
		}
		goto check_race;
	}

	/* --- PWM (1–249) --- */
	cancel_delayed_work(&burst_work);
	if (enf_available)
		gpio_set_value_cansleep(gpio_enf, 0);

	on_ns = (s64)PWM_PERIOD_NS * br / (BURST_THRESHOLD - 1);
	if (on_ns < MIN_PULSE_NS)
		on_ns = MIN_PULSE_NS;
	off_ns = PWM_PERIOD_NS - on_ns;
	if (off_ns < MIN_PULSE_NS)
		off_ns = MIN_PULSE_NS;

	timer_was_active = hrtimer_active(&pwm_timer);

	/* Update duty — hrtimer picks up new values on next edge */
	WRITE_ONCE(pwm_on_ns, on_ns);
	WRITE_ONCE(pwm_off_ns, off_ns);

	/* Start timer if not already running (PWM→PWM keeps running) */
	if (!timer_was_active) {
		gpio_set_value(gpio_enm, 1);
		pwm_phase = 1;
		hrtimer_start(&pwm_timer, ns_to_ktime(on_ns),
			      HRTIMER_MODE_REL);
	}

check_race:
	/* Catch brightness changes that arrived while we were working */
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

	gpio_enf = of_get_named_gpio(np, "qcom,flash-gpios", 0);
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

	/* ENF is optional — enables burst mode (250–255) */
	if (gpio_is_valid(gpio_enf)) {
		ret = gpio_request(gpio_enf, "flashlight_enf");
		if (ret) {
			pr_warn("flashlight: ENF GPIO unavailable (%d), burst disabled\n", ret);
			enf_available = false;
		} else {
			gpio_direction_output(gpio_enf, 0);
			enf_available = true;
		}
	}

	hrtimer_init(&pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pwm_timer.function = pwm_timer_cb;
	INIT_WORK(&update_work, update_work_fn);
	INIT_DELAYED_WORK(&burst_work, burst_work_fn);

	ret = led_classdev_register(NULL, &flashlight_led);
	if (ret) {
		pr_err("flashlight: LED class register failed (%d)\n", ret);
		goto err_free;
	}

	pr_info("flashlight: ready (hrtimer PWM 1-249, burst %s)\n",
		enf_available ? "250-255" : "disabled");
	return 0;

err_free:
	if (enf_available)
		gpio_free(gpio_enf);
	gpio_free(gpio_enm);
	return ret;
}

static void __exit flashlight_exit(void)
{
	led_classdev_unregister(&flashlight_led);
	hrtimer_cancel(&pwm_timer);
	cancel_work_sync(&update_work);
	cancel_delayed_work_sync(&burst_work);
	gpio_set_value(gpio_enm, 0);
	if (enf_available) {
		gpio_set_value_cansleep(gpio_enf, 0);
		gpio_free(gpio_enf);
	}
	gpio_free(gpio_enm);
}

late_initcall(flashlight_init);
module_exit(flashlight_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Smooth flashlight brightness via hrtimer PWM");
