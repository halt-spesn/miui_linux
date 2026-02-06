// SPDX-License-Identifier: GPL-2.0
/*
 * Smooth flashlight brightness driver for Xiaomi spes (Redmi Note 11)
 *
 * Hardware: SGM3785-style flash LED driver IC with two control pins:
 *   ENF (pm6125_gpios 2) — Flash enable: rising edge triggers timed
 *                           high-current flash pulse (~500ms), then decays.
 *   ENM (tlmm 85)        — Torch/movie enable: steady = full torch,
 *                           PWM input = duty-proportional torch current.
 *
 * The IC's internal circuitry converts PWM duty on ENM directly to LED
 * current. PWM at ~10 kHz — within typical SGM3785 acceptance range.
 *
 * Brightness mapping (0–255):
 *   0       — off (both pins low, thread sleeps)
 *   1–249   — PWM on ENM, duty = brightness/249
 *   250–255 — BURST mode: ENM full on + ENF re-triggered every ~400ms
 *             to sustain the high-current flash pulse continuously.
 *             ENM provides base illumination during the brief ENF
 *             low→high re-trigger so there's no visible gap.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/ktime.h>

#define PWM_PERIOD_US	  100		/* 10 kHz  (100 µs period) */
#define MIN_PULSE_US	  5		/* 5 µs floor */
#define BURST_THRESHOLD	  250		/* brightness >= this enters burst */
#define BURST_RETRIG_MS	  350		/* re-trigger before ~500ms decay */
#define BURST_LOW_MS	  10		/* 10ms low pulse — SPMI needs time */

static int gpio_enf = -1;	/* ENF — flash enable (pm6125_gpios 2) */
static int gpio_enm = -1;	/* ENM — torch/movie  (tlmm 85)       */
static bool enf_available;
static atomic_t target_brightness = ATOMIC_INIT(0);
static struct task_struct *pwm_thread;

/* ---- software PWM kernel thread ---- */

static int pwm_thread_fn(void *data)
{
	ktime_t last_enf_trigger = 0;

	/* Elevated but not RT — avoids starving system processes */
	set_user_nice(current, -20);

	while (!kthread_should_stop()) {
		int br = atomic_read(&target_brightness);
		long on_us, off_us;

		/* --- OFF: all pins low, sleep until woken --- */
		if (br == 0) {
			if (enf_available)
				gpio_set_value(gpio_enf, 0);
			gpio_set_value(gpio_enm, 0);
			last_enf_trigger = 0;
			set_current_state(TASK_INTERRUPTIBLE);
			if (atomic_read(&target_brightness) == 0 &&
			    !kthread_should_stop())
				schedule();
			__set_current_state(TASK_RUNNING);
			continue;
		}

		/* --- BURST mode (250–255): ENM on + ENF re-trigger --- */
		if (br >= BURST_THRESHOLD && enf_available) {
			gpio_set_value(gpio_enm, 1);

			if (last_enf_trigger == 0 ||
			    ktime_ms_delta(ktime_get(), last_enf_trigger)
			    >= BURST_RETRIG_MS) {
				/*
				 * Pull ENF low long enough for the IC to
				 * fully reset, then re-trigger with rising edge.
				 */
				gpio_set_value(gpio_enf, 0);
				msleep(BURST_LOW_MS);
				gpio_set_value(gpio_enf, 1);
				last_enf_trigger = ktime_get();
			}

			/* Sleep a bit, then re-check brightness */
			usleep_range(20000, 25000);
			continue;
		}

		/* --- MAX torch (no burst available): ENM steady on --- */
		if (br >= BURST_THRESHOLD) {
			gpio_set_value(gpio_enm, 1);
			set_current_state(TASK_INTERRUPTIBLE);
			if (atomic_read(&target_brightness) >= BURST_THRESHOLD &&
			    !kthread_should_stop())
				schedule();
			__set_current_state(TASK_RUNNING);
			continue;
		}

		/* Leaving burst — turn off ENF */
		if (enf_available)
			gpio_set_value(gpio_enf, 0);
		last_enf_trigger = 0;

		/* --- PWM range (1–249) --- */
		on_us = PWM_PERIOD_US * br / (BURST_THRESHOLD - 1);
		off_us = PWM_PERIOD_US - on_us;

		if (on_us < MIN_PULSE_US)
			on_us = MIN_PULSE_US;
		if (off_us < MIN_PULSE_US)
			off_us = MIN_PULSE_US;

		/* High phase */
		gpio_set_value(gpio_enm, 1);
		usleep_range(on_us, on_us + 10);

		/* Low phase */
		gpio_set_value(gpio_enm, 0);
		usleep_range(off_us, off_us + 10);
	}

	if (enf_available)
		gpio_set_value(gpio_enf, 0);
	gpio_set_value(gpio_enm, 0);
	return 0;
}

/* ---- LED class interface ---- */

static void flashlight_brightness_set(struct led_classdev *led_cdev,
				       enum led_brightness brightness)
{
	atomic_set(&target_brightness, brightness);
	wake_up_process(pwm_thread);
}

static struct led_classdev flashlight_led = {
	.name		= "led:torch",
	.max_brightness	= 255,
	.brightness_set	= flashlight_brightness_set,
};

/* ---- init / exit ---- */

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

	pwm_thread = kthread_run(pwm_thread_fn, NULL, "flashlight_pwm");
	if (IS_ERR(pwm_thread)) {
		ret = PTR_ERR(pwm_thread);
		pr_err("flashlight: thread creation failed (%d)\n", ret);
		goto err_free;
	}

	ret = led_classdev_register(NULL, &flashlight_led);
	if (ret) {
		pr_err("flashlight: LED class register failed (%d)\n", ret);
		goto err_stop_thread;
	}

	pr_info("flashlight: smooth brightness ready (PWM 1-249, burst %s)\n",
		enf_available ? "250-255" : "disabled");
	return 0;

err_stop_thread:
	kthread_stop(pwm_thread);
err_free:
	gpio_free(gpio_enm);
	return ret;
}

static void __exit flashlight_exit(void)
{
	led_classdev_unregister(&flashlight_led);
	kthread_stop(pwm_thread);
	if (enf_available)
		gpio_free(gpio_enf);
	gpio_free(gpio_enm);
}

late_initcall(flashlight_init);
module_exit(flashlight_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Smooth flashlight brightness via software PWM");
