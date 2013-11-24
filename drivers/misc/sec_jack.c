#if 1 // set if 0 for froyo source
/*  drivers/misc/sec_jack.c
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/sec_jack.h>

#define MAX_ZONE_LIMIT		10
/* keep this value if you support double-pressed concept */
#define SEND_KEY_CHECK_TIME_MS	20		/* 20ms */
#define DET_CHECK_TIME_MS	50		/* 50ms to remove path change noise */
#define WAKE_LOCK_TIME		(HZ * 5)	/* 5 sec */
#define NUM_INPUT_DEVICE_ID	2

static struct class *jack_class;
static struct device *jack_dev;

struct sec_jack_info {
	struct sec_jack_platform_data *pdata;
	struct delayed_work jack_detect_work;
	struct work_struct buttons_work;
	struct workqueue_struct *queue;
	struct input_dev *input_dev;
	struct wake_lock det_wake_lock;
	struct sec_jack_zone *zone;
	struct input_handler handler;
	struct input_handle handle;
	struct input_device_id ids[NUM_INPUT_DEVICE_ID];
	int det_irq;
	int dev_id;
	int pressed;
	int pressed_code;
	struct platform_device *send_key_dev;
	unsigned int cur_jack_type;
};

/* with some modifications like moving all the gpio structs inside
 * the platform data and getting the name for the switch and
 * gpio_event from the platform data, the driver could support more than
 * one headset jack, but currently user space is looking only for
 * one key file and switch for a headset so it'd be overkill and
 * untestable so we limit to one instantiation for now.
 */
static atomic_t instantiated = ATOMIC_INIT(0);

/* sysfs name HeadsetObserver.java looks for to track headset state
 */
struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

/* To support AT+FCESTEST=1 */
struct switch_dev switch_sendend = {
		.name = "send_end",
};

static struct gpio_event_direct_entry sec_jack_key_map[] = {
	{
		.code	= KEY_UNKNOWN,
	},
};

static struct gpio_event_input_info sec_jack_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.type = EV_KEY,
	.debounce_time.tv.nsec = SEND_KEY_CHECK_TIME_MS * NSEC_PER_MSEC,
	.keymap = sec_jack_key_map,
	.keymap_size = ARRAY_SIZE(sec_jack_key_map)
};

static struct gpio_event_info *sec_jack_input_info[] = {
	&sec_jack_key_info.info,
};

static struct gpio_event_platform_data sec_jack_input_data = {
	.name = "sec_jack",
	.info = sec_jack_input_info,
	.info_count = ARRAY_SIZE(sec_jack_input_info),
};

#if 1 // for S1-KOR mic_bias and ear_mic_bias
static int recording_status=0;
static unsigned int g_cur_jack_type=0;
static unsigned int g_cur_jack_check=0;

unsigned int get_headset_status(int *type, int *check)
{
	//pr_info("[JACK] %s : type = %#x, check = %d\n", __func__, g_cur_jack_type, g_cur_jack_check);
	//return g_cur_jack_type;

	*type = (int)g_cur_jack_type;
	*check= (int)g_cur_jack_check;

	return 0;
}
EXPORT_SYMBOL(get_headset_status);

void set_recording_status(int value)
{
	recording_status = value;
}
EXPORT_SYMBOL(set_recording_status);

static int get_recording_status(void)
{
	return recording_status;
}
#endif // for S1-KOR mic_bias and ear_mic_bias

/* gpio_input driver does not support to read adc value.
 * We use input filter to support 3-buttons of headset
 * without changing gpio_input driver.
 */
static bool sec_jack_buttons_filter(struct input_handle *handle,
				    unsigned int type, unsigned int code,
				    int value)
{
	struct sec_jack_info *hi = handle->handler->private;

	if (type != EV_KEY || code != KEY_UNKNOWN)
		return false;

	hi->pressed = value;

	/* This is called in timer handler of gpio_input driver.
	 * We use workqueue to read adc value.
	 */
	queue_work(hi->queue, &hi->buttons_work);

	return true;
}

static int sec_jack_buttons_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	struct sec_jack_info *hi;
	struct sec_jack_platform_data *pdata;
	struct sec_jack_buttons_zone *btn_zones;
	int err;
	int i;

	//pr_info("%s A\n", __func__);

	/* bind input_handler to input device related to only sec_jack */
	if (dev->name != sec_jack_input_data.name)
		return -ENODEV;

	//pr_info("%s B\n", __func__);

	hi = handler->private;
	pdata = hi->pdata;
	btn_zones = pdata->buttons_zones;

	hi->input_dev = dev;
	hi->handle.dev = dev;
	hi->handle.handler = handler;
	hi->handle.open = 0;
	hi->handle.name = "sec_jack_buttons";

	err = input_register_handle(&hi->handle);
	if (err) {
		pr_err("%s: Failed to register sec_jack buttons handle, "
			"error %d\n", __func__, err);
		goto err_register_handle;
	}

	err = input_open_device(&hi->handle);
	if (err) {
		pr_err("%s: Failed to open input device, error %d\n",
			__func__, err);
		goto err_open_device;
	}

	for (i = 0; i < pdata->num_buttons_zones; i++)
		input_set_capability(dev, EV_KEY, btn_zones[i].code);

	return 0;

 err_open_device:
	input_unregister_handle(&hi->handle);
 err_register_handle:

	return err;
}

static void sec_jack_buttons_disconnect(struct input_handle *handle)
{
	//pr_info("%s\n", __func__);
	input_close_device(handle);
	input_unregister_handle(handle);
}

static void sec_jack_set_type(struct sec_jack_info *hi, int jack_type)
{
	struct sec_jack_platform_data *pdata = hi->pdata;

	/* this can happen during slow inserts where we think we identified
	 * the type but then we get another interrupt and do it again
	 */
	if (jack_type == hi->cur_jack_type) // skip state check without changing
		return;

    if (hi->cur_jack_type) // if jack was inserted already,
    {
        if (jack_type) // type changing between 3P & 4P is error.
        {
            printk(KERN_ERR "SEC JACK: Wrong input(%d->%d). skip state changing !!!\n", hi->cur_jack_type, jack_type);
	        return;
        }
    }

	if (jack_type == SEC_HEADSET_4POLE) {
		/* for a 4 pole headset, enable detection of send/end key */
		if (hi->send_key_dev == NULL)
			/* enable to get events again */
			hi->send_key_dev = platform_device_register_data(NULL,
					GPIO_EVENT_DEV_NAME,
					hi->dev_id,
					&sec_jack_input_data,
					sizeof(sec_jack_input_data));
		/* for sleep current after 4 pole ear jack inserted */
		//pdata->set_ear_ldo_state(false);
	} else {
		/* for all other jacks, disable send/end key detection */
		if (hi->send_key_dev != NULL) {
			/* disable to prevent false events on next insert */
			platform_device_unregister(hi->send_key_dev);
			hi->send_key_dev = NULL;
		}
		/* micbias is left enabled for 4pole and disabled otherwise */
		pdata->set_micbias_state(false);
	}

	hi->cur_jack_type = jack_type;
	g_cur_jack_type = jack_type;
	pr_info("%s : jack_type = %d\n", __func__, jack_type);

	switch_set_state(&switch_jack_detection, jack_type);

	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);
}

static void handle_jack_not_inserted(struct sec_jack_info *hi)
{
	//pr_info("%s\n", __func__);
	sec_jack_set_type(hi, SEC_JACK_NO_DEVICE);
	hi->pdata->set_micbias_state(false);
	/* for sleep current after 3 pole ear jack inserted */
	//hi->pdata->set_ear_ldo_state(true);
}

static void determine_jack_type(struct sec_jack_info *hi)
{
	struct sec_jack_platform_data *pdata = hi->pdata;
	struct sec_jack_zone *zones = pdata->zones;
	int size = pdata->num_zones;
	int count[MAX_ZONE_LIMIT] = {0};
	int adc;
	int i;
	unsigned npolarity = !pdata->det_active_high;

	//pr_info("%s A\n", __func__);

	/* set mic bias to enable adc */
	pdata->set_micbias_state(true);

	//pr_info("%s B\n", __func__);

	while (gpio_get_value(pdata->det_gpio) ^ npolarity) {
        g_cur_jack_check = 1;
		adc = pdata->get_adc_value();
		//pr_info("%s: adc = %d\n", __func__, adc);

		/* determine the type of headset based on the
		 * adc value.  An adc value can fall in various
		 * ranges or zones.  Within some ranges, the type
		 * can be returned immediately.  Within others, the
		 * value is considered unstable and we need to sample
		 * a few more types (up to the limit determined by
		 * the range) before we return the type for that range.
		 */
		for (i = 0; i < size; i++) {
			if (adc <= zones[i].adc_high) {
				if (++count[i] > zones[i].check_count) {
					sec_jack_set_type(hi,
							  zones[i].jack_type);
					g_cur_jack_check = 0;
					return;
				}
				msleep(zones[i].delay_ms);
				break;
			}
		}
	}
	/* jack removed before detection complete */
	//pr_info("%s : jack removed before detection complete\n", __func__);
	g_cur_jack_check = 0;
	handle_jack_not_inserted(hi);
}

/* thread run whenever the headset detect state changes (either insertion
 * or removal).
 */
static irqreturn_t sec_jack_detect_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = DET_CHECK_TIME_MS;
	unsigned npolarity = !pdata->det_active_high;

	//pr_info("%s A\n", __func__);
	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

	/* debounce headset jack.  don't try to determine the type of
	 * headset until the detect state is true for a while.
	 */
	while (time_left_ms > 0) {
		if (!(gpio_get_value(pdata->det_gpio) ^ npolarity)) {
			/* jack not detected. */
			handle_jack_not_inserted(hi);
			return IRQ_HANDLED;
		}
		msleep(10);
		time_left_ms -= 10;
	}

	//pr_info("%s B\n", __func__);

	/* jack presence was detected the whole time, figure out which type */
	determine_jack_type(hi);
	return IRQ_HANDLED;
}

/* thread run whenever the button of headset is pressed or released */
void sec_jack_buttons_work(struct work_struct *work)
{
	struct sec_jack_info *hi =
		container_of(work, struct sec_jack_info, buttons_work);
	struct sec_jack_platform_data *pdata = hi->pdata;
	struct sec_jack_buttons_zone *btn_zones = pdata->buttons_zones;
	int adc;
	int i;

	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

	/* when button is released */
	if (hi->pressed == 0) {
		input_report_key(hi->input_dev, hi->pressed_code, 0);
		switch_set_state(&switch_sendend, 0);
		input_sync(hi->input_dev);
		//pr_info("%s: keycode=%d, is released\n", __func__, hi->pressed_code);
		pr_info("%s: called\n", __func__);
		return;
	}

	/* when button is pressed */
	adc = pdata->get_adc_value();

	for (i = 0; i < pdata->num_buttons_zones; i++)
		if (adc >= btn_zones[i].adc_low &&
		    adc <= btn_zones[i].adc_high) {
			hi->pressed_code = btn_zones[i].code;
			input_report_key(hi->input_dev, btn_zones[i].code, 1);
			switch_set_state(&switch_sendend, 1);
			input_sync(hi->input_dev);
			pr_info("%s: keycode=%d, is pressed\n", __func__, btn_zones[i].code);
			pr_info("%s: done\n", __func__);
			return;
		}

	pr_warn("%s: key is skipped. ADC value is %d\n", __func__, adc);
}

static ssize_t select_jack_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	
	pr_info("%s\n", __func__);

	pr_info("%s : operate nothing\n", __func__);

	pr_info("%s hi->cur_jack_type = %#x \n", __func__, hi->cur_jack_type);

	return hi->cur_jack_type;
}       
static ssize_t select_jack_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;

	//pr_info("%s\n", __func__);

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);
	if (value == SEC_HEADSET_4POLE) {
		pdata->set_micbias_state(true);
		msleep(100);
	}

	sec_jack_set_type(hi, value);

	return size;
}

static DEVICE_ATTR(select_jack, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
	select_jack_show, select_jack_store);

static int sec_jack_probe(struct platform_device *pdev)
{
	struct sec_jack_info *hi;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	pr_info("%s : Registering jack driver\n", __func__);
	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}

	if (!pdata->get_adc_value || !pdata->zones ||
	    !pdata->set_micbias_state || pdata->num_zones > MAX_ZONE_LIMIT) {
		pr_err("%s : need to check pdata\n", __func__);
		return -ENODEV;
	}

	if (atomic_xchg(&instantiated, 1)) {
		pr_err("%s : already instantiated, can only have one\n",
			__func__);
		return -ENODEV;
	}

	sec_jack_key_map[0].gpio = pdata->send_end_gpio;

	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (hi == NULL) {
		pr_err("%s : Failed to allocate memory.\n", __func__);
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	hi->pdata = pdata;

	/* make the id of our gpio_event device the same as our platform device,
	 * which makes it the responsiblity of the board file to make sure
	 * it is unique relative to other gpio_event devices
	 */
	hi->dev_id = pdev->id;

	ret = gpio_request(pdata->det_gpio, "ear_jack_detect");
	if (ret) {
		pr_err("%s : gpio_request failed for %d\n",
		       __func__, pdata->det_gpio);
		goto err_gpio_request;
	}

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) {
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_dev_register_send_end;
	}
	wake_lock_init(&hi->det_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack_det");

	INIT_WORK(&hi->buttons_work, sec_jack_buttons_work);
	hi->queue = create_singlethread_workqueue("sec_jack_wq");
	if (hi->queue == NULL) {
		ret = -ENOMEM;
		pr_err("%s: Failed to create workqueue\n", __func__);
		goto err_create_wq_failed;
	}

	hi->det_irq = gpio_to_irq(pdata->det_gpio);

	jack_class = class_create(THIS_MODULE, "jack");
	if (IS_ERR(jack_class))
		pr_err("Failed to create class(sec_jack)\n");

	/* support PBA function test */
	jack_dev = device_create(jack_class, NULL, 0, hi, "jack_selector");
	if (IS_ERR(jack_dev))
		pr_err("Failed to create device(sec_jack)!= %ld\n",
			IS_ERR(jack_dev));

	if (device_create_file(jack_dev, &dev_attr_select_jack) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_select_jack.attr.name);

	set_bit(EV_KEY, hi->ids[0].evbit);
	hi->ids[0].flags = INPUT_DEVICE_ID_MATCH_EVBIT;
	hi->handler.filter = sec_jack_buttons_filter;
	hi->handler.connect = sec_jack_buttons_connect;
	hi->handler.disconnect = sec_jack_buttons_disconnect;
	hi->handler.name = "sec_jack_buttons";
	hi->handler.id_table = hi->ids;
	hi->handler.private = hi;

	ret = input_register_handler(&hi->handler);
	if (ret) {
		pr_err("%s : Failed to register_handler\n", __func__);
		goto err_register_input_handler;
	}
	ret = request_threaded_irq(hi->det_irq, NULL,
				   sec_jack_detect_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "sec_headset_detect", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);
		goto err_request_detect_irq;
	}

	/* to handle insert/removal when we're sleeping in a call */
	ret = enable_irq_wake(hi->det_irq);
	if (ret) {
		pr_err("%s : Failed to enable_irq_wake.\n", __func__);
		goto err_enable_irq_wake;
	}

	dev_set_drvdata(&pdev->dev, hi);

	return 0;

err_enable_irq_wake:
	free_irq(hi->det_irq, hi);
err_request_detect_irq:
	input_unregister_handler(&hi->handler);
err_register_input_handler:
	destroy_workqueue(hi->queue);
err_create_wq_failed:
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_sendend);
err_switch_dev_register_send_end:
	switch_dev_unregister(&switch_jack_detection);
err_switch_dev_register:
	gpio_free(pdata->det_gpio);
err_gpio_request:
	kfree(hi);
err_kzalloc:
	atomic_set(&instantiated, 0);

	return ret;
}

static int sec_jack_remove(struct platform_device *pdev)
{

	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);

	pr_info("%s :\n", __func__);
	disable_irq_wake(hi->det_irq);
	free_irq(hi->det_irq, hi);
	destroy_workqueue(hi->queue);
	if (hi->send_key_dev) {
		platform_device_unregister(hi->send_key_dev);
		hi->send_key_dev = NULL;
	}
	input_unregister_handler(&hi->handler);
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_sendend);
	switch_dev_unregister(&switch_jack_detection);
	gpio_free(hi->pdata->det_gpio);
	kfree(hi);
	atomic_set(&instantiated, 0);

	return 0;
}

static struct platform_driver sec_jack_driver = {
	.probe = sec_jack_probe,
	.remove = sec_jack_remove,
	.driver = {
			.name = "sec_jack",
			.owner = THIS_MODULE,
		   },
};
static int __init sec_jack_init(void)
{
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	platform_driver_unregister(&sec_jack_driver);
}

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("ms17.kim@samsung.com");
MODULE_DESCRIPTION("Samsung Electronics Corp Ear-Jack detection driver");
MODULE_LICENSE("GPL");
#else
/*
 *	JACK device detection driver.
 *
 *	Copyright (C) 2009 Samsung Electronics, Inc.
 *
 *	Authors:
 *		Uk Kim <w0806.kim@samsung.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <mach/gpio-aries.h>
//#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/sec_jack.h>

//#define CONFIG_DEBUG_SEC_JACK
#define SUBJECT "JACK_DRIVER"

// reverse ear jack detection value - noik.heo (20100204)
#define CONFIG_EARJACK_GPIO_LOW_ACTIVE_ENABLE
#define CONFIG_SEC_HEADSETHOOK_KEY

#ifdef CONFIG_DEBUG_SEC_JACK
#define SEC_JACKDEV_DBG(format,...)\
	printk ("["SUBJECT" (%s,%d)] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#else
#define SEC_JACKDEV_DBG(format,...)
#endif

#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
#define KEYCODE_HEADSETHOOK 248
#else
#define KEYCODE_SENDEND 248
#endif

#define DETECTION_CHECK_COUNT	2
#define	DETECTION_CHECK_TIME	get_jiffies_64() + (HZ/10)// 1000ms / 10 = 100ms
#define	SEND_END_ENABLE_TIME	get_jiffies_64() + (HZ*1)// 1000ms * 1 = 1sec

#define SEND_END_CHECK_COUNT	3
#define SEND_END_CHECK_TIME     get_jiffies_64() + (HZ * 2) //2000ms 

#define WAKELOCK_DET_TIMEOUT	HZ * 5 //5 sec

static struct platform_driver sec_jack_driver;

struct class *jack_class;
EXPORT_SYMBOL(jack_class);
static struct device *jack_selector_fs;				// Sysfs device, this is used for communication with Cal App.
EXPORT_SYMBOL(jack_selector_fs);
extern int s3c_adc_get_adc_data(int channel);
extern unsigned int HWREV;
//extern int isVoiceCall;
//extern int proximity_onoff(u8 onoff);
//extern bool prox_factorymode;

struct sec_jack_info {
	struct sec_jack_port port;
	struct input_dev *input;
};

static struct sec_jack_info *hi;

struct switch_dev switch_jack_detection = {
		.name = "h2w",
};

struct switch_dev switch_dock_detection = {
		.name = "dock",
};

/* To support AT+FCESTEST=1 */
struct switch_dev switch_sendend = {
		.name = "send_end",
};
static struct timer_list send_end_key_event_timer;

static unsigned int current_jack_type_status;
static unsigned int send_end_enable = 0;
static unsigned int send_end_pressed = 0;
static struct wake_lock jack_sendend_wake_lock;
static int recording_status=0;
static int send_end_irq_token=0;

unsigned int get_headset_status(void)
{
	SEC_JACKDEV_DBG(" headset_status %d", current_jack_type_status);
	return current_jack_type_status;
}
EXPORT_SYMBOL(get_headset_status);


void set_recording_status(int value)
{
	recording_status = value;
}


static int get_recording_status(void)
{
	return recording_status;
}

void set_dock_state(int value)
{
	printk(KERN_INFO "set_dock_state : 0X%x\n", value);
	switch_set_state(&switch_dock_detection, value);
}

static void jack_input_selector(int jack_type_status)
{
	SEC_JACKDEV_DBG("jack_type_status = 0X%x", jack_type_status);

	// [DH18-2315] // test // commented out //
	switch(jack_type_status)
	{
		case SEC_HEADSET_3_POLE_DEVICE:
		case SEC_HEADSET_4_POLE_DEVICE:	
		{
			break;
		}
		case SEC_TVOUT_DEVICE:
		{
			break;
		}
		case SEC_UNKNOWN_DEVICE:
		{
			printk("unknown jack device attached. User must select jack device type\n");
			break;
		}
		default:
		{
			printk(KERN_ERR "wrong selector value\n");
			break;			
		}
	}
}

static int jack_type_detect_change(struct work_struct *ignored)
{
	int adc = 0;
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	int state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int count_abnormal=0;
	int count_pole=0;
	bool bQuit = false;

	while(!bQuit)
	{
		state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

		if(state)
		{
			adc = s3c_adc_get_adc_data(SEC_HEADSET_ADC_CHANNEL);

			/* 4 pole - jig zone */
			if( (630 <= adc) && (adc < 3700) )
			{
				current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
				printk(KERN_INFO "[JACK_DRIVER (%s,%d)] 4 pole attached : adc = %d\n", __func__, __LINE__, adc);
				bQuit = true;
				if(send_end_irq_token==0)
				{
					enable_irq(send_end->eint);
					send_end_irq_token=1;
				}
			}
			/* 3 pole zone or unstable zone */
			else
			{	
				if( !adc || (count_pole == 10) )
				{
					/* detect 3pole or tv-out cable */
					printk(KERN_INFO "[JACK_DRIVER (%s,%d)] 3 pole attatched : adc = %d\n", __func__,__LINE__,adc);
					count_pole = 0;
					if(send_end_irq_token==1)
					{
						disable_irq(send_end->eint);
						send_end_irq_token=0;
					}
					current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
#if 1
					if (HWREV >= 7) {
						gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
					} else {
						if(!get_recording_status())
						{
							gpio_set_value(GPIO_MICBIAS_EN, 0);
						}
					}
#elif defined CONFIG_M115S
                    gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
#endif
					bQuit=true;
				}
				/* If 4 pole is inserted slowly, ADC value should be lower than 250.
			 	* So, check again.
				 */
				else
				{
					++count_pole;
					/* Todo : to prevent unexpected reset bug.
					 * 		  is it msleep bug? need wakelock.
					 */
					wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
					msleep(20);
				}
			}
		} /* if(state) */
		else
		{
			bQuit = true;

			current_jack_type_status = SEC_JACK_NO_DEVICE;
#if 1
			if (HWREV >= 7) {
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			} else {
				gpio_set_value(GPIO_MICBIAS_EN, 0);
			}
#elif defined CONFIG_M115S
            gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
#endif
			SEC_JACKDEV_DBG("JACK dev detached\n");
		}
		switch_set_state(&switch_jack_detection, current_jack_type_status);
		jack_input_selector(current_jack_type_status);
	}

	return 0;

}

static DECLARE_DELAYED_WORK(detect_jack_type_work, jack_type_detect_change);

static int jack_detect_change(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state;
	int count=20;

	SEC_JACKDEV_DBG("");
	//del_timer(&jack_detect_timer);
	cancel_delayed_work_sync(&detect_jack_type_work);
	state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
	
	/* block send/end key event */
	mod_timer(&send_end_key_event_timer, SEND_END_CHECK_TIME);
#if 0
	if(!prox_factorymode){
		if(state)
		{
			proximity_onoff(0);
		}
		else
		{
			// [DE14-1705] hi99.an // for sleep current when ear-jack out
			if (isVoiceCall)
			{
				proximity_onoff(1);
				gpio_direction_output(GPIO_PS_ON, 1);
				//printk("proximity on while voice call\n");
			}
		}
	}/*	if(prox_factorymode) */
#endif
	if(state)
	{		
		/* check pin state repeatedly */
		while(count--)
		{
			if(state != (gpio_get_value(det_jack->gpio) ^ det_jack->low_active))
			{
				return -1;
			}
			msleep(10); // [DH18-2015] // 3pole/4pole timing in VoIP (mdelay->msleep)
			//mdelay(10);
		}

#if 1
		if (HWREV >= 7) {
			gpio_set_value(GPIO_SUB_MICBIAS_EN, 1);
		} else {
			gpio_set_value(GPIO_MICBIAS_EN, 1);
		}

        if (HWREV<=3)
            gpio_set_value(GPIO_EARPATH_SEL_R03, 1);	//1:headset, 0: TV_OUT
        else
            gpio_set_value(GPIO_EARPATH_SEL_R04, 1);	//1:headset, 0: TV_OUT
#elif defined CONFIG_M115S
        gpio_set_value(GPIO_SUB_MICBIAS_EN, 1);
        gpio_set_value(GPIO_EARPATH_SEL, 1);            //1:headset, 0: TV_OUT
#endif
		schedule_delayed_work(&detect_jack_type_work,50);
	}
	else if(!state && current_jack_type_status != SEC_JACK_NO_DEVICE)
	{
		if(send_end_irq_token==1)
		{
			disable_irq(send_end->eint);
			send_end_irq_token=0;
		}
		current_jack_type_status = SEC_JACK_NO_DEVICE;
#if 1
		if (HWREV >= 7) {
			gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			if (HWREV<=3)
				gpio_set_value(GPIO_EARPATH_SEL_R03, 0);	//1:headset, 0: TV_OUT
			else
				gpio_set_value(GPIO_EARPATH_SEL_R04, 0);	//1:headset, 0: TV_OUT
			//printk("jack detach bias off\n");
		} else {
	        if(!get_recording_status())
	        {
				gpio_set_value(GPIO_MICBIAS_EN, 0);
			}
		}
#elif defined CONFIG_M115S
        gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
		gpio_set_value(GPIO_EARPATH_SEL, 0);	            //1:headset, 0: TV_OUT
#endif

		switch_set_state(&switch_jack_detection, current_jack_type_status);
		SEC_JACKDEV_DBG("JACK dev detached  \n");			

	}

	return 0;
}

static int sendend_switch_change(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state, headset_state, check_state;
	SEC_JACKDEV_DBG("");
	int count=6;
	int adc=0;
	
	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	state = gpio_get_value(send_end->gpio) ^ send_end->low_active;

	wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);

	adc = s3c_adc_get_adc_data(SEC_HEADSET_ADC_CHANNEL);
	printk(KERN_INFO "[JACK_DRIVER (%s,%d)] jack adc in send end int = %d\n", __func__, __LINE__, adc);

	/* check pin state repeatedly */
#if 1
	printk(KERN_DEBUG "[JACK] state (%d %d %d)\n", send_end_pressed, headset_state, state);
	if (!send_end_pressed)
	{
		if (state == 0)
		{
			printk(KERN_INFO "[JACK] Button release is ignored. former input was invalid.\n");
			return -1;
		}

		//while(count-- && !send_end_pressed)
		while(count--)
		{
			mdelay(10);

			check_state = gpio_get_value(send_end->gpio);
			printk(KERN_DEBUG "%d\n", check_state);

			if ((state != (check_state ^ send_end->low_active)) || (!headset_state) || (current_jack_type_status != SEC_HEADSET_4_POLE_DEVICE))
			//if(state != (gpio_get_value(send_end->gpio) ^ send_end->low_active) || !headset_state || (current_jack_type_status != SEC_HEADSET_4_POLE_DEVICE) || (adc < 76))
			{
				printk(KERN_INFO "[JACK] Button press is ignored. State is unstable.\n");
				return -1;
			}
		}
	}
#else
	while(count-- && !send_end_pressed)
	{
		if(state != (gpio_get_value(send_end->gpio) ^ send_end->low_active) || !headset_state || (current_jack_type_status != SEC_HEADSET_4_POLE_DEVICE) || (adc < 76))
		{
			printk(KERN_INFO "[JACK_DRIVER (%s,%d)] SEND_END : IGNORED : Unstable state\n", __func__, __LINE__);
			return -1;
		}
		mdelay(10);
	}
#endif

#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
	input_report_key(hi->input, KEYCODE_HEADSETHOOK, state);
#else
	input_report_key(hi->input, KEYCODE_SENDEND, state);
#endif
	input_sync(hi->input);
	switch_set_state(&switch_sendend,state);
	send_end_pressed = state;
	printk(KERN_INFO "[JACK_DRIVER (%s,%d)] Button is %s.\n", __func__, __LINE__, state? "pressed" : "released");

	return 0;
}

static int sendend_timer_work_func(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
 	int headset_state;

	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	send_end_enable = 1;
	
	if(send_end_pressed && !headset_state)
	{
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
		input_report_key(hi->input, KEYCODE_HEADSETHOOK, 0);
#else
		input_report_key(hi->input, KEYCODE_SENDEND, 0);
#endif
		input_sync(hi->input);
		switch_set_state(&switch_sendend,0);
		send_end_pressed = 0;
		SEC_JACKDEV_DBG("Button is %s forcely.\n", "released");
	}
}

static DECLARE_WORK(jack_detect_work, jack_detect_change);
static DECLARE_WORK(sendend_switch_work, sendend_switch_change);
static DECLARE_WORK(sendend_timer_work, sendend_timer_work_func);

//IRQ Handler
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	SEC_JACKDEV_DBG("jack isr");
	send_end_enable = 0;
	schedule_work(&jack_detect_work);
	return IRQ_HANDLED;
}

static void send_end_key_event_timer_handler(unsigned long arg)
{
#if 1
	schedule_work(&sendend_timer_work);
#else
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
 	int headset_state;

	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	send_end_enable = 1;

	if(send_end_pressed && !headset_state)
	{
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
		input_report_key(hi->input, KEYCODE_HEADSETHOOK, 0);
#else
		input_report_key(hi->input, KEYCODE_SENDEND, 0);
#endif
		input_sync(hi->input);
		send_end_pressed = 0;
		SEC_JACKDEV_DBG("Button is %s forcely.\n", "released");
	}
#endif
}

static irqreturn_t send_end_irq_handler(int irq, void *dev_id)
{
   struct sec_gpio_info   *det_jack = &hi->port.det_jack;
   int headset_state;

	SEC_JACKDEV_DBG("sendend isr");
	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	SEC_JACKDEV_DBG("SEND/END isr, enable = %d , headset_state = %d",send_end_enable,headset_state);

	if (send_end_enable && headset_state && (current_jack_type_status == SEC_HEADSET_4_POLE_DEVICE))
	{
		schedule_work(&sendend_switch_work);		
	}
		  
	return IRQ_HANDLED;
}

static ssize_t select_jack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[JACK] %s : operate nothing\n", __FUNCTION__);

	return 0;
}

static ssize_t select_jack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	sscanf(buf, "%d", &value);
	printk(KERN_INFO "[JACK_DRIVER] User  selection : 0X%x", value);
		
	switch(value)
	{
		case SEC_HEADSET_3_POLE_DEVICE:
		{
#if 1
			if (HWREV >= 7) 
#endif
            {
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			}

			current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
			printk(KERN_INFO "[JACK_DRIVER (%s,%d)] 3 pole attached\n", __func__, __LINE__);

			if(send_end_irq_token==1)
			{
				disable_irq(send_end->eint);
				send_end_irq_token=0;
			}

			break;
		}
		case SEC_HEADSET_4_POLE_DEVICE:
		{
#if 1
			if (HWREV >= 7)
#endif
            {
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 1);
			}

			current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
			printk(KERN_INFO "[JACK_DRIVER (%s,%d)] 4 pole attached\n", __func__, __LINE__);

			if(send_end_irq_token==0)
			{
				enable_irq(send_end->eint);
				send_end_irq_token=1;
			}

			break;
		}
		case SEC_JACK_NO_DEVICE:
		{
#if 1
			if (HWREV >= 7) {
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			} else {
				gpio_set_value(GPIO_MICBIAS_EN, 0);
			}
#elif defined CONFIG_M115S
            gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
#endif

			current_jack_type_status = SEC_JACK_NO_DEVICE;
			printk(KERN_INFO "[JACK_DRIVER (%s,%d)] JACK dev detached\n", __func__, __LINE__);

			if(send_end_irq_token==1)
			{
				disable_irq(send_end->eint);
				send_end_irq_token=0;
			}

			break;
		}
		default:
		{
			//current_jack_type_status = SEC_JACK_NO_DEVICE;
#if 1
			if (HWREV >= 7) {
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			} else {
				gpio_set_value(GPIO_MICBIAS_EN, 0);
			}
#elif defined CONFIG_M115S
            gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
#endif
			break;
		}
	}

	switch_set_state(&switch_jack_detection, current_jack_type_status);
	jack_input_selector(current_jack_type_status);

	return size;
}


static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, select_jack_show, select_jack_store);

static int sec_jack_probe(struct platform_device *pdev)
{
	int ret;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	struct sec_gpio_info   *det_jack;
	struct sec_gpio_info   *send_end;
	struct input_dev	   *input;
	current_jack_type_status = SEC_JACK_NO_DEVICE;
	
	printk(KERN_INFO "SEC JACK: Registering jack driver\n");
	
	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	memcpy (&hi->port, pdata->port, sizeof(struct sec_jack_port));

	input = hi->input = input_allocate_device();
	if (!input)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "SEC JACK: Failed to allocate input device.\n");
		goto err_request_input_dev;
	}

	input->name = "sec_jack";
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
	set_bit(KEYCODE_HEADSETHOOK, input->keybit);
#else
	set_bit(KEYCODE_SENDEND, input->keybit);
#endif

	ret = input_register_device(input);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register driver\n");
		goto err_register_input_dev;
	}

	//init_timer(&jack_detect_timer);
	//jack_detect_timer.function = jack_detect_timer_handler;

	init_timer(&send_end_key_event_timer);
	send_end_key_event_timer.function = send_end_key_event_timer_handler;

	SEC_JACKDEV_DBG("registering switch_sendend switch_dev sysfs sec_jack");
	
	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}
/*
	ret = switch_dev_register(&switch_dock_detection);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC DOCK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}
*/
	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}

	//Create JACK Device file in Sysfs
	jack_class = class_create(THIS_MODULE, "jack");
	if(IS_ERR(jack_class))
	{
		printk(KERN_ERR "Failed to create class(sec_jack)\n");
	}

	jack_selector_fs = device_create(jack_class, NULL, 0, NULL, "jack_selector");
	if (IS_ERR(jack_selector_fs))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n", IS_ERR(jack_selector_fs));

	if (device_create_file(jack_selector_fs, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_select_jack.attr.name);	

	//GPIO configuration
	send_end = &hi->port.send_end;
	s3c_gpio_cfgpin(send_end->gpio, S3C_GPIO_SFN(send_end->gpio_af));
	s3c_gpio_setpull(send_end->gpio, S3C_GPIO_PULL_NONE);
	set_irq_type(send_end->eint, IRQ_TYPE_EDGE_BOTH);

	ret = request_irq(send_end->eint, send_end_irq_handler, IRQF_DISABLED, "sec_headset_send_end", NULL);

	SEC_JACKDEV_DBG("sended isr send=0X%x, ret =%d", send_end->eint, ret);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC HEADSET: Failed to register send/end interrupt.\n");
		goto err_request_send_end_irq;
	}

	disable_irq(send_end->eint);
	send_end_irq_token=0;

	det_jack = &hi->port.det_jack;
	s3c_gpio_cfgpin(det_jack->gpio, S3C_GPIO_SFN(det_jack->gpio_af));
	s3c_gpio_setpull(det_jack->gpio, S3C_GPIO_PULL_NONE);
	set_irq_type(det_jack->eint, IRQ_TYPE_EDGE_BOTH);
	
#ifdef CONFIG_EARJACK_GPIO_LOW_ACTIVE_ENABLE		// noik.heo (20100217)
	det_jack->low_active = 1; 
#else
	if(HWREV >= 0xa)
	{
		det_jack->low_active = 1; 
	}
#endif

	ret = request_irq(det_jack->eint, detect_irq_handler, IRQF_DISABLED, "sec_headset_detect", NULL);

	SEC_JACKDEV_DBG("det isr det=0X%x, ret =%d", det_jack->eint, ret);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC HEADSET: Failed to register detect interrupt.\n");
		goto err_request_detect_irq;
	}

#if 1
	// EAR_SEL
    if (HWREV<=3)
    {
    	if(gpio_is_valid(GPIO_EARPATH_SEL_R03))
    	{
    		if(gpio_request(GPIO_EARPATH_SEL_R03, "GPJ2"))
    			printk(KERN_ERR "Failed to request GPIO_EAR_SEL!\n");
    		gpio_direction_output(GPIO_EARPATH_SEL_R03,1);
    	}
    	s3c_gpio_slp_cfgpin(GPIO_EARPATH_SEL_R03, S3C_GPIO_SLP_PREV);
    	if(gpio_is_valid(GPIO_MICBIAS_EN))
    	{
    		if(gpio_request(GPIO_MICBIAS_EN, "GPJ4"))
    			printk(KERN_ERR "Failed to request GPIO_MICBIAS_EN!\n");
    		gpio_direction_output(GPIO_MICBIAS_EN, 0);
    	}
    	s3c_gpio_slp_cfgpin(GPIO_MICBIAS_EN, S3C_GPIO_SLP_PREV);
    }
    else
    {
    	if(gpio_is_valid(GPIO_EARPATH_SEL_R04))
    	{
    		if(gpio_request(GPIO_EARPATH_SEL_R04, "GPJ2"))
    			printk(KERN_ERR "Failed to request GPIO_EAR_SEL!\n");
    		gpio_direction_output(GPIO_EARPATH_SEL_R04,1);
    	}
    	s3c_gpio_slp_cfgpin(GPIO_EARPATH_SEL_R04, S3C_GPIO_SLP_PREV);
		if (HWREV >= 7)
		{
	    	if(gpio_is_valid(GPIO_SUB_MICBIAS_EN))
	    	{
	    		if(gpio_request(GPIO_SUB_MICBIAS_EN, "GPJ2"))
	    			printk(KERN_ERR "Failed to request GPIO_SUB_MICBIAS_EN!\n");
				gpio_direction_output(GPIO_SUB_MICBIAS_EN, 0);
	    	}
			s3c_gpio_slp_cfgpin(GPIO_SUB_MICBIAS_EN, S3C_GPIO_SLP_PREV);
		}
		else // HWREV < 7
		{
	    	if(gpio_is_valid(GPIO_MICBIAS_EN))
	    	{
	    		if(gpio_request(GPIO_MICBIAS_EN, "GPJ4"))
	    			printk(KERN_ERR "Failed to request GPIO_MICBIAS_EN!\n");
				gpio_direction_output(GPIO_MICBIAS_EN, 0);
	    	}
	    	s3c_gpio_slp_cfgpin(GPIO_MICBIAS_EN, S3C_GPIO_SLP_PREV);
		}
    }
#elif defined CONFIG_M115S
    if(gpio_is_valid(GPIO_EARPATH_SEL))
    {
    	if(gpio_request(GPIO_EARPATH_SEL, "GPJ26"))
    		printk(KERN_ERR "Failed to request GPIO_EAR_SEL!\n");
    	gpio_direction_output(GPIO_EARPATH_SEL,1);
    }
    s3c_gpio_slp_cfgpin(GPIO_EARPATH_SEL, S3C_GPIO_SLP_PREV);

	if(gpio_is_valid(GPIO_SUB_MICBIAS_EN))
	{
		if(gpio_request(GPIO_SUB_MICBIAS_EN, "GPJ25"))
			printk(KERN_ERR "Failed to request GPIO_SUB_MICBIAS_EN!\n");
		gpio_direction_output(GPIO_SUB_MICBIAS_EN, 0);
	}
	s3c_gpio_slp_cfgpin(GPIO_SUB_MICBIAS_EN, S3C_GPIO_SLP_PREV);
#endif

	wake_lock_init(&jack_sendend_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack");

	schedule_work(&jack_detect_work);
	
	return 0;

err_request_send_end_irq:
	free_irq(det_jack->eint, 0);
err_request_detect_irq:
	switch_dev_unregister(&switch_jack_detection);
	//switch_dev_unregister(&switch_dock_detection);
	switch_dev_unregister(&switch_sendend);
err_switch_dev_register:
	input_unregister_device(input);
err_register_input_dev:
	input_free_device(input);
err_request_input_dev:
	kfree (hi);

	return ret;	
}

static int sec_jack_remove(struct platform_device *pdev)
{
	SEC_JACKDEV_DBG("");
	input_unregister_device(hi->input);
	free_irq(hi->port.det_jack.eint, 0);
	free_irq(hi->port.send_end.eint, 0);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
	return 0;
}

#ifdef CONFIG_PM
static int sec_jack_suspend(struct platform_device *pdev, pm_message_t state)
{
	//SEC_JACKDEV_DBG("");
	//flush_scheduled_work();

	// [DE11-2132] hi99.an // sleep-current 100uA down
	if (current_jack_type_status != SEC_HEADSET_4_POLE_DEVICE)
	{
#if 1
		if (HWREV >= 7)
		{
			gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
		}
		else
		{
	        if(!get_recording_status())
				gpio_set_value(GPIO_MICBIAS_EN, 0);
		}
#elif defined CONFIG_M115S
        gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
#endif
        printk(KERN_DEBUG "suspend off sub bias\n");
	}

	return 0;
}
static int sec_jack_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define s3c_headset_resume	NULL
#define s3c_headset_suspend	NULL
#endif

static int __init sec_jack_init(void)
{
	SEC_JACKDEV_DBG("");
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	platform_driver_unregister(&sec_jack_driver);
}

static struct platform_driver sec_jack_driver = {
	.probe		= sec_jack_probe,
	.remove		= sec_jack_remove,
	.suspend	= sec_jack_suspend,
	.resume		= sec_jack_resume,
	.driver		= {
		.name		= "sec_jack",
		.owner		= THIS_MODULE,
	},
};

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("Uk Kim <w0806.kim@samsung.com>");
MODULE_DESCRIPTION("SEC JACK detection driver");
MODULE_LICENSE("GPL");

#endif
