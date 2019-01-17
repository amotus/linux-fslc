/*
 * Power Button management 
 * Copyright 2013 Seco S.r.l.
 *
 * Author: Matrco Sandrelli <marco.sandrelli@seco.com>
 * Maintainers: <marco.sandrelli@seco.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Add in board platform definition the following function to create exported device
 * 
 */

#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/module.h>

#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>


#define INPUT_DEV_NAME		  "Embedded Controller"
#define PWR_BUTTON_NAME		  "power_button"
#define INPUT_POWER_NAME	  INPUT_DEV_NAME " - " PWR_BUTTON_NAME
#define MAX_LEN_NAME		  32
#define DELAY_TIME            500
#define HALT_TIME		      4000


struct event_dev {
	unsigned short int        id;
	struct input_dev          *input;
	char                      name[MAX_LEN_NAME];
};


struct pwrb_control {
	struct event_dev          *events;
	int                       irq;
	unsigned long             irq_flags;
	unsigned long             pwrb_gpio;
	unsigned long             halt_gpio;
	char                      *phys;
	struct delayed_work       pwbd_event_work;
	struct workqueue_struct   *pwbd_wq;
	long                      msecs;
};




/*
 *	Manage Power Button event
 *
 */
static void pwbd_pressure_event_worker (struct work_struct *work) {
	/* Report power button event */
	int state = 0;
	int error = 0;
	static int off_event = 0;
	struct pwrb_control *wm = container_of(work, struct pwrb_control, pwbd_event_work.work);

	if ( !wm ) {
		enable_irq(wm->irq);
		return;
	}

	if ( (jiffies_to_msecs(jiffies) - wm->msecs) < HALT_TIME ) {

		error = gpio_request (wm->pwrb_gpio, "PWR_BTN");
		if ( error < 0 ) {
			enable_irq(wm->irq);
			return;
		}
		gpio_direction_input (wm->pwrb_gpio);
		state = gpio_get_value_cansleep(wm->pwrb_gpio) ? 1 : 0;
		gpio_free (wm->pwrb_gpio);

		if ( state == 1 ) {
			/* End of pressure of the button under the max time (HALT_TIME).
			   Shutdown the system calling dpwrevn.
			*/
			input_event (wm->events->input, EV_PWR, BTN_ECTRL_PWR, 1);
			input_sync (wm->events->input);
			wm->msecs = 0;
			enable_irq(wm->irq);
		} else {
			/*  the button stills to be pressed.
			    Schedule again the task
			*/
			queue_delayed_work (wm->pwbd_wq, &wm->pwbd_event_work,msecs_to_jiffies(DELAY_TIME));
		}

	} else {

			if ( !off_event ) {
				printk (KERN_INFO "%s: halt the system!", __func__);
			}
			off_event = 1;
			gpio_request (wm->halt_gpio, "HALT");
			gpio_direction_output (wm->halt_gpio, 1);
			mdelay (1);
			gpio_set_value (wm->halt_gpio, 0);
			queue_delayed_work (wm->pwbd_wq, &wm->pwbd_event_work,msecs_to_jiffies(DELAY_TIME));

	}

}



/*
 *      Catch Power Button Event
 *
 */
static irqreturn_t pwrb_irq_handler (int irq, void *dev_id)  {

	struct pwrb_control *pwbd =  ((struct pwrb_control *) dev_id);
	printk (KERN_INFO "event name: %s\n", pwbd->events->input->name);
	disable_irq_nosync (pwbd->irq);

	if ( !delayed_work_pending (&pwbd->pwbd_event_work) ){
		pwbd->msecs = jiffies_to_msecs (jiffies);
		queue_delayed_work (pwbd->pwbd_wq, &pwbd->pwbd_event_work, msecs_to_jiffies(DELAY_TIME));
	}

	return IRQ_HANDLED;
}



/*
 *      Probing the driver
 *
 */
static int pwrb_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	struct pwrb_control *pwrb_data;
	int err, ret;

	printk("pwrb probing...");

	pwrb_data = kzalloc (sizeof (struct pwrb_control), GFP_KERNEL);
	if ( !pwrb_data ) {
		err = -ENOMEM;
		goto err_pwrb_data;
	}
	platform_set_drvdata (pdev, pwrb_data);

	pwrb_data->events = kzalloc (sizeof (struct event_dev), GFP_KERNEL);
	if ( !pwrb_data->events ) {
		err = -ENOMEM;
		goto err_event_data;
	}

 	pwrb_data->events->input = input_allocate_device ();
	if ( !pwrb_data->events->input ) {
		err = -ENOMEM;
		goto err_input_data;
	}

	/*  set input interface  */
	pwrb_data->phys = kzalloc (sizeof (char) * MAX_LEN_NAME, GFP_KERNEL);
    snprintf (pwrb_data->phys, sizeof(char) * MAX_LEN_NAME,
                                "%s/input%d", dev_name(&pdev->dev), 0);

	pwrb_data->events->input->name       = INPUT_POWER_NAME;
	pwrb_data->events->input->phys       = pwrb_data->phys;
	pwrb_data->events->input->dev.parent = &pdev->dev;
	pwrb_data->events->input->id.bustype = BUS_VIRTUAL;

	input_set_capability (pwrb_data->events->input, EV_PWR, BTN_ECTRL_PWR);

	err = input_register_device (pwrb_data->events->input);
	if ( err ) {
		printk (KERN_ERR "pwrb: No input assigned");
		err = -EINVAL;
		goto err_input_reg;
	}

	pwrb_data->irq = irq_of_parse_and_map (dp, 0);
	pwrb_data->irq_flags = IRQ_TYPE_LEVEL_LOW;
	pwrb_data->pwrb_gpio = of_get_named_gpio(dp, "pwr-gpio", 0);
	pwrb_data->halt_gpio = of_get_named_gpio (dp, "halt-gpio", 0);

	ret = request_irq (pwrb_data->irq, pwrb_irq_handler,
			pwrb_data->irq_flags, "pwrb_irq", pwrb_data);
	if ( ret ) {
		printk ("pwrb error %d: IRQ not acquired", ret);
		err = -EIO;
		goto err_irq_req;
	}

	pwrb_data->pwbd_wq = create_singlethread_workqueue ("pwbd_work");
	if( !pwrb_data->pwbd_wq ){
		printk ("pwrb error %d: Work not allocated", ret);
		err = -EINVAL;
		goto err_free_irq;
	}
	INIT_DELAYED_WORK(&pwrb_data->pwbd_event_work, pwbd_pressure_event_worker);
	pwrb_data->msecs = 0;
	printk ("done\n");

	return 0;
err_free_irq:
	free_irq (pwrb_data->irq, pwrb_data);
err_irq_req:
	input_unregister_device (pwrb_data->events->input);
err_input_reg:
	input_free_device (pwrb_data->events->input);
err_input_data:
	kfree (pwrb_data->events);
err_event_data:
	kfree (pwrb_data);
err_pwrb_data:

	printk ("Errors pwrb (%d)\n", err);
	return err;
}


static int pwrb_remove (struct platform_device *pdev) {

	struct pwrb_control *pwrb_data = platform_get_drvdata(pdev);

	free_irq (pwrb_data->irq, pwrb_data);
	input_unregister_device (pwrb_data->events->input);
	kfree (pwrb_data->events);
	kfree (pwrb_data);
	return 0;
}


static const struct of_device_id seco_pwr_btn_match[] = {
	{ .compatible = "seco,power_button" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_pwr_btn_match);


static struct platform_driver pwrb_driver = {
        .driver = {
		.name           = "imx_seco_pwrb",
		.owner          = THIS_MODULE,
		.of_match_table = seco_pwr_btn_match,
	},
	.probe  = pwrb_probe,
	.remove = pwrb_remove,
};

static int __init pwrb_init (void) {
        return platform_driver_register (&pwrb_driver);
}

subsys_initcall (pwrb_init);


static void __exit pwrb_exit (void) {
        platform_driver_unregister (&pwrb_driver);

}

module_exit (pwrb_exit);



MODULE_AUTHOR("MS DC SECO, Inc.");
MODULE_DESCRIPTION("Power Button Management Driver");
MODULE_LICENSE("GPL");
