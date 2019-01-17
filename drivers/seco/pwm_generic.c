#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_generic.h>
#include <linux/slab.h>
#include <linux/workqueue.h>


#define PWM_NAME_ID   "pwm_generic"

#define PWMG_INFO(fmt, arg...) printk(KERN_INFO "PWM driver: " fmt "\n" , ## arg)
#define PWMG_ERR(fmt, arg...)  dev_err(&pdev->dev, "%s: " fmt "\n" , __func__ , ## arg)
#define PWMG_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

/* One Shot Buzzer Work */
#define PWMG_WORK_POLLING_JIFFIES	                   msecs_to_jiffies(1)            // 1 milliseconds
#define DIV_TIME					   1

struct pwmg_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	int			id;
	unsigned int		period;
	unsigned int		period_min;
	unsigned int		period_max;
	unsigned int		lth_duty;
	unsigned long		duty;
	unsigned long		max_duty;
	int			enable;
	struct mutex		ops_lock;
	unsigned long		oneshot;
	struct delayed_work	pwmg_oneshot_work;
	struct workqueue_struct *pwmg_workq;
};


static int pwmg_update_status (struct  pwmg_data *pd) {

	int duty = pd->duty;
	int max = pd->max_duty;
	
	if (duty == 0 || pd->enable == 0) {
		pwm_config (pd->pwm, 0, pd->period);
		pwm_disable (pd->pwm);
	} else if (pd->enable == 1) {
		duty = pd->lth_duty +
			(duty * (pd->period - pd->lth_duty) / max);
		pwm_config (pd->pwm, duty, pd->period);
		pwm_enable (pd->pwm);
	}
	return 0;
}

/*
 * pwmg_oneshot_work_handler controls wm->oneshot value and it dectivates 
 * the buzzer when jiffies time becomes less than a given value written through 
 * sysfs in the parameter oneshot.
*/

static void pwmg_oneshot_work_handler (struct work_struct *work) {

	struct pwmg_data *wm = container_of(work, struct pwmg_data, pwmg_oneshot_work.work);
	if(wm->oneshot >= 1) {
		wm->oneshot = wm->oneshot - 1;
		queue_delayed_work(wm->pwmg_workq, &wm->pwmg_oneshot_work, PWMG_WORK_POLLING_JIFFIES);
	}	
	else {
		wm->enable = 0;
		pwmg_update_status(wm);
	}
	return;	
}

static ssize_t pwmg_show_enable (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;
	return sprintf(buf, "%d\n", pd->enable);
}


static ssize_t pwmg_store_enable (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long int  enable;
	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	rc = kstrtoul (buf, 0, &enable);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	
	if (enable == 1 || enable == 0) {
		pd->enable = enable;
		pwmg_update_status (pd);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	return sprintf(buf, "%lu\n", pd->duty);
}


static ssize_t pwmg_store_duty (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long duty;
	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	rc = kstrtoul (buf, 0, &duty);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	if (duty > pd->max_duty)
			rc = -EINVAL;
	else {
		pd->duty = duty;
		pwmg_update_status (pd);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_max_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	return sprintf(buf, "%lu\n", pd->max_duty);
}


static ssize_t pwmg_show_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period);
}


static ssize_t pwmg_store_period (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	
	int rc;
	unsigned long period;
	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	rc = kstrtoul (buf, 0, &period);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	if ((period > pd->period_max) || (period < pd->period_min))
		rc = -EINVAL;
	else {
		pd->lth_duty /= pd->period;
		pd->period = period;
		pd->lth_duty = pd->lth_duty * pd->period;
		pwmg_update_status (pd);
		rc = count;
	}

	mutex_unlock (&pd->ops_lock);

	return rc;	
}


static ssize_t pwmg_show_max_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;
	return sprintf(buf, "%d\n", pd->period_max);
}


static ssize_t pwmg_show_min_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period_min);
}

static ssize_t pwmg_show_oneshot_time (struct device *dev,
                struct device_attribute *attr, char *buf) {

	struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

        if (!pd) return 0;
        return sprintf(buf, "%d\n", jiffies_to_msecs(pd->oneshot) / DIV_TIME);
}

static ssize_t pwmg_store_oneshot_time (struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
        unsigned long int  oneshot;
        struct pwmg_data *pd = (struct pwmg_data *)dev_get_drvdata(dev);

        if (!pd) return 0;

        rc = kstrtoul (buf, 0, &oneshot);
        if (rc)
                return rc;

        rc = -ENXIO;

        mutex_lock (&pd->ops_lock);

        if (oneshot >= 0 ) {
                pd->oneshot =  msecs_to_jiffies(oneshot*DIV_TIME);
		pd->enable = 1;
                pwmg_update_status (pd);
                rc = count;
		/* launch work */
                if (pd->pwmg_workq)
                        queue_delayed_work(pd->pwmg_workq, &pd->pwmg_oneshot_work, PWMG_WORK_POLLING_JIFFIES);
        }
	
        mutex_unlock (&pd->ops_lock);

        return rc;

}

static DEVICE_ATTR (enable, 0644, pwmg_show_enable, pwmg_store_enable);
static DEVICE_ATTR (duty, 0644, pwmg_show_duty, pwmg_store_duty);
static DEVICE_ATTR (max_duty, 0444, pwmg_show_max_duty, NULL);
static DEVICE_ATTR (period_ns, 0644, pwmg_show_period, pwmg_store_period);
static DEVICE_ATTR (period_ns_max, 0444, pwmg_show_max_period, NULL);
static DEVICE_ATTR (period_ns_min, 0444, pwmg_show_min_period, NULL);
static DEVICE_ATTR (oneshot_ms, 0644, pwmg_show_oneshot_time, pwmg_store_oneshot_time);

#ifdef CONFIG_OF
static int pwmg_generic_parse_dt(struct device *dev,
                                  struct platform_pwmg_generic_data *data)
{
        struct device_node *node = dev->of_node;
        int ret;

        if (!node)
                return -ENODEV;

        memset(data, 0, sizeof(*data));

	/* determine the pwm-id */

        ret = of_property_read_u32(node, "pwm-id",
                                   &(data->pwm_id));
        if (ret < 0)
                return ret;

        /* determine the max-duty */

        ret = of_property_read_u32(node, "max-duty",
                                   &(data->max_duty));
        if (ret < 0)
                return ret;

	/* determine the dft-duty */	

	ret = of_property_read_u32(node, "dft-duty",
                                   &(data->dft_duty));
        if (ret < 0)
                return ret;

	/* determine the pwm-period-ns */

        ret = of_property_read_u32(node, "pwm-period-ns",
                                   &(data->pwm_period_ns));
        if (ret < 0)
                return ret;

	 /* determine the period-ns-min */

        ret = of_property_read_u32(node, "period-ns-min",
                                   &(data->period_ns_min));
        if (ret < 0)
                return ret;

	/* determine the period-ns-max */

        ret = of_property_read_u32(node, "period-ns-max",
                                   &(data->period_ns_max));
        if (ret < 0)
                return ret;

	/* determine the enable */

	ret = of_property_read_u32(node, "enable",
                                   &(data->enable));
        if (ret < 0)
                return ret;

        return 0;
}

static struct of_device_id pwmg_generic_of_match[] = {
        { .compatible = "pwm-generic" },
        { }
};

MODULE_DEVICE_TABLE(of, pwmg_generic_of_match);
#else
static int pwmg_generic_parse_dt(struct device *dev,
                                  struct platform_pwmg_backlight_data *data)
{
        return -ENODEV;
}
#endif

static int pwmg_generic_probe (struct platform_device *pdev) {

	struct platform_pwmg_generic_data *data = pdev->dev.platform_data;
	struct platform_pwmg_generic_data defdata;
	int ret;
	struct pwmg_data *pd;
	
	dev_info(&pdev->dev,"probe...\n");

	if (!data) {
		ret = pwmg_generic_parse_dt(&pdev->dev, &defdata);
                if (ret < 0) {
                        dev_err(&pdev->dev, "failed to find platform data\n");
                        return ret;
                }

                data = &defdata;
	}

	pd = kzalloc (sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		PWMG_ERR ("no memory for state");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pd->dev = &pdev->dev;
	pd->pwm = pwm_get(&pdev->dev, NULL);
        if (IS_ERR(pd->pwm)) {
                dev_info(&pdev->dev, "unable to request PWM, trying legacy API\n");
                pd->pwm = pwm_request(data->pwm_id, PWM_NAME_ID);
        }
	if (IS_ERR(pd->pwm)) {
		PWMG_ERR ("unable to request PWM");
		ret = PTR_ERR(pd->pwm);
		goto err_pwm;
	} else
		PWMG_DBG ("got pwm for backlight\n");

	data->reserved = pd;

	pd->duty = data->dft_duty;
	pd->max_duty = data->max_duty;
	
	pd->period = data->pwm_period_ns;
	pd->period_max = data->period_ns_max;
	pd->period_min = data->period_ns_min;

	pd->lth_duty = data->lth_duty *
		(data->pwm_period_ns / data->max_duty);

	pd->enable = data->enable;

	mutex_init(&pd->ops_lock);

	ret = device_create_file (pd->dev, &dev_attr_enable);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_duty);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_max_duty);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns_max);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns_min);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_oneshot_ms);
	if (ret < 0)
		goto err_fs;
	pwmg_update_status (pd);

	/* Create workqueue */
	pd->pwmg_workq = create_singlethread_workqueue("pwmg_oneshot");
        if (pd->pwmg_workq == NULL) {
                return -EINVAL;
        }
	INIT_DELAYED_WORK(&pd->pwmg_oneshot_work, pwmg_oneshot_work_handler );
	
	platform_set_drvdata(pdev, pd);
	
	return 0;

err_fs:
	pwm_free(pd->pwm);
err_pwm:
	kfree(pd);
err_alloc:
	return ret;
}


static int pwmg_generic_remove(struct platform_device *pdev) {

	struct platform_pwmg_generic_data *data = pdev->dev.platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;

	pwm_config (pd->pwm, 0, pd->period);
        pwm_disable (pd->pwm);
        pwm_free (pd->pwm);
        kfree (pd);
	return 0;
}


static int pwmg_generic_suspend(struct device *pdev) {
	return 0;
}


static int pwmg_generic_resume (struct device *pdev) {
	return 0;
}

static const struct dev_pm_ops pwmg_generic_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend        = pwmg_generic_suspend,
        .resume         = pwmg_generic_resume,
#endif
};

static struct platform_driver pwmg_generic_driver = {
	.driver		= {
		.name	= "pwm-generic",
		.of_match_table = of_match_ptr(pwmg_generic_of_match),
	},
	.probe		= pwmg_generic_probe,
	.remove		= pwmg_generic_remove,
};

module_platform_driver(pwmg_generic_driver);

MODULE_AUTHOR("DC MS SECO");
MODULE_DESCRIPTION("SECO generic PWM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-generic");

