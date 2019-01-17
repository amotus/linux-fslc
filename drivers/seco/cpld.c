
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/sysctl.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>


#include <linux/seco_cpld.h>


#define DRV_VERSION     "1.0"


#define REG_ADDR(x)  (CPLD_OFFSET_BASE + (x*2))
#define WEIM_ADDR(x) (cpld_d->virt + (x))

#define CPLD_INFO(fmt, arg...) printk(KERN_INFO "CPLD: " fmt "\n" , ## arg)
#define CPLD_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define CPLD_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

static struct cpld_data *cpld_d = NULL;


void cpld_reg_read (unsigned int addr, uint16_t *data) {
	*data = readw (WEIM_ADDR(REG_ADDR(addr)));
}


void cpld_reg_write (unsigned int addr, uint16_t value) {
	writew (value, WEIM_ADDR(REG_ADDR(addr)));
}


void cpld_read (unsigned int addr, uint16_t *data) {
	*data = readw (WEIM_ADDR(addr << 1));
}


void cpld_write (unsigned int addr, uint16_t value) {
	writew (value, WEIM_ADDR(addr << 1));
}


int inline cpld_get_membase (void) {
	return cpld_d->mem_addr_base;
}


int cpld_get_revision (void) {
	uint16_t rev;
	cpld_reg_read (REG_REVISION, &rev);
	return rev;
}


int cpld_is_gpio (void) {
	int is_gpio = 0;
	uint16_t rev = cpld_get_revision ();
	switch (rev) {
		case 0x3a06:
		case 0x3a04:
		case 0x3a02:
			is_gpio = 1;
			break;
		default:
			is_gpio = 0;
			break;
	}
	return is_gpio;
}


int cpld_is_lpc (void) {
	return !cpld_is_gpio ();
}


void dump_reg (void) {
	int i;
	uint16_t val;

	for ( i = 0 ; i < NREG ; i++ ) {
		cpld_reg_read (i, &val);
		CPLD_INFO ("dump register %d -> 0x%04X", i, val);
	}
}


#define CPLD_NAME   "CPLD_device"

int __cpld_init (struct device_node *dp, struct resource *resource) {
	uint16_t rev = 0;
	int err = 0;

	if ( cpld_d != NULL ) {
		CPLD_ERR ("CPLD resource already initialized");
		err = EINVAL;
		goto err_out;
	}

	cpld_d = kzalloc(sizeof(struct cpld_data), GFP_KERNEL);
	if (cpld_d == NULL) {
		err = ENOMEM;
		goto err_out;
	}

	if ( of_address_to_resource(dp, 0, resource) ) {
		err = EINVAL;
		goto err_out;
	}

	CPLD_INFO ("platform CPLD device: %.8llx at %.8llx",
			(unsigned long long)resource_size(&resource[0]),
			(unsigned long long)resource[0].start);

	if (!request_mem_region(resource[0].start,
				resource_size(&resource[0]),
				CPLD_NAME)) {
		CPLD_ERR ("Could not reserve memory region");
		err = -ENOMEM;
		goto err_out;
	}
	pr_debug ("%s: memory request done!\n", __func__);

	cpld_d->name = CPLD_NAME;
	cpld_d->mem_addr_base = resource[0].start;
	cpld_d->mem_size = resource_size(&resource[0]);
	cpld_d->virt = ioremap_nocache (cpld_d->mem_addr_base, cpld_d->mem_size);
	if (cpld_d->virt == NULL) {
		CPLD_ERR ("Failed to ioremap seco CPLD region");
		err = -EIO;
		goto err_out;
	}

	mutex_init (&cpld_d->bus_lock);

	cpld_reg_read (REG_REVISION, &rev);
	CPLD_INFO ("ver.:  %04x", rev);
	return 0;
err_out:
	return err;
}



/* __________________________________________________________________________
* |__________________________________________________________________________|
*/
static ssize_t sys_cpld_dump_regs_show (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	unsigned char msg[500];
	unsigned char tmp[50];
	int i;
	uint16_t value;

	sprintf (msg, "\n#\tvalue");
	for ( i = 0 ; i < NREG ; i++ ) {
		cpld_reg_read (i, &value);
		sprintf (tmp, "\n%02d\t0x%04X", i, value);
		strcat (msg, tmp);
	}

	return sprintf (buf, "%s\n", msg);
}


static ssize_t sys_cpld_version_show (struct device *dev, struct device_attribute *attr,
									char *buf)
{
	uint16_t ver;
	cpld_reg_read (REG_REVISION, &ver);
	return sprintf(buf, "0x%04X\n",ver);
}

static DEVICE_ATTR(dump_regs, S_IRUGO, sys_cpld_dump_regs_show, NULL);
static DEVICE_ATTR(cpld_ver, S_IRUGO, sys_cpld_version_show, NULL);


static struct attribute *cpld_attrs[] = {
	&dev_attr_dump_regs.attr,
	&dev_attr_cpld_ver.attr,
	NULL,
};


static struct attribute_group cpld_attr_group = {
	.attrs = cpld_attrs,
};



/* __________________________________________________________________________
* |__________________________________________________________________________|
*/
#define TYPE_GPIO  0
#define TYPE_LPC   1


static int clpd_client_register (struct device_node *dp, struct device *parent_dev) {
	int ret;
	u32 type = 0;
	u32 ctype = (u32)cpld_is_gpio ();
	struct device_node *child;
	const char *en_code = "okay";

	for_each_child_of_node (dp, child) {
		ret = of_property_read_u32 (child, "type", (u32 *)&type);
		if ( ret == 0 ) {
			if ( (ctype && (type == TYPE_GPIO)) ||
					(!ctype && (type == TYPE_LPC)) ) {

				of_property_write_string (child, "status", en_code);

			}
		} else {
			CPLD_ERR ("failed to create device");
		}
	}

	ret = of_platform_populate (dp,  NULL, NULL, parent_dev);
	if ( ret ) {
		CPLD_ERR ("failed to create device");
	} else {
		CPLD_INFO ("device created");
	}

	return !ret;
}


static int cpld_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	struct resource res;
	int error = 0;
	int registred;
	__cpld_init (dp, &res);
	registred = clpd_client_register (dp, &pdev->dev);
	if ( registred )
		error = sysfs_create_group(&pdev->dev.kobj, &cpld_attr_group);
	return error;
}


static int cpld_remove (struct platform_device *pdev) {
	return 0;
}


static const struct of_device_id seco_cpld_match[] = {
	{ .compatible = "seco,cpld" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_cpld_match);


static struct platform_driver cpld_driver = {
	.driver = {
		.name		    = "cpld",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_cpld_match,
	},
	.probe  = cpld_probe,
	.remove = cpld_remove,
};



static int __init cpld_init(void) {
	return platform_driver_register (&cpld_driver);
}

subsys_initcall(cpld_init);


static void __exit cpld_exit (void) {
	return platform_driver_unregister (&cpld_driver);
}

module_exit(cpld_exit);




MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("SECO CPLD interface");
MODULE_ALIAS("platform:cpld");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
