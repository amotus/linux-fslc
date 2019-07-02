

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/sio-w83627.h>


#define SIO_GPIO_INFO(fmt, arg...) printk(KERN_INFO "GPIO SIO W83627: " fmt "\n" , ## arg)
#define SIO_GPIO_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define SIO_GPIO_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define DRV_VERSION     "1.0"

#define NR_GPIO_BANK    6
#define NR_GPIO         8

#define NUM_REG         4
#define MAX_N_STRAP     3

#define REG_REGISTER    0
#define REG_DATA        1
#define REG_STATUS      2
#define REG_INV         3

#define REG_ENABLE      (uint8_t)0x30

/*  In this moment only GPIO2 and GPIO3 are supported */
static int valid_gpio_id[] = { 2, 3 };

static int gpio_logical_device[] = { 0, 9, 9, 9, 9, 7 };


static uint8_t config_reg_map[NR_GPIO_BANK][NUM_REG] = {
	[2] = {
		[REG_REGISTER] = 0xE3,
		[REG_DATA]     = 0xE4,
		[REG_STATUS]   = 0xE5,
		[REG_INV]      = 0xE6
	},
	[3] = {
		[REG_REGISTER] = 0xF0,
		[REG_DATA]     = 0xF1,
		[REG_STATUS]   = 0xF2,
		[REG_INV]      = 0xE7
	},
};


static uint8_t mux_reg_map[NR_GPIO_BANK][MAX_N_STRAP][3] = {
	[2] = {
	/*   REG   AND    OR	*/
		{0x29, 0x06, 0x02},   // for gpio 0, 1
		{0x24, 0x02, 0x00},   // for gpio 2, 3
		{0x2A, 0x01, 0x01},   // for gpio 4, 5, 6, 7
	},
	[3] = {
		{0x2C, 0xF0, 0x00},   // for all gpio
		{},
		{},
	},
};


#define NR_GPIO       8
#define SIO_GPIO_LBL  "seco_sio_gpio"
#define CAN_SLEEP     0

#define GPIO1      0x01
#define GPIO2      0x02
#define GPIO3      0x04
#define GPIO4      0x08
#define GPIO5      0x0F
#define GPIO6      0x20
#define GPIO7      0x40
#define GPIO8      0x80
#define GPIO_ALL   0xFF


struct sio_gpio_exp_chip {
	struct gpio_chip        gpio_chip;
	int                     gpio_id;
	int                     gpio_ldev;
	struct sio_w83627_ops   *sio_ops;

	struct mutex            lock;

	uint8_t                 cache[NUM_REG];

	uint8_t                 *conf_reg_map;
	uint8_t                 (*mux_reg_map)[3];
};




#define sio_gpio_read(sio_chip, addr, data)    \
	(sio_chip)->sio_ops->read (addr, data, (sio_chip)->gpio_ldev)

#define sio_gpio_write(sio_chip, addr, data)    \
	(sio_chip)->sio_ops->write (addr, data, (sio_chip)->gpio_ldev)

#define sio_conf_reg(sio_chip, reg)            \
	(sio_chip)->conf_reg_map[(reg)]



/* __________________________________________________________________________
* |                                                                          |
* |                             HW SETTING FUNCTION                          |
* |__________________________________________________________________________|
*/
static void sio_gpio_wb_active_gpio (struct sio_gpio_exp_chip *sio_gpio_chip, int en) {
	uint8_t val = 0;
	sio_gpio_chip->sio_ops->read (REG_ENABLE, &val, sio_gpio_chip->gpio_ldev);
	val = en ? val | (1u << (sio_gpio_chip->gpio_id - 2)) :
		val & ~(1u << (sio_gpio_chip->gpio_id - 2));
	sio_gpio_chip->sio_ops->write (REG_ENABLE, val, sio_gpio_chip->gpio_ldev);
}


static void sio_gpio_wb_set_muxing (struct sio_gpio_exp_chip *sio_gpio_chip) {
	int i;
	uint8_t (*mux_v)[3] = sio_gpio_chip->mux_reg_map;
	uint8_t val = 0;

	for ( i = 0 ; i < MAX_N_STRAP ; i++ ) {
		if ( mux_v[i] == 0 )
			break;
		sio_gpio_chip->sio_ops->g_read (mux_v[i][0], &val);
		val &= ~mux_v[i][1];
		val |= mux_v[i][2];
		sio_gpio_chip->sio_ops->g_write (mux_v[i][0], val);
	}
}


static void sio_gpio_wb_set_info (struct sio_gpio_exp_chip *sio_gpio_chip) {
	sio_gpio_chip->conf_reg_map = &config_reg_map[sio_gpio_chip->gpio_id][0];
	sio_gpio_chip->mux_reg_map = &mux_reg_map[sio_gpio_chip->gpio_id][0];
}


static int is_valid_gpio_id (int gpio_id) {
	int i;
	for ( i = 0 ; i < ARRAY_SIZE(valid_gpio_id) ; i++ ) {
		if ( valid_gpio_id[i] == gpio_id )
			break;
	}

	return  i == ARRAY_SIZE(valid_gpio_id) ? 0 : 1;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                             CACHE SYNC FUNCTION                          |
* |__________________________________________________________________________|
*/
static int sio_gpio_exp_sync_to_cache_reg (struct sio_gpio_exp_chip *sio_gpio_chip) {
	int i, nreg;
	int ret = 0;
	nreg = ARRAY_SIZE (sio_gpio_chip->cache);

	for ( i = 0 ; i < nreg && ret >= 0 ; i++ ) {
		sio_gpio_write (sio_gpio_chip, i, sio_gpio_chip->cache[i]);
	}

	return (nreg - i);
}


static int sio_gpio_exp_sync_from_cache_reg (struct sio_gpio_exp_chip *sio_gpio_chip) {

	sio_gpio_write (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_REGISTER),
		   sio_gpio_chip->cache[REG_REGISTER]);
	sio_gpio_write (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_INV),
		   sio_gpio_chip->cache[REG_INV]);
	return 0;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



/* __________________________________________________________________________
* |                                                                          |
* |                             GPIO CHIP INTERFACE                          |
* |__________________________________________________________________________|
*/
static int sio_gpio_exp_get_value (struct gpio_chip *chip, unsigned offset) {
	uint8_t reg_value;
	int status = 0;
	struct sio_gpio_exp_chip *sio_gpio_chip;
	uint8_t mask = 1u << offset;

	sio_gpio_chip = container_of (chip, struct sio_gpio_exp_chip, gpio_chip);

	mutex_lock (&sio_gpio_chip->lock);
	sio_gpio_read (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_DATA), &reg_value);
	mutex_unlock (&sio_gpio_chip->lock);

	status = (int)(reg_value & mask);
	return status;
}


static void __sio_gpio_exp_set_value (struct sio_gpio_exp_chip *sio_gpio_chip, unsigned mask, int value) {
	uint8_t reg_out = (value) ? sio_gpio_chip->cache[REG_DATA] | (uint8_t)mask :
	   							sio_gpio_chip->cache[REG_DATA] & (uint8_t)~mask;

	sio_gpio_write (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_DATA), reg_out);

	sio_gpio_chip->cache[REG_DATA] = reg_out;
}


static void sio_gpio_exp_set_value (struct gpio_chip *chip, unsigned offset, int value) {
	struct sio_gpio_exp_chip *sio_gpio_chip;
	uint8_t mask = 1u << offset;

	sio_gpio_chip = container_of (chip, struct sio_gpio_exp_chip, gpio_chip);

	mutex_lock (&sio_gpio_chip->lock);
	__sio_gpio_exp_set_value (sio_gpio_chip, mask, value);
	mutex_unlock (&sio_gpio_chip->lock);
}


static int sio_gpio_exp_direction_input (struct gpio_chip *chip, unsigned offset) {
	struct sio_gpio_exp_chip *sio_gpio_chip;

	sio_gpio_chip = container_of (chip, struct sio_gpio_exp_chip, gpio_chip);

	mutex_lock (&sio_gpio_chip->lock);
	sio_gpio_chip->cache[REG_REGISTER] |= (1 << offset);
	sio_gpio_write (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_REGISTER),
		   sio_gpio_chip->cache[REG_REGISTER]);
	mutex_unlock (&sio_gpio_chip->lock);

	return 0;
}


static int sio_gpio_exp_direction_output (struct gpio_chip *chip, unsigned offset, int value) {
	struct sio_gpio_exp_chip *sio_gpio_chip;
	unsigned mask = 1 << offset;

	sio_gpio_chip = container_of (chip, struct sio_gpio_exp_chip, gpio_chip);

	mutex_lock (&sio_gpio_chip->lock);
	__sio_gpio_exp_set_value (sio_gpio_chip, mask, value);
	sio_gpio_chip->cache[REG_REGISTER] &= ~mask;
	sio_gpio_write (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, REG_REGISTER),
			sio_gpio_chip->cache[REG_REGISTER]);
	mutex_unlock (&sio_gpio_chip->lock);

	return 0;
}


static void sio_gpio_exp_dbg_show (struct seq_file *s, struct gpio_chip *chip) {
}

/* __________________________________________________________________________
* |__________________________________________________________________________|
*/

/* __________________________________________________________________________
* |                                                                          |
* |                              SYSFS INTERFACE                             |
* |__________________________________________________________________________|
*/
static ssize_t sys_sio_gpio_dump_global_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	struct sio_gpio_exp_chip *sio_gpio_chip = (struct sio_gpio_exp_chip *)dev_get_drvdata (dev);
	unsigned char msg[500];
	unsigned char tmp[50];
	uint8_t value;
	int i;

	sprintf (msg, "\n#\tvalue");
	for ( i = 0 ; i < NUM_REG ; i++ ) {
		sio_gpio_read (sio_gpio_chip, sio_conf_reg(sio_gpio_chip, i), &value);

		sprintf (tmp, "\n0x%02X\t0x%02X", sio_conf_reg(sio_gpio_chip, i), value);
		strcat (msg, tmp);
	}
	sio_gpio_read (sio_gpio_chip, REG_ENABLE, &value);
	sprintf (tmp, "\nActive GPIO reg: %#X", value);
	strcat (msg, tmp);

	return sprintf (buf, "%s\n", msg);
}

static DEVICE_ATTR(dump_regs, S_IRUGO, sys_sio_gpio_dump_global_regs, NULL);


static struct attribute *sio_gpio_attrs[] = {
	&dev_attr_dump_regs.attr,
	NULL,
};


static struct attribute_group sio_gpio_attr_group = {
	.attrs = sio_gpio_attrs,
};



/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


static int sio_gpio_exp_setup (struct sio_gpio_exp_chip *sio_gpio_chip, struct device *dev,
		unsigned base) {
	int err, status;

	/* init gpio chip structure */
	sio_gpio_chip->gpio_chip.get = sio_gpio_exp_get_value;
	sio_gpio_chip->gpio_chip.set = sio_gpio_exp_set_value;
	sio_gpio_chip->gpio_chip.direction_input = sio_gpio_exp_direction_input;
	sio_gpio_chip->gpio_chip.direction_output = sio_gpio_exp_direction_output;
	sio_gpio_chip->gpio_chip.dbg_show = sio_gpio_exp_dbg_show;

	sio_gpio_chip->gpio_chip.of_gpio_n_cells = 2;
	sio_gpio_chip->gpio_chip.of_node = dev->of_node;

	sio_gpio_chip->gpio_chip.ngpio = NR_GPIO;
	sio_gpio_chip->gpio_chip.label = SIO_GPIO_LBL;

	sio_gpio_chip->gpio_chip.base = base;
	sio_gpio_chip->gpio_chip.can_sleep = CAN_SLEEP;
	sio_gpio_chip->gpio_chip.parent = dev;
	sio_gpio_chip->gpio_chip.owner = THIS_MODULE;

	mutex_init (&sio_gpio_chip->lock);

	/* init registers configuration */
	sio_gpio_chip->cache[REG_REGISTER]   = 0xFF;	// all gpios as input
	sio_gpio_chip->cache[REG_INV]        = 0x00;    // no inversion value

	sio_gpio_exp_sync_from_cache_reg (sio_gpio_chip);
	sio_gpio_exp_sync_to_cache_reg (sio_gpio_chip);

	/* add this gpio bank */
	status = gpiochip_add (&sio_gpio_chip->gpio_chip);
	if ( status < 0 ) {
		SIO_GPIO_ERR ("cannot add gpio bank to the system");
		err = status;
		goto err_gpio_add;
	}

	return 0;
err_gpio_add:
	return err;
}




static int sio_gpio_wb_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	struct sio_gpio_exp_chip *sio_gpio_chip;
	const __be32 *addr;
	int ret, err, len, gpio_id;
	unsigned base = -1;

	sio_gpio_chip = kzalloc (sizeof (struct sio_gpio_exp_chip), GFP_KERNEL);
	if ( !sio_gpio_chip ) {
		SIO_GPIO_ERR ("cannot allocate memory for structure data");
		err = -ENOMEM;
		goto err_data_allocate;
	}

	platform_set_drvdata (pdev, sio_gpio_chip);

	sio_gpio_chip->sio_ops = (struct sio_w83627_ops *)dev_get_platdata(&pdev->dev);

	if ( !sio_gpio_chip->sio_ops ) {
		SIO_GPIO_ERR ("cannot obtain ops structure data");
		err = -EINVAL;
		goto err_ops_data;
	}
	if ( !sio_gpio_chip->sio_ops->read || !sio_gpio_chip->sio_ops->write ||
			!sio_gpio_chip->sio_ops->g_read || !sio_gpio_chip->sio_ops->g_write) {
		SIO_GPIO_ERR ("write and read operation not assigned");
		err= -EINVAL;
		goto err_ops_data;
	}

	addr = of_get_property(dp, "gpio_id", &len);
	if ( !addr || (len < sizeof(int))) {
		SIO_GPIO_ERR ("unable to obtain gpio ID!!!");
		err = -EINVAL;
		goto err_gpio_id;
	}
	gpio_id = be32_to_cpup (addr);

	if ( !is_valid_gpio_id (gpio_id) ) {
		SIO_GPIO_ERR ("invalid gpio ID (%d)!!!", gpio_id);
		err = -EINVAL;
		goto err_gpio_id;
	}
	sio_gpio_chip->gpio_id = gpio_id;
	sio_gpio_chip->gpio_ldev = gpio_logical_device[gpio_id];
	sio_gpio_wb_active_gpio (sio_gpio_chip, 1);

	sio_gpio_wb_set_info (sio_gpio_chip);
	sio_gpio_wb_set_muxing (sio_gpio_chip);

	ret = sio_gpio_exp_setup (sio_gpio_chip, &pdev->dev, base);
	if ( ret != 0 ) {
		SIO_GPIO_ERR ("cannot driver setup");
		err = ret;
		goto err_setup;
	}

	err = sysfs_create_group (&pdev->dev.kobj, &sio_gpio_attr_group);
	if ( err ) {
		SIO_GPIO_ERR ("cannot create sysfs interface");
		goto err_sysfs;
	}

	SIO_GPIO_INFO ("LPC SuperIO GPIO-%d driver ver %s probed!!!", gpio_id, DRV_VERSION);
	return 0;
err_sysfs:
err_setup:
err_ops_data:
err_gpio_id:
	kfree (sio_gpio_chip);
err_data_allocate:
	return err;
}


static int sio_gpio_wb_remove (struct platform_device *pdev) {
	return 0;
}


static const struct of_device_id seco_sio_gpio_wb_match[] = {
	{ .compatible = "nuvoton,w83627dhg_gpio" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_sio_gpio_wb_match);


static struct platform_driver sio_gpio_wb_driver = {
	.driver = {
		.name		    = "sio_gpio_w83627",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_sio_gpio_wb_match,
	},
	.probe  = sio_gpio_wb_probe,
	.remove = sio_gpio_wb_remove,
};



static int __init sio_gpio_wb_init(void) {
	return platform_driver_register (&sio_gpio_wb_driver);
}

subsys_initcall(sio_gpio_wb_init);


static void __exit sio_gpio_wb_exit (void) {
	return platform_driver_unregister (&sio_gpio_wb_driver);
}

module_exit(sio_gpio_wb_exit);



MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("Nuvoton LPC SuperIO GPIO Expander - SECO version");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
