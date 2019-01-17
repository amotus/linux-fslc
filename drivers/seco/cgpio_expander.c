

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/seco_cpld.h>


#define CGPIO_INFO(fmt, arg...) printk(KERN_INFO "SecoCGPIO: " fmt "\n" , ## arg)
#define CGPIO_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define CGPIO_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define DRV_VERSION     "1.0"



#define NR_GPIO    8
#define CGPIO_LBL  "seco_cgpio"
#define CAN_SLEEP  0

#define GPIO1      0x01
#define GPIO2      0x02
#define GPIO3      0x04
#define GPIO4      0x08
#define GPIO5      0x0F
#define GPIO6      0x20
#define GPIO7      0x40
#define GPIO8      0x80
#define GPIO_ALL   0xFF

#define REG_FW_REV          CPLD_REG_0
#define REG_GPIO_BUFFER     CPLD_REG_1
#define REG_GPIO_DIR        CPLD_REG_2
#define REG_GPIO_INT_MASK   CPLD_REG_3
#define REG_GPIO_INT_EN     CPLD_REG_4
#define REG_GPIO_INT_CONF   CPLD_REG_5
#define REG_GPIO_WR_MASK    CPLD_REG_6
#define REG_GPIO_INT        CPLD_REG_7

#define INT_IGNORED   0x0
#define INT_GLOBAL    0x1

#define WRITABLE     0x0
#define NOWRITABLE   0x1

#define DIR_OUT  0x0
#define DIR_IN   0x1

#define INT_MASK    1
#define INT_UNMASK  0

#define INT_EN      1
#define INT_DIS     0


struct cgpio_exp_chip {
	struct gpio_chip        gpio_chip;

	struct mutex            lock;
	struct mutex            lock_irq;

	uint8_t                 cache[8];

	struct irq_domain       *irq_domain;
	int                     irq;

};

static struct lock_class_key gpio_lock_class;


/* __________________________________________________________________________
* |                                                                          |
* |                              W/R BASIC FUNCTION                          |
* |__________________________________________________________________________|
*/
static void cgpio_exp_writeb (unsigned int reg, uint8_t value) {
	cpld_reg_write (reg, (uint16_t)value);
}


static void cgpio_exp_readb (unsigned int reg, uint8_t *data) {
	uint16_t value;
	cpld_reg_read (reg, &value);
	*data = (uint8_t)value;
}


static int cgpio_exp_sync_to_cache_reg (struct cgpio_exp_chip *cgpio_chip) {
	int i, nreg;
	int ret = 0;
	nreg = ARRAY_SIZE (cgpio_chip->cache);

	for ( i = 0 ; i < nreg && ret >= 0 ; i++ ) {
		cgpio_exp_readb (i, &cgpio_chip->cache[i]);
	}

	return (nreg - i);
}


static int cgpio_exp_sync_from_cache_reg (struct cgpio_exp_chip *cgpio_chip) {

	/* no all registers are to write */
	cgpio_exp_writeb (REG_GPIO_DIR, cgpio_chip->cache[REG_GPIO_DIR]);
	cgpio_exp_writeb (REG_GPIO_INT_MASK, cgpio_chip->cache[REG_GPIO_INT_MASK]);
	cgpio_exp_writeb (REG_GPIO_INT_EN, cgpio_chip->cache[REG_GPIO_INT_EN]);
	cgpio_exp_writeb (REG_GPIO_INT_CONF, cgpio_chip->cache[REG_GPIO_INT_CONF]);
	cgpio_exp_writeb (REG_GPIO_WR_MASK, cgpio_chip->cache[REG_GPIO_WR_MASK]);

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
static int cgpio_exp_get_value (struct gpio_chip *chip, unsigned offset) {
	uint8_t reg_value;
	int status = 0;
	struct cgpio_exp_chip *cgpio_chip;
	uint8_t mask = 1u << offset;

	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	mutex_lock (&cgpio_chip->lock);
	cgpio_exp_readb (REG_GPIO_BUFFER, &reg_value);
	mutex_unlock (&cgpio_chip->lock);

	status = (int)(reg_value & mask);
	return status;
}


static void __cgpio_exp_set_value (struct cgpio_exp_chip *cgpio_chip, unsigned mask, int value) {
	uint8_t reg_out = (value) ? cgpio_chip->cache[REG_GPIO_BUFFER] | (uint8_t)mask :
	   							cgpio_chip->cache[REG_GPIO_BUFFER] & (uint8_t)~mask;

	cgpio_exp_writeb (REG_GPIO_WR_MASK, 0xFF & ~mask);
	cgpio_exp_writeb (REG_GPIO_BUFFER, reg_out);

	cgpio_chip->cache[REG_GPIO_BUFFER] = reg_out;
}


static void cgpio_exp_set_value (struct gpio_chip *chip, unsigned offset, int value) {
	struct cgpio_exp_chip *cgpio_chip;
	uint8_t mask = 1u << offset;

	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	mutex_lock (&cgpio_chip->lock);
	__cgpio_exp_set_value (cgpio_chip, mask, value);
	mutex_unlock (&cgpio_chip->lock);
}


static int cgpio_exp_direction_input (struct gpio_chip *chip, unsigned offset) {
	struct cgpio_exp_chip *cgpio_chip;

	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	mutex_lock (&cgpio_chip->lock);
	cgpio_chip->cache[REG_GPIO_DIR] |= (1 << offset);
	cgpio_exp_writeb (REG_GPIO_DIR, cgpio_chip->cache[REG_GPIO_DIR]);
	mutex_unlock (&cgpio_chip->lock);

	return 0;
}


static int cgpio_exp_direction_output (struct gpio_chip *chip, unsigned offset, int value) {
	struct cgpio_exp_chip *cgpio_chip;
	unsigned mask = 1 << offset;

	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	mutex_lock (&cgpio_chip->lock);
	__cgpio_exp_set_value (cgpio_chip, mask, value);
	cgpio_chip->cache[REG_GPIO_DIR] &= ~mask;
	cgpio_exp_writeb (REG_GPIO_DIR, cgpio_chip->cache[REG_GPIO_DIR]);
	mutex_unlock (&cgpio_chip->lock);

	return 0;
}


static void cgpio_exp_dbg_show (struct seq_file *s, struct gpio_chip *chip) {
	struct cgpio_exp_chip *cgpio_chip;
	int i;
	unsigned mask = 1u;

	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	mutex_lock (&cgpio_chip->lock);

	/* sync cache with current register sertting */
	i = cgpio_exp_sync_to_cache_reg (cgpio_chip);
	if ( i < 0 ) {
		seq_printf (s, " I/O ERROR %d\n", i);
		goto done;
	}

	for ( i = 0 ; i < chip->ngpio ; i++, mask <<= 1) {
		const char  *label;

		label = gpiochip_is_requested (chip, i);
		if (!label)
			continue;

		seq_printf(s, " gpio-%-3d (%-12s) %s %s",
			chip->base + i, label,
			(cgpio_chip->cache[REG_GPIO_DIR] & mask) ? "in " : "out",
			(cgpio_chip->cache[REG_GPIO_BUFFER] & mask) ? "hi" : "lo");
		seq_printf(s, "\n");
	}

done:
	mutex_unlock (&cgpio_chip->lock);

}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



/* __________________________________________________________________________
* |                                                                          |
* |                                IRQ MANAGEMENT                            |
* |__________________________________________________________________________|
*/
static irqreturn_t cgpio_exp_hanlder_irq (int irq, void *data) {
	unsigned int child_irq, i;
	uint8_t reg_interrupt;
	struct cgpio_exp_chip *cgpio_chip = (struct cgpio_exp_chip *)data;

	cgpio_exp_readb (REG_GPIO_INT, &reg_interrupt);
	cgpio_chip->cache[REG_GPIO_INT] = reg_interrupt;

	for ( i = 0 ; i < cgpio_chip->gpio_chip.ngpio ; i++ ) {
		if ( BIT(i) & reg_interrupt ) {
			child_irq = irq_find_mapping (cgpio_chip->irq_domain, i);
			handle_nested_irq (child_irq);
		}
	}

	cgpio_exp_writeb (REG_GPIO_INT, 0xFF);

	return IRQ_HANDLED;
}


static void cgpio_exp_free_irq (struct cgpio_exp_chip *cgpio_chip) {
	unsigned int irq, i;

	free_irq (cgpio_chip->irq, cgpio_chip);

	for ( i = 0; i < cgpio_chip->gpio_chip.ngpio ; i++ ) {
		irq = irq_find_mapping (cgpio_chip->irq_domain, i);
		if ( irq > 0 )
			irq_dispose_mapping (irq);
	}

	irq_domain_remove (cgpio_chip->irq_domain);
}


static int cgpio_exp_gpio_to_irq (struct gpio_chip *chip, unsigned offset) {
	struct cgpio_exp_chip *cgpio_chip;
	cgpio_chip = container_of (chip, struct cgpio_exp_chip, gpio_chip);

	return irq_find_mapping(cgpio_chip->irq_domain, offset);
}


static void cgpio_exp_irq_mask (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;
	unsigned int pos;

	cgpio_chip = irq_data_get_irq_chip_data(data);
	pos = data->hwirq;

	cgpio_chip->cache[REG_GPIO_INT_EN] &= ~BIT(pos);
	cgpio_chip->cache[REG_GPIO_INT_MASK] |= BIT(pos);
}


static void cgpio_exp_irq_unmask (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;
	unsigned int pos;

	cgpio_chip = irq_data_get_irq_chip_data(data);
	pos = data->hwirq;

	cgpio_chip->cache[REG_GPIO_INT_EN] |= BIT(pos);
	cgpio_chip->cache[REG_GPIO_INT_MASK] &= ~BIT(pos);
}


static int cgpio_exp_irq_set_type (struct irq_data *data, unsigned int type) {
	struct cgpio_exp_chip *cgpio_chip;
	unsigned int pos = data->hwirq;
	int status = 0;

	cgpio_chip = irq_data_get_irq_chip_data(data);

	if ( type & IRQ_TYPE_EDGE_FALLING ) {
		cgpio_chip->cache[REG_GPIO_INT_CONF] &= ~BIT(pos);
	} else if ( type & IRQ_TYPE_LEVEL_HIGH ) {
		cgpio_chip->cache[REG_GPIO_INT_CONF] |= BIT(pos);
	} else
		status = -EINVAL;

	return status;
}


static void cgpio_exp_irq_bus_lock (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;

	cgpio_chip = irq_data_get_irq_chip_data(data);
	mutex_lock (&cgpio_chip->lock_irq);
}


static void cgpio_exp_irq_bus_unlock (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;

	cgpio_chip = irq_data_get_irq_chip_data(data);

	mutex_lock (&cgpio_chip->lock);
	cgpio_exp_writeb (REG_GPIO_INT_MASK, cgpio_chip->cache[REG_GPIO_INT_MASK]);
	cgpio_exp_writeb (REG_GPIO_INT_EN, cgpio_chip->cache[REG_GPIO_INT_EN]);
	cgpio_exp_writeb (REG_GPIO_INT_CONF, cgpio_chip->cache[REG_GPIO_INT_CONF]);
	mutex_unlock (&cgpio_chip->lock);

	mutex_unlock (&cgpio_chip->lock_irq);

}


static unsigned int cgpio_exp_irq_startup (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;

	cgpio_chip = irq_data_get_irq_chip_data(data);

	if ( gpiochip_lock_as_irq (&cgpio_chip->gpio_chip, data->hwirq) ) {
		CGPIO_ERR ("unable to lock HW IRQ %lu for IRQ usage", data->hwirq);
	}

	cgpio_exp_irq_unmask (data);
	return 0;
}


static void cgpio_exp_irq_shutdown (struct irq_data *data) {
	struct cgpio_exp_chip *cgpio_chip;

	cgpio_chip = irq_data_get_irq_chip_data(data);
	cgpio_exp_irq_mask (data);
	gpiochip_unlock_as_irq (&cgpio_chip->gpio_chip, data->hwirq);
}


static struct irq_chip cgpio_exp_irq_chip = {
	.name                 = "cgpio-exp-seco",
	.irq_mask             = cgpio_exp_irq_mask,
	.irq_unmask           = cgpio_exp_irq_unmask,
	.irq_set_type         = cgpio_exp_irq_set_type,
	.irq_bus_lock         = cgpio_exp_irq_bus_lock,
	.irq_bus_sync_unlock  = cgpio_exp_irq_bus_unlock,
	.irq_startup          = cgpio_exp_irq_startup,
	.irq_shutdown         = cgpio_exp_irq_shutdown,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



/* __________________________________________________________________________
* |                                                                          |
* |                                DRIVER SETUP                              |
* |__________________________________________________________________________|
*/
static int cgpio_exp_irq_setup (struct cgpio_exp_chip *cgpio_chip) {
	int err, irq, i, done;
	struct gpio_chip *chip = &cgpio_chip->gpio_chip;

	cgpio_chip->irq_domain = irq_domain_add_linear(chip->of_node, chip->ngpio,
								&irq_domain_simple_ops, cgpio_chip);
	if ( !cgpio_chip->irq_domain ) {
		CGPIO_ERR ("cannot assign irq domain");
		err = -ENODEV;
		goto err_irq_domain_add;
	}

	err = devm_request_threaded_irq (chip->parent, cgpio_chip->irq, NULL, cgpio_exp_hanlder_irq,
										IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
										dev_name(chip->parent), cgpio_chip);
	if (err != 0) {
		CGPIO_ERR ("unable to request IRQ#%d: %d", cgpio_chip->irq, err);
		goto err_request_thr_irq;
	}

	chip->to_irq = cgpio_exp_gpio_to_irq;

	for ( i = 0 ; i < chip->ngpio ; i++ ) {
		irq = irq_create_mapping (cgpio_chip->irq_domain, i);
		if ( irq < 0 ) {
			CGPIO_ERR ("cannot map irq");
			done = i;
			err = -EINVAL;
			goto err_map_irq;
		}
		irq_set_lockdep_class (irq, &gpio_lock_class);
		irq_set_chip_data (irq, cgpio_chip);
		irq_set_chip (irq, &cgpio_exp_irq_chip);
		irq_set_nested_thread (irq, true);
	}

	return 0;
err_map_irq:
	while ( --done ) {
		irq = irq_find_mapping (cgpio_chip->irq_domain, i);
		if ( irq > 0 )
			irq_dispose_mapping (irq);
	}
err_request_thr_irq:
	irq_domain_remove (cgpio_chip->irq_domain);
err_irq_domain_add:
	return err;
}


static int cgpio_exp_setup (struct cgpio_exp_chip *cgpio_chip, struct device *dev,
							unsigned base) {
	int ret, err, status;

	/* init gpio chip structure */
	cgpio_chip->gpio_chip.get = cgpio_exp_get_value;
	cgpio_chip->gpio_chip.set = cgpio_exp_set_value;
	cgpio_chip->gpio_chip.direction_input = cgpio_exp_direction_input;
	cgpio_chip->gpio_chip.direction_output = cgpio_exp_direction_output;
	cgpio_chip->gpio_chip.dbg_show = cgpio_exp_dbg_show;

	cgpio_chip->gpio_chip.of_gpio_n_cells = 2;
	cgpio_chip->gpio_chip.of_node = dev->of_node;

	cgpio_chip->gpio_chip.ngpio = NR_GPIO;
	cgpio_chip->gpio_chip.label = CGPIO_LBL;

	cgpio_chip->gpio_chip.base = base;
	cgpio_chip->gpio_chip.can_sleep = CAN_SLEEP;
	cgpio_chip->gpio_chip.parent = dev;
	cgpio_chip->gpio_chip.owner = THIS_MODULE;

	mutex_init (&cgpio_chip->lock);
	mutex_init (&cgpio_chip->lock_irq);

	/* init registers configuration */
	cgpio_chip->cache[REG_GPIO_DIR]      = 0xFF;	// all gpios as input
	cgpio_chip->cache[REG_GPIO_INT_MASK] = 0x00;    // all interrupt masked
	cgpio_chip->cache[REG_GPIO_INT_EN]   = 0x00;    // all interrupt disabled
	cgpio_chip->cache[REG_GPIO_INT_CONF] = 0x00;    // all interrupt over falling edge
	cgpio_chip->cache[REG_GPIO_WR_MASK]  = 0x00;	// all output gpios as writable
	cgpio_exp_sync_from_cache_reg (cgpio_chip);

	/* init cache with current register sertting */
	ret = cgpio_exp_sync_to_cache_reg (cgpio_chip);
	if ( ret != 0 ) {
		CGPIO_ERR ("cannot read external register... no sync");
		err = -ret;
		goto err_sync_cache;
	}

	/* add this gpio bank */
	status = gpiochip_add (&cgpio_chip->gpio_chip);
	if ( status < 0 ) {
		CGPIO_ERR ("cannot add gpio bank to the system");
		err = status;
		goto err_gpio_add;
	}

	status = cgpio_exp_irq_setup (cgpio_chip);
	if ( status ) {
		CGPIO_ERR ("cannot setup gpio irq");
		err = status;
		goto err_irq_setup;
	}

	return 0;
err_irq_setup:
err_gpio_add:
err_sync_cache:
	return err;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



static int cgpio_exp_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	struct cgpio_exp_chip *cgpio_chip;
	int ret, err;
	unsigned base = -1;

	/* check that the CPLD firmware is a GPIO expander */
	if ( !cpld_is_gpio () ) {
		CGPIO_ERR ("the CPLD firmware is no a GPIO expander");
		err = EINVAL;
		goto err_cpld;
	}

	cgpio_chip = kzalloc (sizeof (struct cgpio_exp_chip), GFP_KERNEL);
	if ( !cgpio_chip ) {
		CGPIO_ERR ("cannot allocate memory for structure data");
		err = -ENOMEM;
		goto err_data_allocate;
	}

	platform_set_drvdata (pdev, cgpio_chip);

	cgpio_chip->irq = irq_of_parse_and_map (dp, 0);

	ret = cgpio_exp_setup (cgpio_chip, &pdev->dev, base);
	if ( ret != 0 ) {
		CGPIO_ERR ("cannot driver setup");
		err = ret;
		goto err_setup;
	}

	CGPIO_INFO ("cgpio_expander driver ver %s probed!!!", DRV_VERSION);
	return 0;
err_setup:
	kfree (cgpio_chip);
err_data_allocate:
err_cpld:
	return err;
}


static int cgpio_exp_remove (struct platform_device *pdev) {

	struct cgpio_exp_chip *cgpio_chip = (struct cgpio_exp_chip *)platform_get_drvdata (pdev);

	cgpio_exp_free_irq (cgpio_chip);

	gpiochip_remove (&cgpio_chip->gpio_chip);

	kfree (cgpio_chip);

	return 0;
}


static const struct of_device_id seco_cgpio_exp_match[] = {
	{ .compatible = "seco,cgpio_expander" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_cgpio_exp_match);


static struct platform_driver cgpio_exp_driver = {
	.driver = {
		.name		    = "cpgio_expander",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_cgpio_exp_match,
	},
	.probe  = cgpio_exp_probe,
	.remove = cgpio_exp_remove,
};



static int __init cgpio_exp_init(void) {
	return platform_driver_register (&cgpio_exp_driver);
}

subsys_initcall(cgpio_exp_init);


static void __exit cgpio_exp_exit (void) {
	return platform_driver_unregister (&cgpio_exp_driver);
}

module_exit(cgpio_exp_exit);



MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("SECO GPIO Expander over CPLD logic");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
