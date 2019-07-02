

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

#include <linux/seco_lpc.h>
#include <linux/seco_cpld.h>

#define LPC_INFO(fmt, arg...) printk(KERN_INFO "LPC Bridge: " fmt "\n" , ## arg)
#define LPC_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define LPC_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define LPC_REG_IRQ_BUFFER     CPLD_REG_1
#define LPC_REG_MEM_PAGE_SEL   CPLD_REG_2
#define LPC_REG_IRQ_MASK       CPLD_REG_3
#define LPC_REG_LPC_BUSY       CPLD_REG_4
#define LPC_REG_IRQ_CONF       CPLD_REG_5

#define NR_IRQ                3

#define LPC_BUSY              0x1

#define NR_RETRIES            10
#define LPC_TIME_OUT          100

#define IRQ_MASKERED          0x00
#define IRQ_UNMASKERED        0x01

#define LPC_IRQ_MASKABLE      0x1
#define LPC_IRQ_UNMASKABLE    0x0

#define NR_LPC_SLOTS          16

#define DRV_VERSION     "1.0"


struct lpc_table_element *lpc_table;


struct lpc_data {
	const char           *name;
	struct device        *dev;

	uint16_t             cache[6];

	struct mutex         bus_lock;
	struct mutex         lock_irq;

	resource_size_t      mem_addr_base;
	unsigned long        mem_base;

	struct irq_domain    *irq_domain;
	int                  irq;

};



struct lpc_data *lpc_d;

static struct lock_class_key lpc_lock_class;

/* __________________________________________________________________________
* |                                                                          |
* |                            BASIC I/O FUNCTIONS                           |
* |__________________________________________________________________________|
*/
static void lpc_reg_read (unsigned int reg, uint16_t *data) {
	cpld_reg_read (reg, data);
}


static void lpc_reg_write (unsigned int reg, uint16_t value) {
	cpld_reg_write (reg, value);
}


static int inline lpc_read_status (void) {
	uint16_t state;
	cpld_reg_read (LPC_REG_LPC_BUSY, &state);
	return ((state & 0x8000) >> 15);
}

int lpc_readw (unsigned long mem_addr, uint16_t *data) {
	unsigned long orig_jiffies;
	int try;
	int res = 0;
	int status = -EIO;

	orig_jiffies = jiffies;
	for (try = 0 ; try <= NR_RETRIES ; try++) {
		res = lpc_read_status();
		if (res != LPC_BUSY) {
			cpld_read (mem_addr, data);
			status = res;
			break;
		}
		if (time_after (jiffies, orig_jiffies + LPC_TIME_OUT)) {
			LPC_ERR ("read operation TIMEOUT!");
			break;
		}
	}

	return status;
}


int lpc_writew (unsigned long mem_addr, uint16_t value) {
	unsigned long orig_jiffies;
	int try;
	int res = 0;
	int status = -EIO;

	orig_jiffies = jiffies;
	for (try = 0 ; try <= NR_RETRIES ; try++) {
		res = lpc_read_status();
		if (res != LPC_BUSY) {
			cpld_write (mem_addr, value);
			status = res;
			break;
		}
		if (time_after (jiffies, orig_jiffies + LPC_TIME_OUT)) {
			LPC_ERR ("read operation TIMEOUT!");
			break;
		}
	}

	return status;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                                IRQ MANAGEMENT                            |
* |__________________________________________________________________________|
*/
#include <asm/delay.h>
static irqreturn_t lpc_bridge_hanlder_irq (int irq, void *data) {
	unsigned int child_irq, i;
	uint16_t reg_interrupt, mask, conf;
	int count;
	struct lpc_data *lpc_d = (struct lpc_data *)data;

	lpc_reg_read (LPC_REG_IRQ_BUFFER, &reg_interrupt);
	lpc_reg_read (LPC_REG_IRQ_MASK, &mask);
	lpc_reg_read (LPC_REG_IRQ_CONF, &conf);
	lpc_d->cache[LPC_REG_IRQ_BUFFER] = reg_interrupt;

	for ( i = 0 ; i < NR_LPC_SLOTS ; i++ ) {
		if ( BIT(i) & reg_interrupt ) {
			child_irq = irq_find_mapping (lpc_d->irq_domain, i);
			handle_nested_irq (child_irq);
			count++;
		}
	}

	return IRQ_HANDLED;
}


static void lpc_bridge_irq_disable (struct irq_data *data) {
	struct lpc_data *lpc_d;
	unsigned int pos;

	lpc_d = irq_data_get_irq_chip_data (data);
	pos = data->hwirq;

	lpc_d->cache[LPC_REG_IRQ_MASK] &= ~BIT(pos);
}


static void lpc_bridge_irq_mask (struct irq_data *data) {
	struct lpc_data *lpc_d;
	unsigned int pos;

	lpc_d = irq_data_get_irq_chip_data (data);
	pos = data->hwirq;

	lpc_d->cache[LPC_REG_IRQ_MASK] &= ~BIT(pos);
}


static void lpc_bridge_irq_enable (struct irq_data *data) {
	struct lpc_data *lpc_d;
	unsigned int pos;

	lpc_d = irq_data_get_irq_chip_data (data);
	pos = data->hwirq;

	lpc_d->cache[LPC_REG_IRQ_MASK] |= BIT(pos);
}


static void lpc_bridge_irq_unmask (struct irq_data *data) {
	struct lpc_data *lpc_d;
	unsigned int pos;

	lpc_d = irq_data_get_irq_chip_data (data);
	pos = data->hwirq;

	lpc_d->cache[LPC_REG_IRQ_MASK] |= BIT(pos);
}


static int lpc_bridge_irq_set_type (struct irq_data *data, unsigned int type) {
	struct lpc_data *lpc_d;
	unsigned int pos = data->hwirq;
	int status = 0;

	lpc_d = irq_data_get_irq_chip_data (data);

	if ( type & IRQ_TYPE_EDGE_FALLING ) {
		lpc_d->cache[LPC_REG_IRQ_CONF] &= ~BIT(pos);
	} else if ( type & IRQ_TYPE_LEVEL_HIGH ) {
		lpc_d->cache[LPC_REG_IRQ_CONF] |= BIT(pos);
	} else
		status = -EINVAL;

	return status;

}


static void lpc_bridge_irq_bus_lock (struct irq_data *data) {
	struct lpc_data *lpc_d;

	lpc_d = irq_data_get_irq_chip_data (data);
	mutex_lock (&lpc_d->lock_irq);

}


static void lpc_bridge_irq_bus_unlock (struct irq_data *data) {
	struct lpc_data *lpc_d;

	lpc_d = irq_data_get_irq_chip_data (data);

	mutex_lock (&lpc_d->bus_lock);
	lpc_reg_write (LPC_REG_IRQ_MASK, lpc_d->cache[LPC_REG_IRQ_MASK]);
	lpc_reg_write (LPC_REG_IRQ_CONF, lpc_d->cache[LPC_REG_IRQ_CONF]);
	mutex_unlock (&lpc_d->bus_lock);

	mutex_unlock (&lpc_d->lock_irq);

}


static unsigned int lpc_bridge_irq_startup (struct irq_data *data) {
	struct lpc_data *lpc_d;

	lpc_d = irq_data_get_irq_chip_data (data);
	lpc_bridge_irq_unmask (data);

	return 0;
}


static void lpc_bridge_irq_shutdown (struct irq_data *data) {
	struct lpc_data *lpc_d;

	lpc_d = irq_data_get_irq_chip_data (data);
	lpc_bridge_irq_mask (data);

}


static struct irq_chip lpc_bridge_irq_chip = {
	.name                 = "lpc_bridge-seco",
	.irq_mask             = lpc_bridge_irq_mask,
	.irq_unmask           = lpc_bridge_irq_unmask,
	.irq_enable           = lpc_bridge_irq_enable,
	.irq_disable          = lpc_bridge_irq_disable,
	.irq_set_type         = lpc_bridge_irq_set_type,
	.irq_bus_lock         = lpc_bridge_irq_bus_lock,
	.irq_bus_sync_unlock  = lpc_bridge_irq_bus_unlock,
	.irq_startup          = lpc_bridge_irq_startup,
	.irq_shutdown         = lpc_bridge_irq_shutdown,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



/* __________________________________________________________________________
* |                                                                          |
* |                           SUB-DEVICE ALLOCATION                          |
* |__________________________________________________________________________|
*/
static int lpc_driver_allocate (struct device *parent_dev, struct device_node *dp) {
	int status;

	status = of_platform_populate (dp,  NULL, NULL, parent_dev);
	if ( status ) {
		LPC_ERR ("failed to create device");
	} else {
		LPC_INFO ("lpc device created");
	}

	return status;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                                BUS SETUP                                 |
* |__________________________________________________________________________|
*/
static int lpc_bridge_irq_setup (struct lpc_data *lpc_d) {
	int err, irq, i, done;

	lpc_d->irq_domain = irq_domain_add_linear(lpc_d->dev->of_node, NR_LPC_SLOTS,
								&irq_domain_simple_ops, lpc_d);
	if ( !lpc_d->irq_domain ) {
		LPC_ERR ("cannot assign irq domain");
		err = -ENODEV;
		goto err_irq_domain_add;
	}

	err = devm_request_threaded_irq (lpc_d->dev, lpc_d->irq, NULL, lpc_bridge_hanlder_irq,
										IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
										dev_name(lpc_d->dev), lpc_d);
	if (err != 0) {
		LPC_ERR ("unable to request IRQ#%d: %d", lpc_d->irq, err);
		goto err_request_thr_irq;
	}

	for ( i = 0 ; i < NR_LPC_SLOTS ; i++ ) {
		irq = irq_create_mapping (lpc_d->irq_domain, i);
		if ( irq < 0 ) {
			LPC_ERR ("cannot map irq");
			done = i;
			err = -EINVAL;
			goto err_map_irq;
		}
		irq_set_lockdep_class (irq, &lpc_lock_class);
		irq_set_chip_data (irq, lpc_d);
		irq_set_chip (irq, &lpc_bridge_irq_chip);
		irq_set_nested_thread (irq, true);
	}

	return 0;
err_map_irq:
	while ( --done ) {
		irq = irq_find_mapping (lpc_d->irq_domain, i);
		if ( irq > 0 )
			irq_dispose_mapping (irq);
	}
err_request_thr_irq:
	irq_domain_remove (lpc_d->irq_domain);
err_irq_domain_add:
	return err;

}


static int lpc_setup (struct device *dev, struct device_node *dp,
	   					struct lpc_data *lpc_d)
{
	int err, status;

	if (lpc_d == NULL) {
		err = -ENOMEM;
		goto err_lpc_alloc;
	}

	lpc_d->mem_base = cpld_get_membase ();

	lpc_d->name = dev_name (dev);

	lpc_d->dev = dev;

	mutex_init (&lpc_d->bus_lock);
	mutex_init (&lpc_d->lock_irq);

	// set register init

	status = lpc_bridge_irq_setup (lpc_d);
	if ( status ) {
		LPC_ERR ("cannot setup lpc irq");
		err = status;
		goto err_irq_setup;
	}

	return 0;
err_irq_setup:
err_lpc_alloc:
	return err;
}


static int lpc_bridge_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	int err = 0;
	int done, irq, ret;
	int i = 0;

	/* check that the CPLD firmware is a GPIO expander */
	if ( !cpld_is_lpc () ) {
		LPC_ERR ("the CPLD firmware is no a LPC Bridge");
		err = EINVAL;
		goto err_cpld;
	}

	lpc_d = kzalloc (sizeof(struct lpc_data), GFP_KERNEL);
	if ( !lpc_d ) {
		LPC_ERR ("cannot allocate memory for structure data");
		err = -ENOMEM;
		goto err_data_allocate;
	}

	platform_set_drvdata (pdev, lpc_d);

	lpc_d->irq = irq_of_parse_and_map (dp, 0);

	ret = lpc_setup (&pdev->dev, dp, lpc_d);
	if ( ret != 0 ) {
		err = ret;
		LPC_ERR ("LPC initialization failed. Probe stopped!");
		goto err_lpc_setup;
	}

	ret = lpc_driver_allocate (&pdev->dev, dp);
	if ( ret != 0 ) {
		err = -EINVAL;
		LPC_ERR ("cannot allocate sub device!");
		goto err_driver_allocate;
	}

	LPC_INFO ("lpc_bridge driver ver %s probed!!!", DRV_VERSION);
	return 0;
err_driver_allocate:
	free_irq (lpc_d->irq, lpc_d);
	done = NR_LPC_SLOTS;
	while ( --done ) {
		irq = irq_find_mapping (lpc_d->irq_domain, i);
		if ( irq > 0 )
			irq_dispose_mapping (irq);
	}
	irq_domain_remove (lpc_d->irq_domain);
err_lpc_setup:
	kfree (lpc_d);
err_data_allocate:
	dp = NULL;
err_cpld:
	return err;
}


static int lpc_bridge_remove (struct platform_device *pdev) {
	return 0;
}


static const struct of_device_id seco_lpc_bridge_match[] = {
	{ .compatible = "seco,lpc_bridge" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_lpc_bridge_match);


static struct platform_driver lpc_bridge_driver = {
	.driver = {
		.name		    = "lpc_bridge",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_lpc_bridge_match,
	},
	.probe  = lpc_bridge_probe,
	.remove = lpc_bridge_remove,
};



static int __init lpc_bridge_init(void) {
	return platform_driver_register (&lpc_bridge_driver);
}

subsys_initcall(lpc_bridge_init);


static void __exit lpc_bridge_exit (void) {
	return platform_driver_unregister (&lpc_bridge_driver);
}

module_exit(lpc_bridge_exit);



MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("SECO LPC Bridge over CPLD logic");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
