

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
#include <linux/seco_lpc.h>
#include <linux/sio-w83627.h>


#define SIO_INFO(fmt, arg...) printk(KERN_INFO "SIO W83627: " fmt "\n" , ## arg)
#define SIO_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define SIO_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define DRV_VERSION     "1.0"


#define W83627DHG_UARTA       (uint8_t)0x02 	/* UART A */
#define W83627DHG_UARTB       (uint8_t)0x03 	/* UART B IR */
#define W83627EHF_LD_HWM      (uint8_t)0x0b
#define W83667HG_LD_VID       (uint8_t)0x0d    /* Logical device = ?? */

#define SIO_REG_LDSEL         (uint8_t)0x07	/* Logical device select */
#define SIO_REG_DEVID         (uint8_t)0x20	/* Device ID (2 bytes) */
#define SIO_REG_GLOBALOPT     (uint8_t)0x24	/* Global Options */
#define SIO_REG_EN_VRM10      (uint8_t)0x2C	/* GPIO3, GPIO4 selection */
#define SIO_REG_ENABLE        (uint8_t)0x30	/* Logical device enable */
#define SIO_REG_ADDR          (uint8_t)0x60	/* Logical device address (2 bytes) */
#define SIO_REG_IRQ           (uint8_t)0x70	/* Logical device irq*/
#define SIO_REG_VID_CTRL      (uint8_t)0xF0	/* VID control */
#define SIO_REG_VID_DATA      (uint8_t)0xF1	/* VID data */

#define CFG_REG_UART_A_CLK    0xF0 	/* CFG REGISTER OF THE UART A CLOCK */

#define CFG_REG_UART_B_CLK    0xF0 	/* CFG REGISTER OF THE UART B CLOCK */

#define WDT_CLOCK_1846200     (0x00)  /* 1.8462MHz clock source (24MHz/13) */
#define WDT_CLOCK_2000000     (0x01)  /* 2MHz clock source (24MHz/12) */
#define WDT_CLOCK_2400000     (0x02)  /* 24MHz clock source (24MHz/1) */

#define SIO_W83627EHF_ID      0x8850
#define SIO_W83627EHG_ID      0x8860
#define SIO_W83627DHG_ID      0xa020
#define SIO_W83627DHG_P_ID    0xb070
#define SIO_W83667HG_ID       0xa510
#define SIO_ID_MASK           0xFFF0

#define CFG_REG_IDX           (0x2E << 1)
#define CFG_REG_DATA          (0x2F << 1)


#define SLOT_UART_A           4
#define SLOT_UART_B           3

#define BASE_CHIP_ID          0xB070

#define CR_SOFT_RESET         0x02
#define CR_LOGICAL_DEV        0x07
#define CR_CHIP_ID_LO         0x21
#define CR_CHIP_ID_HI         0x20
#define CR_DEV_PWR_DOWN       0x22
#define CR_IPD                0x23
#define CR_GLOBAL_OPT1        0x24
#define CR_INTERFACE_TRI_EN   0x25
#define CR_GLOBAL_OPT2        0x26


#define REG_EFER              0x2E    // Extended Function Enable/Index Register
#define REG_EFDR              0x2F    // Extended Function Data Register


#define ENTER_EXT_MODE_CODE   0x87
#define EXIT_EXT_MODE_CODE    0xAA
#define SW_RESET_CODE         0x01


#define WINBOND_UART_CLOCK    24000000

#define SUPERIO_UART3_BASE    0x03F8
#define SUPERIO_UART4_BASE    0x02F8



static uint8_t reg_global_addr[] = {
	0x02, 0x07, 0x20, 0x21, 0x22,
	0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2A, 0x2B, 0x2C,
   	0x2D, 0x2E, 0x2F
};


static uint8_t reg_ldev_2_addr[] = {
	0x30, 0x60, 0x61, 0x70, 0xF0
};


static uint8_t reg_ldev_3_addr[] = {
	0x30, 0x60, 0x61, 0x70, 0xF0, 0xF1
};


static uint8_t reg_ldev_9_addr[] = {
	0x30, 0XE0, 0XE1, 0XE2, 0XE3,
	0XE4, 0XE5, 0XE6, 0XE7, 0XE8,
	0XE9, 0XF0, 0XF1, 0XF2, 0XF3,
	0XF4, 0XF5, 0XF6, 0XF7, 0XF8,
	0XF9, 0XFA, 0XFE
};


/* __________________________________________________________________________
* |                                                                          |
* |                              BASIC FUNCTIONS                             |
* |__________________________________________________________________________|
*/

static inline void superio_outb (uint8_t addr, uint8_t value) {
	uint16_t val16 = (uint16_t)value;
	uint16_t addr16 = (uint16_t)addr;
	lpc_writew (REG_EFER, addr16);
	lpc_writew (REG_EFDR, val16);
}


static inline void superio_inb (uint16_t addr, uint8_t *data) {
	uint16_t val16 = 0;
	uint16_t addr16 = (uint16_t)addr;
	lpc_writew (REG_EFER, addr16);
	lpc_readw (REG_EFDR, &val16);
	*data = (uint16_t)(val16);
}


static inline void superio_enter_ext_mode (void) {
	lpc_writew (REG_EFER, ENTER_EXT_MODE_CODE);
	lpc_writew (REG_EFER, ENTER_EXT_MODE_CODE);
}


static inline void superio_exit_ext_mode (void) {
	lpc_writew (REG_EFER, EXIT_EXT_MODE_CODE);
}


static inline void superio_sw_reset (void) {
	lpc_writew (REG_EFER, CR_SOFT_RESET);
	lpc_writew (REG_EFDR, SW_RESET_CODE);
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/
static uint16_t getChipID (void) {
	uint8_t r_h = 0x00;
	uint8_t r_l = 0x00;

	udelay (10);
	superio_inb (CR_CHIP_ID_HI, &r_h);
	superio_inb (CR_CHIP_ID_LO, &r_l);

	return ((uint16_t)r_l) | (((uint16_t)r_h) << 8);
}


static int chipIDvalidate (uint16_t id) {
	return (id & 0xFFF0) == BASE_CHIP_ID ? 1 : 0;
}


static inline void superio_select (uint8_t logical_dev_id) {
	superio_outb (CR_LOGICAL_DEV, logical_dev_id);
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


static void superio_gpio_read (uint8_t addr, uint8_t *data, uint8_t ldev_id) {
	superio_enter_ext_mode ();
	superio_select (ldev_id);
	superio_inb (addr, data);
	superio_exit_ext_mode ();
}


static void superio_gpio_write (uint8_t addr, uint8_t value, uint8_t ldev_id) {
	superio_enter_ext_mode ();
	superio_select (ldev_id);
	superio_outb (addr, value);
	superio_exit_ext_mode ();
}


static void superio_uart_read (uint8_t addr, uint8_t *data, uint8_t ldev_id) {
	superio_enter_ext_mode ();
	superio_select (ldev_id);
	superio_inb (addr, data);
	superio_exit_ext_mode ();
}


static void superio_uart_write (uint8_t addr, uint8_t value, uint8_t ldev_id) {
	superio_enter_ext_mode ();
	superio_select (ldev_id);
	superio_outb (addr, value);
	superio_exit_ext_mode ();
}


static void superio_global_read (uint8_t addr, uint8_t *data) {
	superio_enter_ext_mode ();
	superio_inb (addr, data);
	superio_exit_ext_mode ();
}


static void superio_global_write (uint8_t addr, uint8_t value) {
	superio_enter_ext_mode ();
	superio_outb (addr, value);
	superio_exit_ext_mode ();
}


static struct sio_w83627_ops gpio_ops = {
	.read    = superio_gpio_read,
	.write   = superio_gpio_write,
	.g_read  = superio_global_read,
	.g_write = superio_global_write
};


static struct sio_w83627_ops uart_ops = {
	.read    = superio_uart_read,
	.write   = superio_uart_write,
	.g_read  = superio_global_read,
	.g_write = superio_global_write
};


static struct of_dev_auxdata sio_auxdata_lookup[] = {
	OF_DEV_AUXDATA("nuvoton,w83627dhg_gpio", 0, "sio_gpio_w83627", &gpio_ops),
	OF_DEV_AUXDATA("nuvoton,w83627dhg_A_16550a", 0, "sio_uart_a_w83627", &uart_ops),
	OF_DEV_AUXDATA("nuvoton,w83627dhg_B_16550a", 0, "sio_uart_b_w83627", &uart_ops),
	{ /*  sentinel */ },
};


static int superio_wb_driver_allocate (struct device *parent_dev, struct device_node *dp) {
	int status;

	status = of_platform_populate (dp,  NULL, sio_auxdata_lookup, parent_dev);
	if ( status ) {
		SIO_ERR ("failed to create device");
	} else {
		SIO_INFO ("superio device created");
	}

	return status;
}


/* __________________________________________________________________________
* |                                                                          |
* |                              SYSFS INTERFACE                             |
* |__________________________________________________________________________|
*/
static ssize_t sys_superio_wb_revision (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	uint16_t ver;

	superio_enter_ext_mode ();
	ver = getChipID ();
	superio_exit_ext_mode ();

	return sprintf(buf, "0x%04X\n",ver);
}

static DEVICE_ATTR(superio_ver, S_IRUGO, sys_superio_wb_revision, NULL);



static ssize_t sys_dump_regs (const uint8_t *reg_array, int a_size, char *buf, uint8_t ldev) {
	unsigned char msg[500];
	unsigned char tmp[50];
	uint8_t value;
	int i;

	superio_enter_ext_mode ();
	if ( ldev < 0xD ) {
		superio_select (ldev);
	}
	sprintf (msg, "\n#\tvalue");
	for ( i = 0 ; i < a_size ; i++ ) {
		superio_inb (reg_array[i], &value);
		sprintf (tmp, "\n0x%02X\t0x%02X", reg_array[i], value);
		strcat (msg, tmp);
	}
	superio_exit_ext_mode ();

	return sprintf (buf, "%s\n", msg);
}


static ssize_t sys_superio_dump_global_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	return sys_dump_regs (reg_global_addr, ARRAY_SIZE (reg_global_addr), buf, (uint8_t)-1);
}

static DEVICE_ATTR(dump_global_regs, S_IRUGO, sys_superio_dump_global_regs, NULL);


static ssize_t sys_superio_dump_log_dev_2_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	return sys_dump_regs (reg_ldev_2_addr, ARRAY_SIZE (reg_ldev_2_addr), buf, (uint8_t)2);
}

static DEVICE_ATTR(dump_ldev2_regs, S_IRUGO, sys_superio_dump_log_dev_2_regs, NULL);


static ssize_t sys_superio_dump_log_dev_3_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	return sys_dump_regs (reg_ldev_3_addr, ARRAY_SIZE (reg_ldev_3_addr), buf, (uint8_t)3);
}

static DEVICE_ATTR(dump_ldev3_regs, S_IRUGO, sys_superio_dump_log_dev_3_regs, NULL);


static ssize_t sys_superio_dump_log_dev_9_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	return sys_dump_regs (reg_ldev_9_addr, ARRAY_SIZE (reg_ldev_9_addr), buf, (uint8_t)9);
}

static DEVICE_ATTR(dump_ldev9_regs, S_IRUGO, sys_superio_dump_log_dev_9_regs, NULL);


static struct attribute *superio_wb_attrs[] = {
	&dev_attr_superio_ver.attr,
	&dev_attr_dump_global_regs.attr,
	&dev_attr_dump_ldev2_regs.attr,
	&dev_attr_dump_ldev3_regs.attr,
	&dev_attr_dump_ldev9_regs.attr,
	NULL,
};


static struct attribute_group superio_wb_attr_group = {
	.attrs = superio_wb_attrs,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/
static int superio_wb_probe (struct platform_device *pdev) {
	struct device_node *dp = pdev->dev.of_node;
	int err = 0;
	uint16_t rev = 0;

	superio_sw_reset ();
	udelay (10);

	superio_enter_ext_mode ();

	rev = getChipID ();
	if  (chipIDvalidate (rev) )
		SIO_INFO ("Winbond version: 0x%04X", rev);
	else {
		SIO_ERR ("invalid Winbond version: 0x%04X", rev);
		superio_exit_ext_mode ();
		err = -EINVAL;
		goto err_rev_inval;
	}

	superio_exit_ext_mode ();


	err = sysfs_create_group (&pdev->dev.kobj, &superio_wb_attr_group);
	if ( err ) {
		SIO_ERR ("cannot create sysfs interface");
		goto err_sysfs;
	}

	err = superio_wb_driver_allocate (&pdev->dev, dp);
	if ( err ) {
		SIO_ERR ("cannot allocat superIO sub-devices: %#X", err);
		goto err_device_allocate;
	}


	SIO_INFO ("LPC SuperIO driver ver %s probed!!!", DRV_VERSION);
	return 0;
err_device_allocate:
	sysfs_remove_group(&pdev->dev.kobj, &superio_wb_attr_group);
err_sysfs:
err_rev_inval:
	return err;
}


static int superio_wb_remove (struct platform_device *pdev) {
	return 0;
}


static const struct of_device_id seco_superio_wb_match[] = {
	{ .compatible = "nuvoton,w83627dhg" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_superio_wb_match);


static struct platform_driver superio_wb_driver = {
	.driver = {
		.name		    = "sio_w83627",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_superio_wb_match,
	},
	.probe  = superio_wb_probe,
	.remove = superio_wb_remove,
};



static int __init superio_wb_init(void) {
	return platform_driver_register (&superio_wb_driver);
}

subsys_initcall(superio_wb_init);


static void __exit superio_wb_exit (void) {
	return platform_driver_unregister (&superio_wb_driver);
}

module_exit(superio_wb_exit);



MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("Nuvoton LPC SuperIO - SECO version");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
