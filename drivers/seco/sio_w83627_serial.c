#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/sio-w83627.h>
#include <linux/seco_cpld.h>
#include <linux/seco_lpc.h>

#include <linux/sio_lpc_serial.h>



#define SIO_W83627_REG_DEV_PWR        0x22
#define SIO_W83627_REG_UART_TRI       0x25
#define SIO_W83627_REG_MUX_A          0x29
#define SIO_W83627_REG_MUX_B          0x2C

#define SIO_W83627_PWR_UART_A         0x10
#define SIO_W83627_PWR_UART_B         0x20
#define SIO_W83627_TRI_UART_A         0x10
#define SIO_W83627_TRI_UART_B         0x20
#define SIO_W83627_MUX_A              0x08
#define SIO_W83627_MUX_B              0x03

#define SIO_W83627_REG_UART_PWR       0x30
#define SIO_W83627_REG_UART_ADDR_H    0x60
#define SIO_W83627_REG_UART_ADDR_L    0x61
#define SIO_W83627_REG_UART_IRQ_SRC   0x70
#define SIO_W83627_REG_UART_IRQ_CTRL  0xF0
#define SIO_W83627_REG_UART_IRQ_IR    0xF1

#define SIO_W83627_UART_ACTIVE        0x01
#define SIO_W83627_UART_MAX_CLK       24000000   // 24 MHz

#define sio_uart_read(sio_uart, addr, data)    \
	(sio_uart)->sio_ops->read (addr, data, (sio_uart)->uart_ldev)

#define sio_uart_write(sio_uart, addr, data)    \
	(sio_uart)->sio_ops->write (addr, data, (sio_uart)->uart_ldev)

#define sio_conf_reg(sio_uart, reg)            \
	(sio_uart)->conf_reg_map[(reg)]




/* __________________________________________________________________________
* |                                                                          |
* |                            W83627DHG UART INIT                           |
* |__________________________________________________________________________|
*/
static void w83627_uart_A_init (struct lpc_serial_port *info) {
	uint8_t val = 0;
	uint8_t clk_set = 0;

	/*  Global Register  */

	info->sio_ops->g_read (SIO_W83627_REG_DEV_PWR, &val);
	val |= SIO_W83627_PWR_UART_A;
	info->sio_ops->g_write (SIO_W83627_REG_DEV_PWR, val);

	info->sio_ops->g_read (SIO_W83627_REG_UART_TRI, &val);
	val &= ~SIO_W83627_TRI_UART_A;
	info->sio_ops->g_write (SIO_W83627_REG_UART_TRI, val);

	info->sio_ops->g_read (SIO_W83627_REG_MUX_A, &val);
	val &= ~SIO_W83627_MUX_A;
	info->sio_ops->g_write (SIO_W83627_REG_MUX_A, val);

	/*  UART Register  */
	sio_uart_write (info, SIO_W83627_REG_UART_ADDR_H, (info->membase & 0xFF00) >> 8);
	sio_uart_write (info, SIO_W83627_REG_UART_ADDR_L, info->membase & 0x00FF);
	switch (info->uart_clk / SIO_W83627_UART_MAX_CLK) {
		case 1:
			clk_set = 0x02;
			break;
		case 12:
			clk_set = 0x01;
			break;
		case 13:
			clk_set = 0x00;
			break;
		default:
			clk_set = 0x02;
	}
	sio_uart_write (info, SIO_W83627_REG_UART_IRQ_CTRL, clk_set);
	sio_uart_write (info, SIO_W83627_REG_UART_PWR, SIO_W83627_UART_ACTIVE);
}


static void w83627_uart_B_init (struct lpc_serial_port *info) {
	uint8_t val = 0;
	uint8_t clk_set = 0;

	info->sio_ops->g_read (SIO_W83627_REG_DEV_PWR, &val);
	val |= SIO_W83627_PWR_UART_B;
	info->sio_ops->g_write (SIO_W83627_REG_DEV_PWR, val);

	info->sio_ops->g_read (SIO_W83627_REG_UART_TRI, &val);
	val &= ~SIO_W83627_TRI_UART_B;
	info->sio_ops->g_write (SIO_W83627_REG_UART_TRI, val);

	info->sio_ops->g_read (SIO_W83627_REG_MUX_B, &val);
	val |= SIO_W83627_MUX_B;
	info->sio_ops->g_write (SIO_W83627_REG_MUX_B, val);

	/*  UART Register  */
	sio_uart_write (info, SIO_W83627_REG_UART_ADDR_H, (info->membase & 0xFF00) >> 8);
	sio_uart_write (info, SIO_W83627_REG_UART_ADDR_L, info->membase & 0x00FF);
	switch (info->uart_clk / SIO_W83627_UART_MAX_CLK) {
		case 1:
			clk_set = 0x02;
			break;
		case 12:
			clk_set = 0x01;
			break;
		case 13:
			clk_set = 0x00;
			break;
		default:
			clk_set = 0x02;
	}
	sio_uart_write (info, SIO_W83627_REG_UART_IRQ_CTRL, clk_set);
	sio_uart_write (info, SIO_W83627_REG_UART_PWR, SIO_W83627_UART_ACTIVE);
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                      W83627DHG UART COMMUNICATION                        |
* |__________________________________________________________________________|
*/
unsigned int w83627_serial_in (struct uart_port *port, int offset) {
	uint16_t data;
	uint16_t addr = (uint16_t)(port->mapbase ) & 0xFFFF;
	lpc_readw (addr + offset, &data);
	return (int)data;
}


void w83627_serial_out (struct uart_port *port, int offset, int value) {
	uint16_t addr = (uint16_t)(port->mapbase ) & 0xFFFF;
	lpc_writew (addr + offset, value);
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                              SYSFS INTERFACE                             |
* |__________________________________________________________________________|
*/
static ssize_t sys_w83627_dump_uart_regs (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	struct lpc_serial_port *info = (struct lpc_serial_port *)dev_get_drvdata (dev);
	unsigned char msg[500];
	unsigned char tmp[50];
	uint8_t value;
	int i;

	sprintf (msg, "\n#\tvalue");
	for ( i = 0 ; i <= 7 ; i++) {
		value = (uint16_t)w83627_serial_in (&info->port, i);
		sprintf (tmp, "\n0x%02X\t0x%02X", i, value);
		strcat (msg, tmp);
	}

	return sprintf (buf, "%s\n", msg);
}


static ssize_t sys_w83627_store_uart_regs (struct device *dev, struct device_attribute *attr,
									const char *buf, size_t count)
{
	struct lpc_serial_port *info = (struct lpc_serial_port *)dev_get_drvdata (dev);
	uint8_t reg, value;
	unsigned long val;
	char *start = (char *)buf;

	while (*start == ' ')
		start++;

	reg = (uint8_t)simple_strtoul (start, &start, 16);

	printk (KERN_INFO "davide reg %X\n", reg);
	if ( reg < 0 || reg > 7 ) {
		return -EINVAL;
	}

	while (*start == ' ')
		start++;

	if ( kstrtoul (start, 16, &val) ) {
		return -EINVAL;
	}

	value = (uint8_t)val;

	printk (KERN_INFO "davide scrivo %X in %X\n", value, reg);
	w83627_serial_out (&info->port, reg, value);

	return count;
}


static DEVICE_ATTR(dump_uart_regs, 0664, sys_w83627_dump_uart_regs, sys_w83627_store_uart_regs);


static ssize_t sys_w83627_payload_size_show (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	struct lpc_serial_port *info = (struct lpc_serial_port *)dev_get_drvdata (dev);

	return sprintf (buf, "%d\n", info->payload_size);
}


static DEVICE_ATTR(payload_size, 0444, sys_w83627_payload_size_show, NULL);


static struct attribute *w83627_uart_attrs[] = {
	&dev_attr_dump_uart_regs.attr,
	&dev_attr_payload_size.attr,
	NULL,
};


static struct attribute_group w83627_uart_attr_group = {
	.attrs = w83627_uart_attrs,
};


int lpc_w83627_sys_serial_setup (struct platform_device *pdev, struct lpc_serial_port *info) {
	int err = 0;
	switch ( info->uart_index ) {
		case LPC_UART_TYPE_W83627_A:
		case LPC_UART_TYPE_W83627_B:
			err = sysfs_create_group (&pdev->dev.kobj, &w83627_uart_attr_group);
			break;
		default:
			break;
	}
	return err;
}

/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


int lpc_w83627_platform_serial_setup (struct platform_device *pdev,
		//	int type, struct uart_port *port,
			struct lpc_serial_port *info)
{
	int ret = 0;

	switch ( info->uart_index ) {
		case LPC_UART_TYPE_W83627_A:
			w83627_uart_A_init (info);
			break;
		case LPC_UART_TYPE_W83627_B:
			w83627_uart_B_init (info);
			break;
		default:
			ret = -1;
	}

	if ( !ret) {
		switch ( info->uart_index ) {
			case LPC_UART_TYPE_W83627_A:
			case LPC_UART_TYPE_W83627_B:
				info->serial_in = w83627_serial_in;
				info->serial_out = w83627_serial_out;
				info->type = LPC_PORT_TYPE_W83627;
				break;
			default:
				break;
		}
	}

	return ret;
}

