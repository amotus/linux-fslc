#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>


#define DRV_VERSION     "1.0"


struct board_data {
	unsigned int   num_gpio;
	int            code;
	int            *gpios;
};

#define GET_SIZE(x)    (sizeof(struct board_data) + (sizeof(int)) * (x))


static struct proc_dir_entry *board_id_code_proc = NULL;


static ssize_t board_id_code_read_proc (struct seq_file *seq, void *offset) {
	struct board_data *bid = (struct board_data *)seq->private;

	if ( !bid )
		return -EINVAL;

	seq_printf(seq, "0x%X\n", bid->code);
	return 0;
}


static int board_id_code_open_proc (struct inode *inode, struct file *file) {
        return single_open(file, board_id_code_read_proc,  PDE_DATA(inode));
}


static const struct file_operations code_proc_ops = {
    .owner = THIS_MODULE,
	.open  = board_id_code_open_proc,
    .read  = seq_read,
    .write = NULL,
};


static int code_setup (struct device *dev, struct device_node *np, struct board_data **board_id) {
	int ret;
	unsigned int num;
	int err = 0;
	ssize_t size = 0;
	struct board_data *bid;

	ret = of_property_read_u32 (np, "gpio-num", &num);

	if ( num <= 0 )
		return -EINVAL;

	size = GET_SIZE(num);

	*board_id = kzalloc (size, GFP_KERNEL);
	if ( !board_id )
		return -ENOMEM;

	bid = *board_id;
	bid->num_gpio = num;

	bid->gpios = kzalloc (sizeof(int) * bid->num_gpio, GFP_KERNEL);
	if ( !bid->gpios ) {
		err = -ENOMEM;
		dev_err (dev, "memory error!!!\n");
		goto err_gpio_mem;
	}
	for ( num = 0 ; num < bid->num_gpio ; num++ ) {
		bid->gpios[num] = of_get_named_gpio (np, "code-gpios", num);
		if ( bid->gpios[num] < 0 )
			break;
	}

	if ( num != bid->num_gpio ) {
		err = -EINVAL;
		dev_err (dev, "Failed to obtain gpios\n");
		goto err_gpio;
	}


	bid->code = 0;
	for ( num = 0 ; num < bid->num_gpio ; num++ ) {
		gpio_direction_input (bid->gpios[num]);
		bid->code |= gpio_get_value (bid->gpios[num]) << num;
	}

	return 0;

err_gpio:
err_gpio_mem:

	return err;
}


static int board_id_probe (struct platform_device *pdev) {

	struct device_node *dp = pdev->dev.of_node;
	struct board_data  *board_id = NULL;
	int                ret = 0;

	ret = code_setup (&pdev->dev, dp, &board_id);
	if ( ret == 0 ) {
		dev_info (&pdev->dev, "used %d gpios\n", board_id->num_gpio);
	} else {
		return ret;
	}
	dev_set_drvdata (&pdev->dev, board_id);

	board_id_code_proc = proc_create_data ("bid", 0444, NULL, &code_proc_ops, board_id);
	if ( board_id_code_proc == NULL ) {
		dev_err (&pdev->dev, "create_proc_entry bid failed\n");
	}

	return 0;
}


static int board_id_remove (struct platform_device *pdev) {
	return 0;
}


static const struct of_device_id seco_board_id_match[] = {
	{ .compatible = "seco,board_id" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, seco_board_id_match);


static struct platform_driver board_id_driver = {
	.driver = {
		.name		    = "board_id",
		.owner          = THIS_MODULE,
		.of_match_table	= seco_board_id_match,
	},
	.probe  = board_id_probe,
	.remove = board_id_remove,
};



static int __init board_id_init(void) {
	return platform_driver_register (&board_id_driver);
}

subsys_initcall(board_id_init);


static void __exit board_id_exit (void) {
	return platform_driver_unregister (&board_id_driver);
}

module_exit(board_id_exit);



MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("SECO Custom board identification");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
