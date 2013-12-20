#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/kfifo.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>

#include "hpsdr.h"

#define TX_DEVICE_NAME "tx"
#define CLASS_NAME "hpsdr"

#define LWH2FBRG_BASE	0xFF200000
#define H2FBRG_BASE	0xC0000000
#define LED_BASE	0x00010040
#define LED_SIZE	0x20
#define BUTTON_BASE	0x000100c0
#define BUTTON_SIZE	0x0F
#define BUTTON_IRQ	73
#define FIFO_SIZE	24192

#define HPSDR_NR_RX	7

#define err(format, arg...) pr_err(CLASS_NAME ": " format, ## arg)
#define info(format, arg...) pr_info(CLASS_NAME ": " format, ## arg)
#define warn(format, arg...) pr_warn(CLASS_NAME ": " format, ## arg)

int hpsdr_nr_rx = HPSDR_NR_RX;

module_param(hpsdr_nr_rx, int, S_IRUGO);

static struct class *hpsdr_class = NULL;
static dev_t first_dev;
static uint32_t *capture_registers;

//  These should be in a filp eventually
struct hpsdr_dev {
	uint8_t *led_registers;
	uint32_t *memory_registers;
	struct kfifo read_fifo;
	struct cdev cdev;
};

static struct hpsdr_dev *hpsdr_devices;

static irqreturn_t hpsdr_irq_handler(int irq, void *dev_id) {
	info("Received hpsdr interrupt\n");

	//  Need to clear the caputre register to reset the interrupt
	iowrite32(0x00000000, capture_registers);
	return IRQ_HANDLED;
}

static int hpsdr_tx_device_open(struct inode *inode, struct file *filp) {
	int i;
	struct hpsdr_dev *dev;

	//  Need to read from the device for testing
	/* if(((filp->f_flags & O_ACCMODE) == O_RDONLY) || 
           ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		return -EACCES;
	} */

	//  Need to check if we're already open!

	dev = container_of(inode->i_cdev, struct hpsdr_dev, cdev);
	filp->private_data = dev;

	dev->led_registers = (uint8_t *) ioremap(LWH2FBRG_BASE + LED_BASE, sizeof(uint8_t));
	dev->memory_registers = (uint32_t *) ioremap(H2FBRG_BASE, sizeof(uint32_t) * 64);

	/* info("Test reading FPS2FPGA bridge region\n");
	for(i = 0; i < 64; ++i) {
		pr_info("%d: %8.8X\n", i, ioread32(dev->memory_registers + (i * sizeof(uint32_t))));
	} */

	return 0;
}

static int hpsdr_tx_device_close(struct inode *inode, struct file *filp) {
	struct hpsdr_dev *dev = filp->private_data;

	kfifo_free(&dev->read_fifo);

	iounmap(dev->led_registers);
	iounmap(dev->memory_registers);
	return 0;
}

static ssize_t hpsdr_tx_device_write(struct file *filp, const char __user *buffer, size_t length, loff_t *offset) {
	//int retval;
	struct hpsdr_dev *dev = filp->private_data;

	return length;
}

static long hpsdr_tx_device_ioctl(struct file *filp, unsigned cmd, unsigned long arg) {
	struct hpsdr_dev *dev = filp->private_data;

	switch(cmd) {
		case HPSDR_IOCTPREAMP:
			iowrite8((arg & 0x01), dev->led_registers);
			break;
		case HPSDR_IOCQPREAMP:
			return ioread8(dev->led_registers) & 0x01;
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}


static struct file_operations tx_fops = {
	.write = hpsdr_tx_device_write,
	.open = hpsdr_tx_device_open,
	.release = hpsdr_tx_device_close,
	.unlocked_ioctl = hpsdr_tx_device_ioctl
};

static struct file_operations rx_fops = {
	.write = hpsdr_tx_device_write,
	.open = hpsdr_tx_device_open,
	.release = hpsdr_tx_device_close,
	.unlocked_ioctl = hpsdr_tx_device_ioctl
};

static int hpsdr_create_rxdev(int index, struct hpsdr_dev *dev) {
	//  ALlocate a fifo for the device data
	if(kfifo_alloc(&dev->read_fifo, FIFO_SIZE, GFP_KERNEL)) {
		err("Couldn't allocate a read FIFO\n");
		return -ENOMEM;
	}

	if(device_create(hpsdr_class, NULL, first_dev + index, NULL, "hpsdrrx%d", index) == NULL) {
		return -1;
	}
	cdev_init(&dev->cdev, &rx_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &rx_fops;
	if(cdev_add(&dev->cdev, first_dev + index, 1)) {
		device_destroy(hpsdr_class, first_dev + index);
		return -1;
	}

	return 0;
}

static void hpsdr_cleanup_module(void) {
	int i;

	if(hpsdr_devices) {
		for(i = 0; i < hpsdr_nr_rx; ++i) {
			cdev_del(&hpsdr_devices[i].cdev);
			device_destroy(hpsdr_class, first_dev + i);
		}
		kfree(hpsdr_devices);
	}

	class_unregister(hpsdr_class);
	class_destroy(hpsdr_class);

	unregister_chrdev_region(first_dev, hpsdr_nr_rx);
}

static int __init hpsdr_init(void) {
	int retval = -1;
	int i;
	uint32_t *button_registers;

	if(alloc_chrdev_region(&first_dev, 0, hpsdr_nr_rx, "hpsdr") < 0) {
		err("failed to register device region\n");
		retval = -1;
		goto fail;
	}

	hpsdr_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(hpsdr_class)) {
		err("failed to register device class '%s'\n", CLASS_NAME);
		retval = PTR_ERR(hpsdr_class);
		goto fail;
	}

	//  Allocate the receiver devices
	hpsdr_devices = kmalloc(hpsdr_nr_rx * sizeof(struct hpsdr_dev), GFP_KERNEL);
	if(!hpsdr_devices) {
		retval = -ENOMEM;
		goto fail;
	}
	memset(hpsdr_devices, 0, hpsdr_nr_rx * sizeof(struct hpsdr_dev));

	for(i = 0; i < hpsdr_nr_rx; ++i) {
		hpsdr_create_rxdev(i, &hpsdr_devices[i]);
	}

	// Allocate memory region for the registers in the lightweight bridge
	// This is for parameter changes accomplished through ioctls
	if(request_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE, "hpsdr-control") == NULL) {
		err("failed to reserve memory\n");
		goto fail;
	}

	//  Set up an interrupt handler
	if(request_irq(BUTTON_IRQ, (irq_handler_t) hpsdr_irq_handler, 0, "hpsdr", NULL)) {
		err("Couldn't assign interrupt: %d\n", BUTTON_IRQ);
		//  This isn't the right error
		return -ENOMEM;
	}

	//  Remap memory and activate intrrupts
	button_registers = (uint32_t *) ioremap(LWH2FBRG_BASE + BUTTON_BASE + 8, sizeof(uint32_t));
	iowrite32(0xFFFFFFFF, button_registers);
	iounmap(button_registers);
	capture_registers = (uint32_t *) ioremap(LWH2FBRG_BASE + BUTTON_BASE + 12, sizeof(uint32_t));
	iowrite32(0x00000000, capture_registers);

	return 0;

	fail:
		hpsdr_cleanup_module();
		return retval;
}

static void __exit hpsdr_exit(void) {
	uint32_t *button_registers;

	//  Disable interrupts
	button_registers = (uint32_t *) ioremap(LWH2FBRG_BASE + BUTTON_BASE + 8, sizeof(uint32_t));
	iowrite32(0x00000000, button_registers);
	iounmap(button_registers);
	iounmap(capture_registers);
	free_irq(BUTTON_IRQ, NULL);

	release_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE);
	hpsdr_cleanup_module();
}

module_init(hpsdr_init);
module_exit(hpsdr_exit);

MODULE_AUTHOR("Jeremy McDermond <nh6z@nh6z.net>");
MODULE_DESCRIPTION("OpenHPSDR SoC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
