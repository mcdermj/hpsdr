#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include "hpsdr.h"

#define TX_DEVICE_NAME "tx"
#define CLASS_NAME "hpsdr"

#define LWH2FBRG_BASE	0xFF200000
#define LED_BASE	0x00010040
#define LED_SIZE	0x20

#define err(format, arg...) pr_err(CLASS_NAME ": " format, ## arg)
#define info(format, arg...) pr_info(CLASS_NAME ": " format, ## arg)
#define warn(format, arg...) pr_warn(CLASS_NAME ": " format, ## arg)

static struct class *hpsdr_class = NULL;
static struct device *hpsdr_device = NULL;
static int hpsdr_major;
static uint8_t *led_registers;

static int hpsdr_tx_device_open(struct inode *inode, struct file *filp) {
	if(((filp->f_flags & O_ACCMODE) == O_RDONLY) || 
           ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		return -EACCES;
	}

	//  Need to check if we're already open!

	//  These registers should be in the filp priv structure
	led_registers = (uint8_t *) ioremap(LWH2FBRG_BASE + LED_BASE, sizeof(uint8_t));
	return 0;
}

static int hpsdr_tx_device_close(struct inode *inode, struct file *filp) {
	iounmap(led_registers);
	return 0;
}

static ssize_t hpsdr_tx_device_write(struct file *filp, const char __user *buffer, size_t length, loff_t *offset) {
	//int retval;

	return length;
}

static long hpsdr_tx_device_ioctl(struct file *filp, unsigned cmd, unsigned long arg) {
	switch(cmd) {
		case HPSDR_IOCTPREAMP:
			iowrite8((arg & 0x01), led_registers);
			break;
		case HPSDR_IOCQPREAMP:
			return ioread8(led_registers) & 0x01;
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

static int __init hpsdr_init(void) {
	int retval;

	hpsdr_major = register_chrdev(0, TX_DEVICE_NAME, &tx_fops);
	if(hpsdr_major < 0) {
		err("failed to register device: error %d\n", hpsdr_major);
		retval = hpsdr_major;
		goto failed_chrdevreg;
	}

	hpsdr_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(hpsdr_class)) {
		err("failed to register device class '%s'\n", CLASS_NAME);
		retval = PTR_ERR(hpsdr_class);
		goto failed_classreg;
	}

	hpsdr_device = device_create(hpsdr_class, NULL, MKDEV(hpsdr_major, 0), NULL, CLASS_NAME TX_DEVICE_NAME);
	if(IS_ERR(hpsdr_device)) {
		err("failed to create device '%s%s'\n", CLASS_NAME, TX_DEVICE_NAME);
		retval = PTR_ERR(hpsdr_device);
		goto failed_devreg;
	}

	// Allocate memory region for the registers in the lightweight bridge
	// This is for parameter changes accomplished through ioctls
	if(request_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE, "hpsdr-control") == NULL) {
		err("failed to reserve memory\n");
		goto failed_devreg;
	}

	return 0;

	failed_devreg:
		class_unregister(hpsdr_class);
		class_destroy(hpsdr_class);
	failed_classreg:
		unregister_chrdev(hpsdr_major, TX_DEVICE_NAME);
	failed_chrdevreg:
		return -1;
}

static void __exit hpsdr_exit(void) {
	release_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE);
	device_destroy(hpsdr_class, MKDEV(hpsdr_major, 0));
	class_unregister(hpsdr_class);
	class_destroy(hpsdr_class);
	unregister_chrdev(hpsdr_major, TX_DEVICE_NAME);
}

module_init(hpsdr_init);
module_exit(hpsdr_exit);

MODULE_AUTHOR("Jeremy McDermond <nh6z@nh6z.net>");
MODULE_DESCRIPTION("OpenHPSDR SoC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
