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
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <asm/div64.h>

#include "hpsdr.h"

#define CLASS_NAME "hpsdr"

#define LWH2FBRG_BASE	0xFF200000
#define H2FBRG_BASE	0xC0000000

#define FIFO_SIZE	327680
#define HW_FIFO_SIZE 	8192 //  In 32-bit words
#define RX_INT_BUFFER_SIZE 512

#define FIFO_STATUS_BASE LWH2FBRG_BASE + 0x00000000
#define FIFO_OUTPUT_BASE H2FBRG_BASE + 0x00000000

#define FIFO_FILL_LEVEL FIFO_STATUS_BASE + 0x00
#define FIFO_I_STATUS FIFO_STATUS_BASE + 0x04
#define FIFO_EVENT FIFO_STATUS_BASE + 0x08
#define FIFO_INTERRUPT_ENABLE FIFO_STATUS_BASE + 0x0C
#define FIFO_ALMOST_FULL FIFO_STATUS_BASE + 0x10

#define FIFO_IRQ	72

#define LED_CONTROL_BASE LWH2FBRG_BASE + 0x00000400

#define FIFO_CONTROL_BASE LWH2FBRG_BASE + 0x00000800
#define PHASE_WORD_CONTROL_BASE FIFO_CONTROL_BASE + 4

#define HPSDR_NR_RX	7

#define M2 0x45E7B273

#define err(format, arg...) pr_err(CLASS_NAME ": " format, ## arg)
#define info(format, arg...) pr_info(CLASS_NAME ": " format, ## arg)
#define warn(format, arg...) pr_warn(CLASS_NAME ": " format, ## arg)

int hpsdr_nr_rx = HPSDR_NR_RX;

module_param(hpsdr_nr_rx, int, S_IRUGO);

static struct class *hpsdr_class = NULL;
static dev_t first_dev;

//  These should be in a filp eventually
struct hpsdr_dev {
	struct cdev cdev;
	uint32_t *fifo_register;
	uint8_t *event;
	DECLARE_KFIFO_PTR(read_fifo, uint32_t);
	struct semaphore sem;
	spinlock_t open_lock;
	wait_queue_head_t queue;
	uint8_t index;
	uint32_t *rx_buffer;
	uint32_t *filllevel_reg;
	uint32_t *status_reg;
	uint32_t *led_control_reg;
	uint32_t *fifo_control_reg;
	uint32_t *phase_word_control_reg;
	int open_count;
};

// XXX This really shouldn't be here
static ssize_t divisor_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size);
static ssize_t divisor_show(struct device *dev, struct device_attribute *attr, char *buffer);
static ssize_t frequency_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size);
static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buffer);

static DEVICE_ATTR_RW(divisor);
static DEVICE_ATTR_RW(frequency);

static struct attribute *control_attrs[] = {
	&dev_attr_divisor.attr,
	&dev_attr_frequency.attr,
	NULL,
};

ATTRIBUTE_GROUPS(control);

static struct hpsdr_dev *hpsdr_devices;

static irqreturn_t hpsdr_irq_handler(int irq, void *dev_id) {
	struct hpsdr_dev *dev = (struct hpsdr_dev *) dev_id;
	uint8_t event;
	unsigned int filllevel;
	
	if(dev == NULL) {
		err("Null device in interrupt hander\n");
		return -EIO;
	}
	
	event = ioread8(dev->event);
	if(event & 0x10) {
		err("Event register indicates overflow: %hhx\n", event);
	}
	if(event & 0x20) {
		err("Event register indicates underflow: %hhx\n", event);
	}
	
	if(event & 0x04) {
		filllevel = ioread32(dev->filllevel_reg);
		// err("Reading %u entries from register\n", filllevel);
		filllevel = filllevel == 0 ? HW_FIFO_SIZE : filllevel;

		//err("Running interrupt handler\n");
		ioread32_rep(dev->fifo_register, dev->rx_buffer, filllevel);
		//print_hex_dump(KERN_ERR, "raw data: ", DUMP_PREFIX_ADDRESS, 16, 4, dev->rx_buffer, RX_INT_BUFFER_SIZE * sizeof(uint32_t), 1);

		if(kfifo_is_full(&dev->read_fifo)) {
			err("FIFO is full in interrupt handler\n");
		}
		kfifo_in(&dev->read_fifo, dev->rx_buffer, filllevel);
		wake_up_interruptible(&dev->queue);
	}


	//  Clear the event register to reset the interrupt
	iowrite8(0xFF, dev->event);

	return IRQ_HANDLED;
}

static int hpsdr_rx_device_open(struct inode *inode, struct file *filp) {
	struct hpsdr_dev *dev;
	uint8_t *interrupts;
	uint8_t event;
	uint32_t *almostfull;
	uint32_t *junk;
	unsigned int filllevel;

	/* if(((filp->f_flags & O_ACCMODE) == O_WRONLY) || 
           ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		return -EACCES;
	} */

	err("Beginning open\n");

	dev = container_of(inode->i_cdev, struct hpsdr_dev, cdev);
	filp->private_data = dev;
	
	//  Make sure that nobody else is using the device.
	spin_lock(&dev->open_lock);
	if(dev->open_count) {
		spin_unlock(&dev->open_lock);
		return -EBUSY;
	}
	
	dev->open_count++;
	spin_unlock(&dev->open_lock);
	
	err("Acquired spinlock\n");

	//  Clean out the FIFOs of old data
	//ioread32(dev->fifo_register);
	filllevel = ioread32(dev->filllevel_reg);
	
	err("Cleaning out %d samples of junk\n", filllevel);
	junk = kmalloc(filllevel * sizeof(uint32_t), GFP_KERNEL);
	ioread32_rep(dev->fifo_register, junk, filllevel);
	kfree(junk);

	kfifo_reset_out(&dev->read_fifo);

	err("Cleaned FIFOs\n");

	dev->status_reg = ioremap(FIFO_I_STATUS, 4);

	//  Set the almost full threshold to the size of the buffer
	almostfull = ioremap(FIFO_ALMOST_FULL, 4);
	iowrite32(HW_FIFO_SIZE / 16, almostfull);
	iounmap(almostfull);

	err("Set Interrupt Threshold\n");

	//  Enable interrupts on the hardware
	interrupts = ioremap(FIFO_INTERRUPT_ENABLE, 1);
	//iowrite8(0x04, interrupts);
	iowrite8(0x34, interrupts);
	iounmap(interrupts);

	err("Enabled hardware interrupts\n");
	
	//  Set up an interrupt handler
	if(request_irq(FIFO_IRQ + dev->index, (irq_handler_t) hpsdr_irq_handler, 0, "hpsdr", (void *) dev)) {
		err("Couldn't assign interrupt: %d\n", FIFO_IRQ + dev->index);
		//  This isn't the right error
		return -ENOMEM;
	}

	
	event = ioread8(dev->event);
	err("Event reg is %hhx\n", event);
	iowrite8(0xFF, dev->event);

	err("Cleared event field\n");

	iowrite32(0xFFFFFFFF, dev->fifo_control_reg);
	err("Data flow enabled\n");

	err("Open complete\n");

	return 0;
}

static int hpsdr_rx_device_close(struct inode *inode, struct file *filp) {
	struct hpsdr_dev *dev = filp->private_data;
	uint8_t *interrupts;

	iowrite32(0x00000000, dev->fifo_control_reg);
	err("Stopped Data flow\n");

	//  Get rid of the interrupt handler
	free_irq(FIFO_IRQ + dev->index, (void *) dev);

	//  Disable interrupts in hardware
	interrupts = ioremap(FIFO_INTERRUPT_ENABLE, 1);
	iowrite8(0x00, interrupts);
	iounmap(interrupts);

	//  Unmap the event register
	
	iounmap(dev->status_reg);
	
	dev->open_count--;

	return 0;
}

static ssize_t hpsdr_rx_device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	int retval;
	unsigned int copied;
	struct hpsdr_dev *dev = filp->private_data;
	size_t count_elements = count / kfifo_esize(&dev->read_fifo);

	if(down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	//while(kfifo_is_empty(&dev->read_fifo)) {
	while(kfifo_len(&dev->read_fifo) < count_elements) {
		up(&dev->sem);
		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if(wait_event_interruptible(dev->queue, kfifo_len(&dev->read_fifo) >= count_elements))
			return -ERESTARTSYS;
		if(down_interruptible(&dev->sem))
			return -ERESTARTSYS;
	}

	retval = kfifo_to_user(&dev->read_fifo, buf, count, &copied);
	up(&dev->sem);
	return retval ? retval : copied;
}

static struct file_operations rx_fops = {
	.read = hpsdr_rx_device_read,
	.open = hpsdr_rx_device_open,
	.release = hpsdr_rx_device_close,
};

static int hpsdr_create_rxdev(int index, struct hpsdr_dev *dev) {
	//  Allocate a fifo for the device data
	if(kfifo_alloc(&dev->read_fifo, FIFO_SIZE, GFP_KERNEL)) {
		err("Couldn't allocate a read FIFO\n");
		return -ENOMEM;
	}

	//  Map the hardware FIFO output register. The register for each receiver
        //  is 4 bytes (a 32-bit value).  They are mapped sequentially into
        //  memory from the base.
	dev->fifo_register = (uint32_t *) ioremap(FIFO_OUTPUT_BASE + (index * 4), 4);

	//  XXX These offsets probably aren't good
	dev->led_control_reg = (uint32_t *) ioremap(LED_CONTROL_BASE + (index * 4), 4);
	dev->fifo_control_reg = (uint32_t *) ioremap(FIFO_CONTROL_BASE + (index * 4) , 4);
	dev->phase_word_control_reg = (uint32_t *) ioremap(PHASE_WORD_CONTROL_BASE + (index * 4), 4);
	dev->event = ioremap(FIFO_EVENT + (index * 4), 1);
	dev->filllevel_reg = ioremap(FIFO_FILL_LEVEL + (index * 4), 4);

	sema_init(&dev->sem, 1);
	init_waitqueue_head(&dev->queue);
	spin_lock_init(&dev->open_lock);
	
	dev->open_count = 0;
	
	dev->rx_buffer = kmalloc(HW_FIFO_SIZE * sizeof(uint32_t), GFP_KERNEL);

	if(device_create(hpsdr_class, NULL, first_dev + index, dev, "hpsdrrx%d", index) == NULL) {
		return -1;
	}
	cdev_init(&dev->cdev, &rx_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &rx_fops;
	if(cdev_add(&dev->cdev, first_dev + index, 1)) {
		device_destroy(hpsdr_class, first_dev + index);
		return -1;
	}

	dev->index = index;

	return 0;
}

static void hpsdr_cleanup_module(void) {
	int i;

	if(hpsdr_devices) {
		for(i = 0; i < hpsdr_nr_rx; ++i) {
	                kfifo_free(&hpsdr_devices[i].read_fifo);
			kfree(hpsdr_devices[i].rx_buffer);
			cdev_del(&hpsdr_devices[i].cdev);
			device_destroy(hpsdr_class, first_dev + i);
		}
		kfree(hpsdr_devices);
		iounmap(&hpsdr_devices[i].fifo_register);
		iounmap(&hpsdr_devices[i].led_control_reg);
		iounmap(&hpsdr_devices[i].filllevel_reg);
		iounmap(&hpsdr_devices[i].event);
	}

	class_unregister(hpsdr_class);
	class_destroy(hpsdr_class);

	unregister_chrdev_region(first_dev, hpsdr_nr_rx);
}

static int __init hpsdr_init(void) {
	int retval = -1;
	int i;

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

	//  XXX Register device attribute groups for the class here.
	hpsdr_class->dev_groups = control_groups;

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

	// This is only advisory in the kernel now.  We should allocate memory regions in both the
	// lightweight and regular FGPA->HPS bridges.
	// Allocate memory region for the registers in the lightweight bridge
	//if(request_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE, "hpsdr-control") == NULL) {
	//	err("failed to reserve memory\n");
	//	goto fail;
	//}

	return 0;

	fail:
		hpsdr_cleanup_module();
		return retval;
}

//  sysfs stuff here
static ssize_t divisor_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size) {
	uint32_t frequency;
	uint32_t divisor;
	struct hpsdr_dev *devinfo;

	if(kstrtouint(buffer, 0, &frequency)) {
		return -EFAULT;
	}

	err("sysfs gave us %d\n", frequency);
	divisor = 50000000 / frequency; 

	//  Figure out what device we're dealing with

	err("device name is %s\n", dev_name(dev));
	devinfo = dev_get_drvdata(dev);
	err("Device index is %d\n", devinfo->index);
	iowrite32(divisor, devinfo->led_control_reg);

	return size;
}

static ssize_t frequency_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size) {
	uint32_t frequency;
	uint64_t phaseword_tmp;
	uint32_t phaseword;
	struct hpsdr_dev *devinfo;

	devinfo = dev_get_drvdata(dev);

	if(kstrtouint(buffer, 0, &frequency)) {
		return -EFAULT;
	}
	
	//  XXX We should probably check the frequency values here to make sure they are from
	//  XXX 1 Hz to 60MHz or so.

	phaseword_tmp = (uint64_t) frequency * M2;
	phaseword = phaseword_tmp >> 25;
	iowrite32(phaseword, devinfo->phase_word_control_reg);

	return size;
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buffer) {
	struct hpsdr_dev *devinfo;
	uint32_t phaseword;
	uint64_t phaseword_tmp;

	devinfo = dev_get_drvdata(dev);

	phaseword = ioread32(devinfo->phase_word_control_reg);
	phaseword_tmp = (uint64_t) phaseword << 25;
	
	//  We use do_div here because the ARM Linux GCC produces an invalid symbol when doing
	//  64-bit divides.  The kernel has a macro to make this work.
	do_div(phaseword_tmp, M2);
	
	//  We add 1 to the result to account for rounding error.  This works for every
	//  frequency from 1 Hz - 60 MHz except for 33.554432 MHz.  The math works out so that
	//  you can never get this frequency back. It always becomes 33.554433.  If the
	//  phaseword is 1172812403, we'll force this to be a frequency of 33554432.
	if(phaseword != 1172812403)
		++phaseword_tmp;

	return snprintf(buffer, PAGE_SIZE, "%u", (uint32_t) phaseword_tmp);
}

static ssize_t divisor_show(struct device *dev, struct device_attribute *attr, char *buffer) {
	uint32_t divisor;
	struct hpsdr_dev *devinfo;

	devinfo = dev_get_drvdata(dev);

	divisor = ioread32(devinfo->led_control_reg);
	return snprintf(buffer, PAGE_SIZE,  "%u", 50000000 / divisor);
}

static void __exit hpsdr_exit(void) {
	// release_mem_region(LWH2FBRG_BASE + LED_BASE, LED_SIZE);
	hpsdr_cleanup_module();
}

module_init(hpsdr_init);
module_exit(hpsdr_exit);

MODULE_AUTHOR("Jeremy McDermond <nh6z@nh6z.net>");
MODULE_DESCRIPTION("OpenHPSDR SoC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
