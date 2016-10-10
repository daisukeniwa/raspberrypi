#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/uaccess.h>

#include "mydev.h"

MODULE_LICENSE("GPL");

#define GPIO_BASE 0x3f200000

#define GPIO_FUNC_SEL_0 0x00
#define GPIO_FUNC_SEL_1 0x04
#define GPIO_FUNC_SEL_2 0x08
#define GPIO_FUNC_SEL_3 0x0c
#define GPIO_PIN_SET_0 0x1c
#define GPIO_PIN_CLR_0 0x28
#define GPIO_PIN_LV_0 0x34
#define GPIO_PULL 0x94
#define GPIO_PULL_CLK 0x98

#define GPIO_SET0_INDEX 7
#define GPIO_CLR0_INDEX 10
#define GPIO_LEV0_INDEX 13

#define RPI_GPIO_MASK (uint32_t)((0x01 << 2) | (0x01 << 3) |(0x01 << 4) | \
				(0x01 << 7) |(0x01 << 8) |(0x01 << 9) | \
				(0x01 << 10) |(0x01 << 11) |(0x01 << 14) | \
				(0x01 << 15) |(0x01 << 17) |(0x01 << 18) | \
				(0x01 << 22) |(0x01 << 23) |(0x01 << 24) | \
				(0x01 << 25) |(0x01 << 27))

//static int char_bufsize = 10;
//static int char_devs = 1;
static int char_major = 0;
static int char_minor = 0;

static void __iomem *gpio_map;
static volatile uint32_t *gpio_base;

struct cdev gDev;
struct class *p;

struct ioctl_cmd {
	unsigned int reg;
	unsigned int offset;
	unsigned int val;
};

#define IOC_MAGIC 'd'

#define IOCTL_VALGET _IOR(IOC_MAGIC, 2, struct ioctl_cmd)
#define IOCTL_VALSET _IOW(IOC_MAGIC, 1, struct ioctl_cmd)

static int gpio_function_set(int pin, uint32_t func)
{
	int index = pin / 10;
//	uint32_t mask = ~(0x7 << ((pin % 10) * 3));
//	gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));
	gpio_base[index] = 0x00000200;
	printk("gpio_base[%d] = %p\n", index, &gpio_base[index]);
	return 1;
}

static void gpio_set32(uint32_t mask, uint32_t val)
{
	gpio_base[GPIO_SET0_INDEX] = val & mask;
}

static uint32_t gpio_get32(uint32_t mask)
{
	return gpio_base[GPIO_LEV0_INDEX] & mask;
}

static void gpio_clear32(uint32_t mask, uint32_t val)
{
	gpio_base[GPIO_CLR0_INDEX] = val & mask;
}

static gpio_setpin(int pin)
{
	gpio_base[GPIO_SET0_INDEX] = 1 << pin;
}

static uint32_t gpio_getpin(int pin)
{
	return (gpio_base[GPIO_LEV0_INDEX] & (1 << pin));
}

static void gpio_clearpin(int pin)
{
	gpio_base[GPIO_CLR0_INDEX] = 1 << pin;
}

static void init_gpio(void)
{
	if (!request_mem_region(GPIO_BASE, 4096, "rpi_gpio_map")) {
		printk("request_mem_resion failed\n");
		return;
	}

	gpio_map = ioremap_nocache(GPIO_BASE, 4096);
	gpio_base = (volatile uint32_t *)gpio_map;
	printk("gpio_map = %p gpio_base = %p\n", gpio_map, gpio_base);
	gpio_function_set(0x17, 0x01);
}

static void deinit_gpio(void)
{
	iounmap(gpio_map);
	release_mem_region(GPIO_BASE, 4096);

	gpio_map = NULL;
	gpio_base = NULL;
}

static int mydev_open(struct inode *node, struct file *file)
{
	printk("*+*+*+*+*+ mydev_open *+*+*+*+*+\n");
	init_gpio();
	return 0;
}

static int mydev_close(struct inode *node, struct file *file)
{
	printk("*+*+*+*+*+ mydev_close *+*+*+*+*+\n");
	deinit_gpio();
	return 0;
}

static int gVal;

static long mydev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ioctl_cmd data;

	printk("*+*+*+*+*+ mydev_ioctl *+*+*+*+*+\n");

	memset(&data, 0, sizeof(data));

	switch (cmd) {
		case IOCTL_VALSET:
			printk("IOCTL_VALSET\n");
			if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
				printk("Failed to copy_from_user\n");
				return -EFAULT;
			}
			gVal = data.val;
			printk("IOCTL_VALSET gVal = %d\n", gVal);
			if (gVal > 0) {
//				gpio_base[GPIO_CLR0_INDEX] = 0x0800000;
				gpio_base[GPIO_SET0_INDEX] = 0x0800000;
//				printk("gpio_base[%d] = %p\n", GPIO_CLR0_INDEX, &gpio_base[GPIO_CLR0_INDEX]);
				printk("gpio_base[%d] = %p\n", GPIO_SET0_INDEX, &gpio_base[GPIO_SET0_INDEX]);
//				gpio_clear32(RPI_GPIO_MASK, 1 << 23);
//				gpio_set32(RPI_GPIO_MASK, 1 << 23);
			} else {
				gpio_base[GPIO_CLR0_INDEX] = 0x0800000;
//				gpio_clear32(RPI_GPIO_MASK, 1 << 23);
			}
			break;
		case IOCTL_VALGET:
			printk("IOCTL_VALGET\n");
			data.val = gVal;
			if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
				printk("Failed to copy_to_user\n");
				return -EFAULT;
			}
			printk("IOCTL_VALGET gVal = %d\n", gVal);
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations gMydev_fops = {
	.open = mydev_open,
	.release = mydev_close,
	.unlocked_ioctl = mydev_ioctl,
};

static int mydev_init(void)
{
	dev_t dev;
	int ret, devno;
	
	printk("*+*+*+*+*+ mydev_init was called *+*+*+*+*+\n");
	ret = alloc_chrdev_region(&dev, 0, 1, "mydev");
	if (ret) {
		printk("*+*+*+*+*+ Failed alloc_chrdev_resion *+*+*+*+*+\n");
		return -1;
	}
	char_major = MAJOR(dev);

	p = class_create(THIS_MODULE, "mydev");
	if (IS_ERR(p)) {
		return -1;
	}

	devno = MKDEV(char_major, char_minor);
	cdev_init(&gDev, &gMydev_fops);
	gDev.owner = THIS_MODULE;
	ret = cdev_add(&gDev, devno, 1);
	if (ret) {
		printk("*+*+*+*+*+ Failed cdev_add *+*+*+*+*+\n");
		return -1;
	}
	device_create(p, NULL, devno, NULL, "mydev");

	

	printk("*+*+*+*+*+ mydev_init END *+*+*+*+*+\n");
	return 0;
}

static void mydev_exit(void)
{
	dev_t devno;

	printk("*+*+*+*+*+ mydev_exit was called *+*+*+*+*+\n");
	cdev_del(&gDev);
	devno = MKDEV(char_major, char_minor);
	device_destroy(p, devno);	
	unregister_chrdev_region(devno, 1);
	class_destroy(p);
	printk("*+*+*+*+*+ mydev_exit END *+*+*+*+*+\n");
}

module_init(mydev_init);
module_exit(mydev_exit);


