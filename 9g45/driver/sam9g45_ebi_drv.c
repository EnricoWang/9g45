/* ***************************************************************************************************************************
 **************   ____________   _______   _________  _________ __       __    _________ ************************************
 **************  /____  _____/ / _____  / / ______ / / _______/ \ \     / /   / _______/ ************************************ 
 **************      / /      / /    / / / /     // / /          \ \   / /   / /         ************************************
 **************     / /      / /    / / / /_____// / /______      \_\ /_/   / /______    ************************************
 **************    / /      / /    / / /  ______/ /______  /        | |    / _____  /    ************************************
 **************   / /      / /____/ / / /        _______/ /         | |  ________/ /     ************************************       
 **************  /_/      /________/ /_/        /________/          |_| /_________/      ************************************
 ************************************************************************************************************************** */          
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include "/home/enrico/topsys/src/kernel/linux-3.6.9/include/linux/smc911x.h"

volatile unsigned short *cs_base;
static int major;
struct class *cls;
unsigned short val;

typedef struct __ioctl_data {
	int offset;
	unsigned int dataBuf;
}pdata;

static long sam9g45_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{

	pdata  *dataTest;
	dataTest = (pdata *)arg;
	switch (cmd) {
		case 1: writew(dataTest->dataBuf, cs_base + (dataTest->offset));
				break;
		case 2: //神经病，原因不明；
				printk("Hello ervryone, i'm neuropathy!\n");
				break;
		case 3: val = readw(cs_base + (dataTest->offset));
				break;
		default:
				printk("cmd error, try again!\n");
				break;
	}
	return 0;
}

static ssize_t sam9g45_write(struct file *filp, const char __user *buff,size_t count, loff_t *f_pos)
{

	return 0;
}

static ssize_t sam9g45_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	int ret;
	ret = copy_to_user(buff, &val, 2);
	if (ret != 0) 
		printk("copy to user error\n");
	return 0;
}

int sam9g45_open(struct inode *inode, struct file *filp)
{
	writew(0x0002, cs_base);
	
	return 0;
}

int sam9g45_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations sam9g45_drv_fops = {
	.open        = sam9g45_open,
	.release     = sam9g45_release,
	.read        = sam9g45_read,
	.write       = sam9g45_write,
	.unlocked_ioctl       = sam9g45_ioctl,
};

static int sam9g45_drv_probe(struct platform_device *pdev)
{
	struct resource		*res;

	/* 根据platform_device的资源进行ioremap */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cs_base = ioremap(res->start, res->end - res->start + 1);
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	/* 注册字符设备驱动程序 */
	major = register_chrdev(0, "sam9g45", &sam9g45_drv_fops);
	if (major < 0) {
		printk(KERN_ALERT"cannot register chrdev\n");
	}

	cls = class_create(THIS_MODULE, "sam9g45");

	device_create(cls, NULL, MKDEV(major, 0), NULL, "sam9g45"); /* /dev/sam9g45 */

	return 0;
}

static int sam9g45_drv_remove(struct platform_device *pdev)
{
	device_destroy(cls, MKDEV(major, 0));
	class_destroy(cls);

	unregister_chrdev(major, "sam9g45");
	iounmap(cs_base);
	return 0;
}

static struct platform_driver sam9g45_driver = {
	.probe		 = sam9g45_drv_probe,
	.remove	 = __devexit_p(sam9g45_drv_remove),
	.driver	 = {
		.name	 = "sam9g45",
		.owner	= THIS_MODULE,
	},
};

static int sam9g45_drv_init(void)
{
	platform_driver_register(&sam9g45_driver);
	return 0;
}

static void sam9g45_drv_exit(void)
{
	platform_driver_unregister(&sam9g45_driver);
}

module_init(sam9g45_drv_init);
module_exit(sam9g45_drv_exit);
MODULE_LICENSE("GPL");
