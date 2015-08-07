/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : pswitch.c
 ** Description : pswitch  Driver
 ** Date        : 2014-03-21 22:49
 ** Author      : Prd.SenDrv
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/ 
 
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>

//#define PSWITCH_STATUS_PROC
//#define PSWITCH_STATUS_SIGNAL

#ifdef PSWITCH_STATUS_PROC
#include <linux/proc_fs.h> 
static struct proc_dir_entry *rotordir = NULL;
static char* rotor_node_name = "cam_status";
#endif


#ifdef PSWITCH_STATUS_SIGNAL
#include <linux/fs.h>   
#include <asm/siginfo.h>
#include <asm/signal.h>
static struct fasync_struct *async_queue = NULL;
#endif

#define PSWITCH_EVENT_TYPE					EV_KEY

#ifdef PSWITCH_STATUS_PROC
#define PSWITCH_EVENT_CODE					KEY_F2
#else
#define PSWITCH_EVENT_CODE					KEY_F1
#endif

#define PSWITCH_DEV_NAME "pswitch"
static unsigned int PSWITCH_IRQ_GPIO = 0xFFFF;
#define PSWITCH_LOG(format, args...) printk(KERN_EMERG PSWITCH_DEV_NAME " "format,##args)


//for switch shrapnel 
#define PSWITCH_IOCTL_BASE                   0x8A
#define PSWITCH_IOCTL_GET_DATA				_IOR(PSWITCH_IOCTL_BASE, 0x00, int*)


struct pswitch_sensor {
	unsigned int irq_num;
	unsigned int gpio_status;
	struct input_dev *input_dev;
	struct work_struct work;
};
static struct workqueue_struct *pswitch_wq;
static int g_pswitch_status = -1;
static struct pswitch_sensor *gpswitch = NULL;
/*----------------------------------------------------------------*/
static struct of_device_id pswitch_device_id[] = {
		{.compatible = "pswitch",},
		{},
};
/*----------------------------------------------------------------*/


/* extern func for camera */
int getCameraStatusByPswtich(void)
{
    return g_pswitch_status;  // 1: back   0: front   -1:invalid
}

#ifdef PSWITCH_STATUS_SIGNAL

static int pswitch_fasync(int fd, struct file * filp, int on) 
{
    return fasync_helper(fd, filp, on, &async_queue);
}

#endif

static irqreturn_t pswitch_irq_handler(int irq, void *dev_id)
{
	struct pswitch_sensor *priv = gpswitch;
	//PSWITCH_LOG(" Interrupt coming\n");
	disable_irq_nosync(priv->irq_num);
	queue_work(pswitch_wq, &(priv->work));
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------*/
static void pswitch_work_func(struct work_struct *work)
{
    int ret;
    struct pswitch_sensor *priv = gpswitch;

    PSWITCH_LOG("%s enter\n",__func__);
    
    msleep(50);

    if (__gpio_get_value(PSWITCH_IRQ_GPIO) != g_pswitch_status)
    {
        g_pswitch_status = priv->gpio_status = !g_pswitch_status;
        
        PSWITCH_LOG(" %s now status is %s\n", __FUNCTION__,  priv->gpio_status ? "back" : "front");

        input_report_key(priv->input_dev, PSWITCH_EVENT_CODE, 1);
        input_sync(priv->input_dev);
        input_report_key(priv->input_dev, PSWITCH_EVENT_CODE, 0);
        input_sync(priv->input_dev);

        if (priv->gpio_status == 1)
        ret = irq_set_irq_type(priv->irq_num, IRQ_TYPE_LEVEL_LOW);
        else 
        ret = irq_set_irq_type(priv->irq_num, IRQ_TYPE_LEVEL_HIGH);

#ifdef PSWITCH_STATUS_SIGNAL
         if (async_queue)
            kill_fasync(&async_queue, SIGIO, POLL_IN); 
#endif
        
    }
    else
    {
        PSWITCH_LOG(" %s ignore the invalid interrupt . \n", __FUNCTION__); 
    }
    enable_irq(priv->irq_num);
}



static ssize_t pswitch_status_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{

    PSWITCH_LOG("%s status:%d \n",__func__, g_pswitch_status);
    return sprintf(buf, "%d \n", g_pswitch_status);
}

static DEVICE_ATTR(status, S_IRUGO | S_IWUSR | S_IWGRP,
                   pswitch_status_show, NULL);

static struct attribute *pswitch_attributes[] = {
       &dev_attr_status.attr,             
	NULL
};

static struct attribute_group pswitch_attribute_group = {
	.attrs = pswitch_attributes
};

#ifdef PSWITCH_STATUS_PROC
static ssize_t rotor_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
	char page[8]; 	
	char *p = page;	
	int len = 0; 	
	p += sprintf(p, "%d\n", g_pswitch_status);	
	len = p - page;	
	if (len > *pos)		
		len -= *pos;	
	else		
		len = 0;	

	if (copy_to_user(buf,page,len < count ? len  : count))		
		return -EFAULT;	
	*pos = *pos + (len < count ? len  : count);	

	return len < count ? len  : count;
}

#if 0
static ssize_t rotor_node_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
	char tmp[32] = {0};	
	int ret;		
	if (count > 2)		
		return -EINVAL;		
	ret = copy_from_user(tmp, buf, 32);
	
	sscanf(tmp, "%d", &g_pswitch_status);	
	
	return count;	
}
#endif
static struct file_operations rotor_node_ctrl = {
	.read = rotor_node_read,
	//.write = rotor_node_write,  
};

#endif


static int pswitch_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int pswitch_release(struct inode *inode, struct file *file)
{
#ifdef PSWITCH_STATUS_SIGNAL
    fasync_helper(-1, file, 0, &async_queue);
#endif
    return 0;
}
static long pswitch_unlocked_ioctl(struct file *file, unsigned int cmd,
                                  unsigned long arg)
{
    //int data;
    long err = 0;
    void __user *ptr = (void __user*) arg;

    switch (cmd)
    {
        case PSWITCH_IOCTL_GET_DATA:
            if (copy_to_user(ptr, &g_pswitch_status, sizeof(g_pswitch_status)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
        }
        return 0;
err_out:
    printk("%s error  cmd:%d \n", __FUNCTION__, cmd);
    return err;
}

static const struct file_operations pswitch_fops =
{
    //.owner = THIS_MODULE,
    .open = pswitch_open,
    .release = pswitch_release,
#ifdef PSWITCH_STATUS_SIGNAL    
    .fasync = pswitch_fasync,    
#endif
    .unlocked_ioctl = pswitch_unlocked_ioctl
};

static struct miscdevice pswitch_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "pswitch",
    .fops = &pswitch_fops,
};



/*----------------------------------------------------------------*/
static int pswitch_probe(struct platform_device *pdev)
{
	int ret;
	struct pswitch_sensor *priv;
	struct device_node *np = pdev->dev.of_node;
	PSWITCH_LOG("%s Enter\n",__func__);
	
	priv = kzalloc(sizeof(struct pswitch_sensor), GFP_KERNEL);
	if (priv == NULL)
	{
		PSWITCH_LOG("allocate mem fail\n");
		goto mem_err;
	}
	
	pswitch_wq = create_singlethread_workqueue("pswitch_handler_wq");
	INIT_WORK(&(priv->work), pswitch_work_func);
	
	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL)
	{
		PSWITCH_LOG("registe input device fail\n");
		goto input_allocate_err;
	}
	priv->input_dev->name = PSWITCH_DEV_NAME;
	set_bit(PSWITCH_EVENT_TYPE, priv->input_dev->evbit);	
	set_bit(PSWITCH_EVENT_CODE, priv->input_dev->keybit);


	ret = input_register_device(priv->input_dev);
	if (ret < 0)
	{
		PSWITCH_LOG("registe input device fail\n");
		goto input_register_err;
	}

        if ((ret = misc_register(&pswitch_device)))
        {
            printk("misc_register fail: %d\n", ret);
            goto input_register_err;
        }

	PSWITCH_IRQ_GPIO = of_get_named_gpio(np, "pswitch,irq-gpio", 0);
	PSWITCH_LOG("GPIO %d use for PSWITCH_DEV_NAME interrupt\n",PSWITCH_IRQ_GPIO);
	
	ret = gpio_request(PSWITCH_IRQ_GPIO, PSWITCH_DEV_NAME);
	if (ret) {
		PSWITCH_LOG(" unable to request gpio %d\n", PSWITCH_IRQ_GPIO);
		goto gpio_err;
	}
	ret = gpio_direction_input(PSWITCH_IRQ_GPIO);
	if (ret)
		PSWITCH_LOG(" unable to set gpio %d\n direction", PSWITCH_IRQ_GPIO);

	priv->irq_num = gpio_to_irq(PSWITCH_IRQ_GPIO);
	
	PSWITCH_LOG("irq_num = %d\n", priv->irq_num);
	
	g_pswitch_status = priv->gpio_status = __gpio_get_value(PSWITCH_IRQ_GPIO);
	PSWITCH_LOG("gpswitch->gpio_status = %d\n", priv->gpio_status);
	
	input_report_switch(priv->input_dev, PSWITCH_EVENT_CODE, priv->gpio_status);
	input_sync(priv->input_dev);

	if (priv->gpio_status == 1)
		ret = request_irq(priv->irq_num, pswitch_irq_handler, 
		IRQ_TYPE_LEVEL_LOW, PSWITCH_DEV_NAME, NULL);
	else 
		ret = request_irq(priv->irq_num, pswitch_irq_handler, 
		IRQ_TYPE_LEVEL_HIGH, PSWITCH_DEV_NAME, NULL);

	if (ret){
		PSWITCH_LOG(" request irq fail\n");
		goto irq_err;
	}

	/*  create sysfs group */
	ret = sysfs_create_group(&priv->input_dev->dev.kobj, &pswitch_attribute_group);
	if(ret) {
		PSWITCH_LOG("sysfs_create_group was failed(%d)", ret);
		goto irq_err;
	}

#ifdef PSWITCH_STATUS_PROC
        rotordir = proc_create(rotor_node_name, 0664, NULL, &rotor_node_ctrl); 
        if (rotordir == NULL)
        {
            PSWITCH_LOG(" create proc/%s fail\n", rotor_node_name);
            goto irq_err;
        }
#endif    

	gpswitch = priv;
	enable_irq_wake(priv->irq_num);
	
	PSWITCH_LOG("%s Exit\n", __func__);
	return ret;
	
irq_err:
	free_irq(priv->irq_num, PSWITCH_DEV_NAME);
gpio_err:
	gpio_free(PSWITCH_IRQ_GPIO);
input_register_err:
	input_unregister_device(priv->input_dev);
input_allocate_err:
	input_free_device(priv->input_dev);
mem_err:
	kfree(priv);
	return -1;
}
/*----------------------------------------------------------------*/
static int pswitch_remove(struct platform_device *dev)
{
	disable_irq(gpswitch->irq_num);
	free_irq(gpswitch->irq_num, PSWITCH_DEV_NAME);
	input_unregister_device(gpswitch->input_dev);
	input_free_device(gpswitch->input_dev);
	gpio_free(PSWITCH_IRQ_GPIO);
	kfree(gpswitch);
	return 0;
}
/*----------------------------------------------------------------*/
static int pswitch_pm_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}
/*----------------------------------------------------------------*/
static int pswitch_pm_resume(struct platform_device *dev)
{
	return 0;
}
/*----------------------------------------------------------------*/
static struct platform_driver pswitch_platform_driver = {
	.probe = pswitch_probe,
	.remove = pswitch_remove,
	.suspend = pswitch_pm_suspend,
	.resume = pswitch_pm_resume,
	.driver = {
		.name = PSWITCH_DEV_NAME,
		.of_match_table = pswitch_device_id
	},
};
/*----------------------------------------------------------------*/
module_platform_driver(pswitch_platform_driver);
/*----------------------------------------------------------------*/
MODULE_AUTHOR("lauson <lauson@oppo.com>");
MODULE_DESCRIPTION("OPPO pswitch driver");
MODULE_LICENSE("GPL");

