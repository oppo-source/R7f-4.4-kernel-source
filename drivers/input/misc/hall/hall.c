/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : hall.c
 ** Description : Hall Sensor Driver
 ** Date        : 2014-1-6 22:49
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
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>

#define HALL_DEV_NAME "hall"
static unsigned int HALL_IRQ_GPIO = 0xFFFF;
#define HALL_LOG(format, args...) printk(KERN_EMERG HALL_DEV_NAME " "format,##args)

#define HALL_STATUS_PROC
#ifdef HALL_STATUS_PROC
#include <linux/proc_fs.h> 
#include <asm/uaccess.h>
static struct proc_dir_entry *halldir = NULL;
static char* hall_node_name = "hall_status";
#endif

#define TIMER_INTERVAL_MS 10

struct hall_sensor {
	unsigned int irq_num;
	unsigned int gpio_status;
	struct input_dev *input_dev;
	struct delayed_work work;
	struct regulator *vreg_ctrl;
	spinlock_t hall_spinlock;
	struct mutex hall_mutex;
};

static struct workqueue_struct *hall_wq;
static struct hall_sensor *ghall = NULL;
/*----------------------------------------------------------------*/
static struct of_device_id hall_device_id[] = {
		{.compatible = "hall",},
		{},
};
/*----------------------------------------------------------------*/
static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{
	struct hall_sensor *priv = ghall;
	unsigned long flags;
	HALL_LOG(" Interrupt coming\n");
	spin_lock_irqsave(&priv->hall_spinlock, flags);
	
	disable_irq_nosync(priv->irq_num);
	queue_delayed_work(hall_wq, &(priv->work), TIMER_INTERVAL_MS);
	
	spin_unlock_irqrestore(&priv->hall_spinlock, flags);
	return IRQ_HANDLED;
}
/*----------------------------------------------------------------*/
static void hall_work_func(struct work_struct *work)
{
	int ret;
	struct hall_sensor *priv = ghall;
	
	priv->gpio_status = __gpio_get_value(HALL_IRQ_GPIO);
	HALL_LOG("%s: now gpio status is %s\n",__func__, priv->gpio_status ? "up" : "down");
	
mutex_lock(&priv->hall_mutex);

	if (priv->gpio_status == 1)
		ret = irq_set_irq_type(priv->irq_num, IRQ_TYPE_LEVEL_LOW);
	else 
		ret = irq_set_irq_type(priv->irq_num, IRQ_TYPE_LEVEL_HIGH);

	input_report_switch(priv->input_dev, SW_LID, !priv->gpio_status);
	input_sync(priv->input_dev);
	enable_irq(priv->irq_num);

mutex_unlock(&priv->hall_mutex);
}

#ifdef HALL_STATUS_PROC
static ssize_t hall_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
	char page[8]; 	
	char *p = page;	
	int len = 0; 	
	p += sprintf(p, "%d\n", ghall->gpio_status);	
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
static ssize_t hall_node_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
	char tmp[32] = {0};	
	int ret;		
	if (count > 2)		
		return -EINVAL;		
	ret = copy_from_user(tmp, buf, 32);
	
	sscanf(tmp, "%d", &(ghall->gpio_status));	
	
	return count;	
}
#endif
static struct file_operations hall_node_ctrl = {
	.read = hall_node_read,
	//.write = hall_node_write,  
};

#endif//HALL_STATUS_PROC

/*----------------------------------------------------------------*/
static int hall_probe(struct platform_device *pdev)
{
	int ret;
	struct hall_sensor *priv;
	struct device_node *np = pdev->dev.of_node;
	HALL_LOG("%s Enter\n",__func__);
	
	priv = kzalloc(sizeof(struct hall_sensor), GFP_KERNEL);
	if (priv == NULL)
	{
		HALL_LOG("allocate mem fail\n");
		goto mem_err;
	}
	
	hall_wq = create_singlethread_workqueue("hall_handler_wq");
	INIT_DELAYED_WORK(&(priv->work), hall_work_func);
	
	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL)
	{
		HALL_LOG("registe input device fail\n");
		goto input_allocate_err;
	}
	priv->input_dev->name = HALL_DEV_NAME;
	set_bit(EV_SW, priv->input_dev->evbit);
    	set_bit(SW_LID, priv->input_dev->swbit);

	ret = input_register_device(priv->input_dev);
	if (ret < 0)
	{
		HALL_LOG("registe input device fail\n");
		goto input_register_err;
	}

	priv->vreg_ctrl = regulator_get(&pdev->dev, "vreg_phy");
	if (IS_ERR(priv->vreg_ctrl)) {
		ret = PTR_ERR(priv->vreg_ctrl);
		HALL_LOG("Regulator get failed vreg_phy ret=%d\n", ret);
	}

	if (regulator_count_voltages(priv->vreg_ctrl) > 0) {
		ret = regulator_set_voltage(priv->vreg_ctrl, 1800000,
					   1800000);
		if (ret) {
			HALL_LOG("Regulator set_vtg failed vreg_phy ret=%d\n", ret);
		}
	}	
	ret = regulator_enable(priv->vreg_ctrl);
	if (ret) {
		HALL_LOG("Regulator vreg_phy enable failed ret=%d\n", ret);
	}
	
	HALL_IRQ_GPIO = of_get_named_gpio(np, "hall,irq-gpio", 0);
	HALL_LOG("GPIO %d use for HALL interrupt\n",HALL_IRQ_GPIO);
	
	ret = gpio_request(HALL_IRQ_GPIO, HALL_DEV_NAME);
	if (ret) {
		HALL_LOG(" unable to request gpio %d\n", HALL_IRQ_GPIO);
		goto gpio_err;
	}
	ret = gpio_direction_input(HALL_IRQ_GPIO);
	if (ret)
		HALL_LOG(" unable to set gpio %d\n direction", HALL_IRQ_GPIO);

	priv->irq_num = gpio_to_irq(HALL_IRQ_GPIO);
	
	HALL_LOG("irq_num = %d\n", priv->irq_num);
	
	irq_set_irq_wake(priv->irq_num, 1);
	
	priv->gpio_status = __gpio_get_value(HALL_IRQ_GPIO);
	HALL_LOG("ghall->gpio_status = %d\n", priv->gpio_status);
	
	input_report_switch(priv->input_dev, SW_LID, !priv->gpio_status);
	input_sync(priv->input_dev);
  
    ghall = priv;
    
#ifdef HALL_STATUS_PROC
        halldir = proc_create(hall_node_name, 0664, NULL, &hall_node_ctrl); 
        if (halldir == NULL)
        {
            printk(" create proc/%s fail\n", hall_node_name);
            goto gpio_err;
        }
#endif   

	spin_lock_init(&priv->hall_spinlock);
	mutex_init(&priv->hall_mutex);

	if (priv->gpio_status == 1)
		ret = request_irq(priv->irq_num, hall_irq_handler, 
		IRQ_TYPE_LEVEL_LOW, HALL_DEV_NAME, NULL);
	else 
		ret = request_irq(priv->irq_num, hall_irq_handler, 
		IRQ_TYPE_LEVEL_HIGH, HALL_DEV_NAME, NULL);

	if (ret){
		HALL_LOG(" request irq fail\n");
		goto irq_err;
	}
	
	enable_irq_wake(priv->irq_num);
	
	HALL_LOG("%s Exit\n", __func__);
	return ret;
	
irq_err:
	free_irq(priv->irq_num, HALL_DEV_NAME);
gpio_err:
	gpio_free(HALL_IRQ_GPIO);
input_register_err:
	input_unregister_device(priv->input_dev);
input_allocate_err:
	input_free_device(priv->input_dev);
mem_err:
	kfree(priv);
	return -1;
}
/*----------------------------------------------------------------*/
static int hall_remove(struct platform_device *dev)
{
	disable_irq(ghall->irq_num);
	free_irq(ghall->irq_num, HALL_DEV_NAME);
	input_unregister_device(ghall->input_dev);
	input_free_device(ghall->input_dev);
	gpio_free(HALL_IRQ_GPIO);
	kfree(ghall);
	return 0;
}
/*----------------------------------------------------------------*/
static int hall_pm_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}
/*----------------------------------------------------------------*/
static int hall_pm_resume(struct platform_device *dev)
{
	return 0;
}
/*----------------------------------------------------------------*/
static struct platform_driver hall_platform_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.suspend = hall_pm_suspend,
	.resume = hall_pm_resume,
	.driver = {
		.name = HALL_DEV_NAME,
		.of_match_table = hall_device_id
	},
};
/*----------------------------------------------------------------*/
module_platform_driver(hall_platform_driver);
/*----------------------------------------------------------------*/
MODULE_AUTHOR("ye.zhang <zhye@oppo.com>");
MODULE_DESCRIPTION("OPPO hall driver");
MODULE_LICENSE("GPL");

