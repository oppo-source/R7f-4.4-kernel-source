/***********************************************************
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: - pn544.c
* Description: Source file for nfc driver.
				
** Version: 1.0
** Date : 2013/10/15	
** Author: yuyi@Dep.Group.Module
** 

** --------------------------- Revision History: --------------------------------
** <author>		                      <data> 	<version >  <desc>
** ------------------------------------------------------------------------------
** Yuyi@Driver.NFC  2014/05/24   1.1	    modify clk_name for nfc input clock 
****************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/nfc/pn544.h>
#include <linux/regulator/consumer.h>
/*OPPO yuyi 2013-10-24 add begin for nfc_devinfo*/
//#include <linux/pcb_version.h>
#include <mach/device_info.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
/*OPPO yuyi 2013-10-24 add end for nfc_devinfo*/

#define MAX_BUFFER_SIZE	512
/* OPPO 2012-07-20 liuhd Add begin for reason */
#define NFC_POWER_ON 1
#define NFC_POWER_OFF 0

/*OPPO yuyi 2013-03-22 yuyi add begin     from 12025 board-8064.c*/
#define APQ_NFC_VEN_GPIO 118 //NFC_ENABLE
#define APQ_NFC_FIRM_GPIO 116  //NFC_UPDATE
#define APQ_NFC_IRQ_GPIO 117   //NFC_IRQ

#define PN544_VEN	GPIO_CFG(APQ_NFC_VEN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define PN544_FIRM	GPIO_CFG(APQ_NFC_FIRM_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define PN544_IRQ	GPIO_CFG(APQ_NFC_IRQ_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

 struct pn544_i2c_platform_data nfc_pdata  = {
		.irq_gpio = APQ_NFC_IRQ_GPIO,   //irq gpio
		.ven_gpio = APQ_NFC_VEN_GPIO,  
		.firm_gpio = APQ_NFC_FIRM_GPIO,  
};

/*OPPO yuyi 2013-03-22 yuyi add end*/

struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int		clk_req_gpio;
	bool				irq_enabled;
	spinlock_t			irq_enabled_lock;
};

/*OPPO yuyi 2013-10-24 add begin for nfc_devinfo*/
struct manufacture_info nfc_info = {
	.version = "pn65T",
	.manufacture = "NXP",
};
/*OPPO yuyi 2013-10-24 add end for nfc_devinfo*/
#if 0
/*OPPO yuyi 2013-03-22 add begin     from 12025 board-8064.c*/
  void pn544_power_init(void)
 {
	 int ret = 0  ;
 
	 //irq
	 ret = gpio_tlmm_config(PN544_IRQ, GPIO_CFG_ENABLE);
	 if (ret) {
		 printk(KERN_ERR "%s:gpio_tlmm_config(%#x)=%d\n",
				 __func__, PN544_IRQ, ret);
	 }
	 
	 //ven 
	 ret = gpio_tlmm_config(PN544_VEN, GPIO_CFG_ENABLE);
	 if (ret) {
		 printk(KERN_ERR "%s:gpio_tlmm_config(%#x)=%d\n",
			 __func__, PN544_VEN, ret);
	 }
	 
	 //gpio_set_value(APQ_NFC_VEN_GPIO, 0);
	 //msleep(50);
	 gpio_set_value(APQ_NFC_VEN_GPIO, 1);
	 msleep(100);

  //firmware gpio
	  ret = gpio_tlmm_config(PN544_FIRM, GPIO_CFG_ENABLE);
	  if (ret) {
		  printk(KERN_ERR "%s:gpio_tlmm_config(%#x)=%d\n",
			  __func__, PN544_FIRM, ret);
	  }
	  gpio_set_value(APQ_NFC_FIRM_GPIO, 0);
#if 0	  
	  printk(KERN_ERR "%s:liuhd for nfc gpio---\n",__func__);
#endif

 }
/*OPPO yuyi 2013-03-22 yuyi add end*/
#endif

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	if (!gpio_get_value(pn544_dev->irq_gpio)) 
	{
		return IRQ_HANDLED;
	}

	pn544_disable_irq(pn544_dev);
/* OPPO 2012-08-13 liuhd Modify begin for nfc */
#if 0
	printk("yuyi pn544 irq working\n");
#endif
	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 1 
	printk("%s : reading %zu bytes.\n", __func__, count);
#endif
/* OPPO 2012-08-13 liuhd Delete end */

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) 
	{
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	/* pn544 seems to be slow in handling I2C read requests
	 * so add 1ms delay after recv operation */
	if (ret < 0) 
	{
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) 
	{
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 1
	printk("IFD->PC:");
	for(i = 0; i < ret; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

    printk("%s before: writing %zu bytes.\n", __func__, count);	
	
	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
	}
	if (copy_from_user(tmp, buf, count)) 
	{
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
	printk("PC->IFD before:");
	for(i = 0; i < count; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 1
	printk("%s : writing %zu bytes.\n", __func__, count);
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) 
	{
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	printk("PC->IFD after:");
	for(i = 0; i < count; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
	
	/* pn544 seems to be slow in handling I2C write requests
	 * so add 1ms delay after I2C send oparation */
	
/* OPPO 2012-08-13 liuhd Delete begin for nfc */
#if 1 
	printk("PC->IFD:");
	for(i = 0; i < count; i++)
	{
		printk(" %02X", tmp[i]);
	}
	printk("\n");
#endif
/* OPPO 2012-08-13 liuhd Delete end */
	
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{

	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	filp->private_data = pn544_dev;

	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;
/*OPPO yuyi 2013-10-04 add begin for NFC_SMX when standby*/
	int ret = 0;
/*OPPO yuyi 2013-10-04 add end for NFC_SMX when standby*/
	switch (cmd) 
	{
	case PN544_SET_PWR:
		if (arg == 2) 
		{
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware\n", __func__);
/*OPPO yuyi 2013-10-04 add begin for NFC_SMX when standby*/
			ret = disable_irq_wake(pn544_dev->client->irq);
			if(ret < 0)
				{
					printk("%s,power on with firmware disable_irq_wake %d\n",__func__,ret);
				}	
/*OPPO yuyi 2013-10-04 add end for NFC_SMX when standby*/
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} 
		else if (arg == 1) 
		{
			/* power on */
/*OPPO yuyi 2013-10-04 add begin for NFC_SMX when standby*/
			ret = enable_irq_wake(pn544_dev->client->irq);
			if(ret < 0)
				{
					printk("%s,power on enable_irq_wake  %d\n",__func__,ret);
				}
/*OPPO yuyi 2013-10-04 add end for NFC_SMX when standby*/
			printk("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} 
		else  if (arg == 0) 
		{
			/* power off */
/*OPPO yuyi 2013-10-04 add begin for NFC_SMX when standby*/
			ret = disable_irq_wake(pn544_dev->client->irq);
			if(ret < 0)
				{
					printk("%s,power off disable_irq_wake %d\n",__func__,ret);
				}
/*OPPO yuyi 2013-10-04 add end for NFC_SMX when standby*/
			printk("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = 
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
};
/*OPPO yuyi 2014-09-19 add begin for nfc gpio MSM8916*/
static int pn544_parse_dts(struct device *dev, struct pn544_i2c_platform_data *nfc_data)
{
	int rc;
	struct device_node *np;
	np = dev->of_node;
	nfc_data->irq_gpio = of_get_named_gpio(np, "pn544,irq-gpio", 0);
	if( nfc_data->irq_gpio < 0 ){
		printk("nfc_data->irq_gpio not specified\n");	
	}
	
	nfc_data->ven_gpio = of_get_named_gpio(np, "pn544,enable-gpio", 0);
	if( nfc_data->ven_gpio < 0 ){
		printk("nfc_data->ven_gpio not specified\n");	
	}	
	
	nfc_data->firm_gpio = of_get_named_gpio(np, "pn544,update-gpio", 0);
	if( nfc_data->firm_gpio < 0 ){
		printk("nfc_data->firm_gpio not specified\n");	
	}	
	
	if( nfc_data->irq_gpio > 0){
		if( gpio_is_valid(nfc_data->irq_gpio) ){
			rc = gpio_request(nfc_data->irq_gpio, "nfc-irq-gpio");
			if(rc){
				printk("unable to request gpio [%d]\n", nfc_data->irq_gpio);
			}
		}
	}
	if( nfc_data->ven_gpio > 0){
		if( gpio_is_valid(nfc_data->ven_gpio) ){
			rc = gpio_request(nfc_data->ven_gpio, "nfc-ven-gpio");
			if(rc){
				printk("unable to request gpio [%d]\n", nfc_data->ven_gpio);
			}
		}
	}
	if( nfc_data->firm_gpio > 0){
		if( gpio_is_valid(nfc_data->firm_gpio) ){
			rc = gpio_request(nfc_data->firm_gpio, "nfc-firm-gpio");
			if(rc){
				printk("unable to request gpio [%d]\n", nfc_data->firm_gpio);
			}
		}
	}	
	return rc;
}
/*OPPO jiaoyifei 2014-09-19 add end for nfc gpio MSM8916*/

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev =NULL;
/*OPPO yuyi 2014-02-24 add begin for nfc pn65T*/
	struct clk*p65T_clk = NULL;
/*OPPO yuyi 2014-02-24 add begin for nfc pn65T*/
	
	if(1) {
		printk(" nfc  pn544_probe  yuyi\n");
	}

	//client->dev.platform_data = &nfc_pdata;

	
/* OPPO yuyi 2013-03-22  Add begin     from 12025 board-8064.c */
//	pn544_power_init();
/* OPPO yuyi  2013-03-22  Add end */

	platform_data = &nfc_pdata;//client->dev.platform_data;
   
/*OPPO yuyi 2013-10-24 add begin for nfc_devinfo*/
//	platform_data->clk_req_gpio = of_get_named_gpio(client->dev.of_node, "pn544,clk_req-gpio", 0);
//	printk("yuyi,pn544_parse_dt pdata->clk_req_gpio = %d\n",platform_data->clk_req_gpio );
	
	//register_device_proc("nfc", nfc_info.version, nfc_info.manufacture);
/*OPPO yuyi 2013-10-24 add end for nfc_devinfo*/
	//if (platform_data == NULL) 
	//{
	 //   printk(" nfc  pn544_probe  yuyi1\n");
	//	pr_err("%s : nfc probe fail\n", __func__);
	//	return  -ENODEV;
	//}
     pn544_parse_dts(&client->dev,platform_data);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

#if 0
	//IRQ 
	printk("yuyi,platform_data->irq_gpio %d \n",platform_data->irq_gpio);
	if(gpio_is_valid(platform_data->irq_gpio)) {
	printk("yuyi,irq gpio is valid");
	ret = gpio_request(20/*platform_data->irq_gpio*/, "nfc_irq_gpio");
	if (ret)
	{
		pr_err("gpio_nfc_int request error\n");
		return  -ENODEV;
	}
	}else {
		printk("irq gpio is invalid");
		}

	//VEN
	ret = gpio_request(platform_data->ven_gpio, "enable");
	if (ret)
	{
	    printk(" nfc  pn544_probe  yuyi4\n");
		pr_err("gpio_nfc_ven request error\n");
		return  -ENODEV;
	}

	//FIRM
	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret)
	{
		pr_err("gpio_nfc_firm request error\n");	
		return  -ENODEV;
	}
#endif
/*OPPO yuyi 2014-02-24 add begin for nfc pn65T*/	
/*	if ((gpio_is_valid(platform_data->clk_req_gpio))) {
		ret = gpio_request(platform_data->clk_req_gpio, "nfc_clk_req");
		if (ret)
		{
			pr_err("gpio_nfc_clk_req request error\n");	
			return  -ENODEV;
		}
*/		

/*OPPO yuyi 2014-05-24 modify begin for rename clock name from core_clk to cxo_out_d1*/
//#ifndef VENDOR_EDIT
//		p65T_clk  = clk_get(&client->dev, "core_clk");
//#else 
		p65T_clk  = clk_get(&client->dev, "ref_clk");
//#endif
/*OPPO yuyi 2014-05-24 modify end for rename clock name from core_clk to cxo_out_d1*/
		if (p65T_clk == NULL) {
			printk("yuyi,pn65T clk error!");
			goto err_exit;
		}

		ret = clk_prepare_enable(p65T_clk);
		printk("yuyi,clk_prepare_enable,return ret = %d",ret);
		if (ret){
			p65T_clk = NULL;
			clk_disable_unprepare(p65T_clk);
		}

/*
	} else {
		printk("yuyi,gpio_is_valid invalid\n");
	}
*/	
/*OPPO yuyi 2014-02-24 add end for nfc pn65T*/

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
//	pn544_dev->clk_req_gpio  = platform_data->clk_req_gpio;
	pn544_dev->client   = client;
	
/* OPPO 2012-07-11 liuhd Add begin for reason */
	ret = gpio_direction_input(pn544_dev->irq_gpio);
	if (ret < 0) {
		pr_err("%s :not able to set irq_gpio as input\n", __func__);
		goto err_exit;
	}
	if (platform_data->firm_gpio) {
		ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set firm_gpio as output\n",
				 __func__);
			goto err_exit;
		}
	}
	ret = gpio_direction_output(pn544_dev->ven_gpio, 1);
	if (ret < 0) {
		pr_err("%s : not able to set ven_gpio as output\n", __func__);
		goto err_exit;
	}

/*		ret = gpio_direction_input(pn544_dev->clk_req_gpio);
		if (ret < 0) {
			pr_err("%s : not able to set ven_gpio as input\n", __func__);
//			gpio_free(platform_data->clk_req_gpio);
		}
	*/	
/* OPPO 2012-07-11 liuhd Add end */
			printk(" nfc  pn544_probe  yuyi14\n");
	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) 
	{
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) 
	{
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);

	#ifdef CONFIG_VENDOR_EDIT
	/*liuhd add for sleep current because of nfc  2013-12-17*/
	gpio_set_value(pn544_dev->ven_gpio, 1);
	msleep(10);
      gpio_set_value(pn544_dev->ven_gpio, 0);
	#endif
	/*add end by liuhd 2013-12-17*/
	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
//	gpio_free(pn544_dev->clk_req_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}
//#ifdef CONFIG_OF
static struct of_device_id pn544_of_match_table[] = {
	{ .compatible = "pn544,nxp-nfc",},
	{ },
};
//#else
//#define pn544_of_match_table NULL
//#endif

static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= 
	{
		.owner	= THIS_MODULE,
		.name	= "pn544",
		.of_match_table = pn544_of_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
