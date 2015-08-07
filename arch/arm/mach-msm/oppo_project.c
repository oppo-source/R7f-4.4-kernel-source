
#include "board-dt.h"
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h> 
//#include <mach/msm_smem.h>
#include <linux/fs.h>
#include <soc/qcom/smem.h>
#include <mach/oppo_project.h>
//#include <mach/oppo_reserve3.h>

/////////////////////////////////////////////////////////////
static struct proc_dir_entry *oppoVersion = NULL;
static ProjectInfoCDTType *format = NULL;



unsigned int init_project_version(void)
{	
	unsigned int len = (sizeof(ProjectInfoCDTType) + 3)&(~0x3);

	format = (ProjectInfoCDTType *)smem_alloc(SMEM_PROJECT,len,0,0);

	if(format)
		return format->nProject;
	
	return 0;
}


unsigned int get_project(void)
{
	if(format)
		return format->nProject;
	return 0;
}

unsigned int is_project(OPPO_PROJECT project )
{
	return (get_project() == project?1:0);
}

unsigned char get_PCB_Version(void)
{
	if(format)
		return format->nPCBVersion;
	return 0;
}

unsigned char get_Modem_Version(void)
{
	if(format)
		return format->nModem;
	return 0;
}

unsigned char get_Operator_Version(void)
{
	if(format)
		return format->nOperator;
	return 0;
}


//this module just init for creat files to show which version
static int prjVersion_read_proc(struct file *file, char __user *buf, 
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	
	#if 0
	unsigned char operator_version;

	operator_version = get_Operator_Version();
	if(is_project(OPPO_14005)){
		if(operator_version == 3)
			len = sprintf(page,"%d",14006);
		else 
			len = sprintf(page,"%d",14005);
    }
	else if(is_project(OPPO_14023)){
		if(operator_version == 3)
			len = sprintf(page,"%d",14023);
		else 
			len = sprintf(page,"%d",14023);
    }
	else if(is_project(OPPO_14045)){
        //#ifdef VENDOR_EDIT
        //lile@EXP.BaseDrv.Audio, 2014-08-07 add for EXP to mark 14085
		if(operator_version == 3 || 5 == operator_version || 6 == operator_version || 7 == operator_version)
        //#endif VENDOR_EDIT
			len = sprintf(page,"%d",14046);
		else if(operator_version == 4)
			len = sprintf(page,"%d",14047);
		else 
			len = sprintf(page,"%d",14045);
    }
	else
	{
		len = sprintf(page,"%d",get_project());
	}
	#else
	len = sprintf(page,"%d",get_project());
	#endif

	if(len > *off)
		len -= *off;
	else
		len = 0;
	
	if(copy_to_user(buf,page,(len < count ? len : count))){
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations prjVersion_proc_fops = {
	.read = prjVersion_read_proc,
};


static int pcbVersion_read_proc(struct file *file, char __user *buf, 
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_PCB_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

struct file_operations pcbVersion_proc_fops = {
	.read = pcbVersion_read_proc,
};


static int operatorName_read_proc(struct file *file, char __user *buf, 
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Operator_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

struct file_operations operatorName_proc_fops = {
	.read = operatorName_read_proc,
};



static int modemType_read_proc(struct file *file, char __user *buf, 
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Modem_Version());

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

struct file_operations modemType_proc_fops = {
	.read = modemType_read_proc,
};

static int __init oppo_project_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;
	
	oppoVersion =  proc_mkdir("oppoVersion", NULL);
	if(!oppoVersion) {
		pr_err("can't create oppoVersion proc\n");
		ret = -ENOENT;
	}

	pentry = proc_create("prjVersion", S_IRUGO, oppoVersion, &prjVersion_proc_fops);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
		return -ENOENT;
	}
	pentry = proc_create("pcbVersion", S_IRUGO, oppoVersion, &pcbVersion_proc_fops);
	if(!pentry) {
		pr_err("create pcbVersion proc failed.\n");
		return -ENOENT;
	}
	pentry = proc_create("operatorName", S_IRUGO, oppoVersion, &operatorName_proc_fops);
	if(!pentry) {
		pr_err("create operatorName proc failed.\n");
		return -ENOENT;
	}
	pentry = proc_create("modemType", S_IRUGO, oppoVersion, &modemType_proc_fops);
	if(!pentry) {
		pr_err("create modemType proc failed.\n");
		return -ENOENT;
	}
	return ret;
}

static void __exit oppo_project_init_exit(void)
{

	remove_proc_entry("oppoVersion", NULL);

}

module_init(oppo_project_init);
module_exit(oppo_project_init_exit);


MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");
