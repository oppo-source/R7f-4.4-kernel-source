/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <asm/mach/arch.h>
#include <soc/qcom/socinfo.h>
#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include "board-dt.h"
#include "platsmp.h"

#ifdef VENDOR_EDIT //Yixue.ge@ProDrv.BL add for ftm 2014-01-04
#include <mach/oppo_project.h>
#include <mach/oppo_boot_mode.h>
static struct kobject *systeminfo_kobj;
#endif

#ifdef VENDOR_EDIT
/* OPPO 2013.07.09 hewei add begin for factory mode*/
#include <linux/gpio.h>
static struct kobject *systeminfo_kobj;
#if 0
enum{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
	
};
#endif

static int ftm_mode = MSM_BOOT_MODE__NORMAL;

int __init  board_mfg_mode_init(void)
{	
    char *substr;

    substr = strstr(boot_command_line, "oppo_ftm_mode=");
    if(substr) {
        substr += strlen("oppo_ftm_mode=");

        if(strncmp(substr, "factory2", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substr, "ftmwifi", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__WLAN;
		else if(strncmp(substr, "ftmmos", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__MOS;
        else if(strncmp(substr, "ftmrf", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substr, "ftmrecovery", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RECOVERY;
    } 	

	pr_err("board_mfg_mode_init, " "ftm_mode=%d\n", ftm_mode);
	
	return 0;

}
//__setup("oppo_ftm_mode=", board_mfg_mode_init);

int get_boot_mode(void)
{
	return ftm_mode;
}

static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", ftm_mode);
}

struct kobj_attribute ftmmode_attr = {
    .attr = {"ftmmode", 0644},

    .show = &ftmmode_show,
};


/* OPPO 2013-01-04 Van add start for ftm close modem*/
#define mdm_drv_ap2mdm_pmic_pwr_en_gpio  27

static ssize_t closemodem_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	//writing '1' to close and '0' to open
	//pr_err("closemodem buf[0] = 0x%x",buf[0]);
	switch (buf[0]) {
	case 0x30:
		break;
	case 0x31:
	//	pr_err("closemodem now");
		gpio_direction_output(mdm_drv_ap2mdm_pmic_pwr_en_gpio, 0);
		mdelay(4000);
		break;
	default:
		break;
	}

	return count;
}

struct kobj_attribute closemodem_attr = {
  .attr = {"closemodem", 0644},
  //.show = &closemodem_show,
  .store = &closemodem_store
};
/* OPPO 2013-01-04 Van add end for ftm close modem*/
static struct attribute * g[] = {
	&ftmmode_attr.attr,
/* OPPO 2013-01-04 Van add start for ftm close modem*/
	&closemodem_attr.attr,
/* OPPO 2013-01-04 Van add end for ftm close modem*/
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};
/* OPPO 2013.07.09 hewei add end for factory modes*/
#endif // VENDOR_EDIT

#ifdef VENDOR_EDIT
/* OPPO 2013-09-03 heiwei add for add interface start reason and boot_mode begin */
char pwron_event[16];
static int __init start_reason_init(void)
{
    int i;
    char * substr = strstr(boot_command_line, "androidboot.startupmode="); 
    if(NULL == substr)
		return 0;
    substr += strlen("androidboot.startupmode=");
    for(i=0; substr[i] != ' '; i++) {
        pwron_event[i] = substr[i];
    }
    pwron_event[i] = '\0';
    
    printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);
	
	return 1;
}
//__setup("androidboot.startupmode=", start_reason_setup);

char boot_mode[16];
static int __init boot_mode_init(void)
{
    int i;
    char *substr = strstr(boot_command_line, "androidboot.mode=");
	
    if(NULL == substr)
	return 0;

    substr += strlen("androidboot.mode=");
	
    for(i=0; substr[i] != ' '; i++) {
        boot_mode[i] = substr[i];
    }
    boot_mode[i] = '\0';

    printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
    return 1;
}
//__setup("androidboot.mode=", boot_mode_setup);
/* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode end */
#endif //VENDOR_EDIT

static void __init msm8916_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8916_map_io(void)
{
	msm_map_msm8916_io();
}

static struct of_dev_auxdata msm8916_auxdata_lookup[] __initdata = {
	{}
};

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8916_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
}

static void __init msm8916_init(void)
{
#ifdef VENDOR_EDIT
	/* OPPO 2013.07.09 hewei add begin for FTM */
	int rc = 0;
	/* OPPO 2013.07.09 hewei add end for FTM */
#endif //VENDOR_EDIT
	struct of_dev_auxdata *adata = msm8916_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

#ifdef VENDOR_EDIT		
	/* OPPO 2013.07.09 hewei add begin for factory mode*/
	board_mfg_mode_init();
	/* OPPO 2013.07.09 hewei add end */
#endif //VENDOR_EDIT

#ifdef VENDOR_EDIT
/* OPPO 2013-09-03 heiwei add for add interface start reason and boot_mode begin */
    start_reason_init();
    boot_mode_init();
/* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode end */
#endif //VENDOR_EDIT

	msm8916_add_drivers();
	#ifdef VENDOR_EDIT //Yixue.ge@ProDrv.BL add for ftm 2014-01-04
	init_project_version();
	#endif
#ifdef VENDOR_EDIT	
	/* OPPO 2013.07.09 hewei add begin for factory mode*/
	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	printk("songxh create systeminto node suscess!\n");
	if (systeminfo_kobj)
		rc = sysfs_create_group(systeminfo_kobj, &attr_group);
	/* OPPO 2013.07.09 hewei add end */
#endif //VENDOR_EDIT	

}

static const char *msm8916_dt_match[] __initconst = {
	"qcom,msm8916",
	"qcom,apq8016",
	NULL
};

static const char *msm8936_dt_match[] __initconst = {
	"qcom,msm8936",
	NULL
};

static const char *msm8939_dt_match[] __initconst = {
	"qcom,msm8939",
	NULL
};

static const char *msm8929_dt_match[] __initconst = {
	"qcom,msm8929",
	NULL
};

static const char *msm8939_bc_dt_match[] __initconst = {
	"qcom,msm8939_bc",
	NULL
};

DT_MACHINE_START(MSM8916_DT,
		"Qualcomm Technologies, Inc. MSM 8916 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8916_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8916_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8939_DT,
		"Qualcomm Technologies, Inc. MSM 8939 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8939_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8936_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8936_DT,
		"Qualcomm Technologies, Inc. MSM 8936 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8936_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8936_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8929_DT,
	"Qualcomm Technologies, Inc. MSM 8929 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8929_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8936_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8939_BC_DT,
	"Qualcomm Technologies, Inc. MSM 8939_BC (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8939_bc_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8936_smp_ops,
MACHINE_END
