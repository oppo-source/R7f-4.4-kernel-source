#ifndef __LINUX_KEYBOARD_H
#define __LINUX_KEYBOARD_H

#include <uapi/linux/keyboard.h>
//#ifdef VENDOR_EDIT
/*geyixue@bsp.drv   add for QCOM patch  in 20141115*/
#include <linux/input.h>
struct notifier_block;
//#endif
struct notifier_block;
extern unsigned short *key_maps[MAX_NR_KEYMAPS];
//#ifdef VENDOR_EDIT
/*geyixue@bsp.drv   add for QCOM patch  in 20141115*/
//extern unsigned short plain_map[NR_KEYS];
extern unsigned short plain_map[KEY_CNT];
//#endif
struct keyboard_notifier_param {
	struct vc_data *vc;	/* VC on which the keyboard press was done */
	int down;		/* Pressure of the key? */
	int shift;		/* Current shift mask */
	int ledstate;		/* Current led state */
	unsigned int value;	/* keycode, unicode value or keysym */
};

extern int register_keyboard_notifier(struct notifier_block *nb);
extern int unregister_keyboard_notifier(struct notifier_block *nb);
#endif
