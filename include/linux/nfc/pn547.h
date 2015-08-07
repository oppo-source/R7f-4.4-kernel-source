/***********************************************************
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: - pn544.h
* Description: Head file for pn544.
*           To define pn544 array and register address.
** Version: 1.0
** Date : 2013/10/15	
** Author: yuyi@Independent Group
** 
****************************************************************/


//ALERT:please relocate pn544.h under .\kernel\include\linux

#define PN547_MAGIC	0xE9

/*
 * PN547 power control via ioctl
 * PN547_SET_PWR(0): power off
 * PN547_SET_PWR(1): power on
 * PN547_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN547_SET_PWR	_IOW(PN547_MAGIC, 0x01, unsigned int)


