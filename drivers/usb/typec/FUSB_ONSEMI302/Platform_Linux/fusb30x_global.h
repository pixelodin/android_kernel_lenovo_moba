/*
* File:   fusb30x_global.h
* Company: Fairchild Semiconductor
*
* Created on September 11, 2015, 15:28 AM
*/

#ifndef FUSB30X_TYPES_H
#define FUSB30X_TYPES_H

#include <linux/i2c.h>                              // i2c_client, spinlock_t
#include <linux/hrtimer.h>                          // hrtimer
#include <linux/pm_wakeup.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/extcon.h>
#include <linux/pinctrl/consumer.h>
#include "FSCTypes.h"                               // FUSB30x custom types

#include "../core/Port.h"                                   // Port object
//#include "../core/modules/dpm.h"
#include <linux/usb/typec.h>
#include <linux/power_supply.h>
#include "../../../pd/usbpd.h"
#include <linux/debugfs.h>

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#endif

#define FSC_HOSTCOMM_BUFFER_SIZE    64              // Length of the hostcomm buffer

struct fusb30x_chip                                 // Contains data required by this driver
{
    struct mutex lock;                              // Synchronization lock
	struct mutex thread_lock;
	struct wakeup_source fusb302_wakelock;          // wake lock
    struct semaphore suspend_lock;

    FSC_U8 dbgTimerTicks;                           // Count of timer ticks
    FSC_U8 dbgTimerRollovers;                       // Timer tick counter rollover counter
    FSC_U8 dbgSMTicks;                              // Count of state machine ticks
    FSC_U8 dbgSMRollovers;                          // State machine tick counter rollover counter
    FSC_S32 dbg_gpio_StateMachine;                  // Gpio that toggles every time the state machine is triggered
    FSC_BOOL dbg_gpio_StateMachine_value;           // Value of sm toggle state machine
    char HostCommBuf[FSC_HOSTCOMM_BUFFER_SIZE];     // Buffer used to communicate with HostComm

    struct dentry *debugfs_parent;                  // Parent for DebugFS entry

    /* Internal config data */
    FSC_S32 InitDelayMS;                            // Number of milliseconds to wait before initializing the fusb30x
    FSC_S32 numRetriesI2C;                          // Number of times to retry I2C reads/writes

    /* I2C */
    struct i2c_client* client;                      // I2C client provided by kernel
    FSC_BOOL use_i2c_blocks;                        // True if I2C_FUNC_SMBUS_I2C_BLOCK is supported

    /* GPIO */
    FSC_S32 gpio_VBus5V;                            // VBus 5V GPIO pin
    FSC_BOOL gpio_VBus5V_value;                     // true if active, false otherwise
    FSC_S32 gpio_VBusOther;                         // VBus other GPIO pin (eg. VBus 12V) (NOTE: Optional feature - if set to <0 during GPIO init, then feature is disabled)
    FSC_BOOL gpio_VBusOther_value;                  // true if active, false otherwise
    FSC_S32 gpio_IntN;                              // INT_N GPIO pin

    FSC_S32 gpio_OTG;                              //usb2 otg GPIO pin
    FSC_S32 charge_2t1_gpio;                              //usb2 charge GPIO90 pin
    FSC_S32 charge_2t2_gpio;                              //usb2 charge GPIO74 pin
    FSC_S32 gpio_IntN_irq;                          // IRQ assigned to INT_N GPIO pin
    FSC_S32 gpio_Discharge;                         // Discharge GPIO pin
    FSC_BOOL gpio_Discharge_value;                  // true if active, false otherwise

    /* Threads */
    struct work_struct sm_worker;                   // Main state machine actions
    struct workqueue_struct *highpri_wq;
    FSC_BOOL queued;

    /* Timers */
    struct hrtimer sm_timer;                        // High-resolution timer for the state machine
    struct delayed_work start_usb_peripheral_work;

    /* Port Object */
    struct Port port;
    DevicePolicyPtr_t dpm;
    struct extcon_dev	*extcon;
    FSC_BOOL	is_vbus_present;

    struct typec_capability	typec_caps;
    struct typec_port	*typec_port;
    struct typec_partner	*partner;
    struct typec_partner_desc partner_desc;
    struct usb_pd_identity	partner_identity;
#ifdef SUPPORT_ONSEMI_PDCONTROL
	struct power_supply  *usb_psy;
    enum data_role		current_dr;
    enum power_role		current_pr;
    enum power_supply_typec_mode typec_mode;
#endif

#ifdef CONFIG_PINCTRL
    struct pinctrl	*pinctrl_int;
    struct pinctrl_state	*pinctrl_state_int;
#endif
};

extern struct fusb30x_chip* g_chip;

struct fusb30x_chip* fusb30x_GetChip(void);         // Getter for the global chip structure
void fusb30x_SetChip(struct fusb30x_chip* newChip); // Setter for the global chip structure

#endif /* FUSB30X_TYPES_H */
