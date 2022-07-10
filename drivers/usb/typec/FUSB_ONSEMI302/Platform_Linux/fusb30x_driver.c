/*
 * File:   fusb30x_driver.c
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality
#include <linux/extcon.h>
#include <linux/extcon-provider.h>

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#include "dfs.h"
#include "fusb30x_driver.h"

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#include <linux/power_supply.h>
#include "pps_example.h"
#include "../core/TypeC.h"
#include <linux/delay.h>

struct charger_object* fusb_fetch_pdo_chip = NULL;
struct fusb30x_chip* fusb_pps_chip = NULL;
#endif
/******************************************************************************
* Driver functions
******************************************************************************/
int fusb_init_fail = 0;
static int __init fusb30x_init(void)
{
    pr_info("FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
    pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb302_i2c_resume(struct device* dev)
{
    struct fusb30x_chip *chip;
        struct i2c_client *client = to_i2c_client(dev);

        if (client) {
            chip = i2c_get_clientdata(client);
                if (chip)
                up(&chip->suspend_lock);
        }
     return 0;
}

static int fusb302_i2c_suspend(struct device* dev)
{
    struct fusb30x_chip* chip;
        struct i2c_client* client =  to_i2c_client(dev);

        if (client) {
             chip = i2c_get_clientdata(client);
                 if (chip)
                    down(&chip->suspend_lock);
        }
        return 0;
}

static const unsigned int usbpd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static int fusb30x_probe (struct i2c_client* client,
                          const struct i2c_device_id* id)
{
    int ret = 0;
    struct fusb30x_chip* chip;
    struct i2c_adapter* adapter;
#ifdef SUPPORT_ONSEMI_PDCONTROL
	struct charger_object* cc_chip;
#endif

	pr_info("FUSB - %s\n", __func__);

    if (!client) {
        pr_err("FUSB  %s - Error: Client structure is NULL!\n", __func__);
        return -EINVAL;
    }
    dev_info(&client->dev, "%s\n", __func__);

    /* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb30x_dt_match, &client->dev)) {
		dev_err(&client->dev,
			"FUSB  %s - Error: Device tree mismatch!\n",
			__func__);
		return -EINVAL;
	}
    pr_debug("FUSB  %s - Device tree matched!\n", __func__);

    /* Allocate space for our chip structure (devm_* is managed by the device) */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);

#ifdef SUPPORT_ONSEMI_PDCONTROL
	cc_chip = devm_kzalloc(&client->dev, sizeof(*cc_chip), GFP_KERNEL);
	if (cc_chip == NULL){
		pr_err("FUSB302X driver:cc_chip is null\n");
	}
#endif

    if (!chip)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to allocate memory for g_chip!\n", __func__);
		return -ENOMEM;
	}

#ifdef SUPPORT_ONSEMI_PDCONTROL
	fusb_pps_chip = chip;
#endif

    chip->client = client;                                                      // Assign our client handle to our chip
    fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory
    pr_debug("FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

#ifdef SUPPORT_ONSEMI_PDCONTROL
	fusb30x_charger_SetChip(cc_chip);
	fusb_fetch_pdo_chip = cc_chip;
#endif

    /* Initialize the chip lock */
    mutex_init(&chip->lock);
#ifdef SUPPORT_ONSEMI_PDCONTROL
	mutex_init(&chip->port.thread_lock);
	mutex_init(&chip->thread_lock);
#endif
    /* Initialize the chip's data members */
    fusb_InitChipData();
    pr_debug("FUSB  %s - Chip struct data initialized!\n", __func__);
	/* Add QRD extcon */
	chip->extcon = devm_extcon_dev_allocate(&client->dev, usbpd_extcon_cable);
	if (!chip->extcon) {
		dev_err(&client->dev,
			"FUSB %s - Error: Unable to allocate memory for extcon!\n",
			__func__);
		return -ENOMEM;
	}
	ret = devm_extcon_dev_register(&client->dev, chip->extcon);
	if (ret) {
		dev_err(&client->dev, "FUSB failed to register extcon device\n");
		return -1;
	}
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_SS);
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_SS);

	fusb_init_event_handler();

    /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
    adapter = to_i2c_adapter(client->dev.parent);
    if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
    {
        chip->use_i2c_blocks = true;
    }
    else
    {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
        dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
        if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
        {
            dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
            dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
            return -EIO;
        }
    }
    pr_debug("FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

    /* Assign our struct as the client's driverdata */
    i2c_set_clientdata(client, chip);
    pr_debug("FUSB  %s - I2C client data set!\n", __func__);

    /* Verify that our device exists and that it's what we expect */
    if (!fusb_IsDeviceValid())
    {
        fusb_init_fail = 1;
        dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
        return -EIO;
    }
    pr_debug("FUSB  %s - Device check passed!\n", __func__);

    /* reset fusb302*/
    fusb_reset();

    /* Initialize semaphore*/
    sema_init(&chip->suspend_lock, 1);

    /* Initialize the platform's GPIO pins and IRQ */
    ret = fusb_InitializeGPIO();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
        return ret;
    }
    pr_debug("FUSB  %s - GPIO initialized!\n", __func__);

    /* Initialize sysfs file accessors */
    fusb_Sysfs_Init();
    pr_debug("FUSB  %s - Sysfs nodes created!\n", __func__);

    /* Initialize debugfs file accessors */
    fusb_DFS_Init();
    pr_debug("FUSB  %s - DebugFS nodes created!\n", __func__);

    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    pr_debug("FUSB  %s - Core is initialized!\n", __func__);

    dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);
	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
    pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);

    /* Remove debugfs file accessors */
    fusb_DFS_Cleanup();
    pr_debug("FUSB  %s - DebugFS nodes removed.\n", __func__);

    fusb_GPIO_Cleanup();
    pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
    return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
	FSC_U8 reset = 0x01; /* regaddr is 0x01 */
	FSC_U8 data = 0x40; /* data is 0x40 */
	FSC_U8 length = 0x01; /* length is 0x01 */
	FSC_BOOL ret = 0;
	struct fusb30x_chip *chip = fusb30x_GetChip();

	if (!chip) {
		pr_err("FUSB shutdown - Chip structure is NULL!\n");
		return;
	}

	core_enable_typec(&chip->port, false);
	DeviceWrite(chip->port.I2cAddr, regControl3, length, &data);
	SetStateUnattached(&chip->port);

	/* Enable the pull-up on CC1 */
	chip->port.Registers.Switches.PU_EN1 = 1;
	/* Disable the pull-down on CC1 */
	chip->port.Registers.Switches.PDWN1 = 0;
	/* Enable the pull-up on CC2 */
	chip->port.Registers.Switches.PU_EN2 = 1;
	/* Disable the pull-down on CC2 */
	chip->port.Registers.Switches.PDWN2 = 0;
	/* Commit the switch state */
	DeviceWrite(chip->port.I2cAddr, regSwitches0, 1,
		&chip->port.Registers.Switches.byte[0]);
	fusb_GPIO_Cleanup();
	/* keep the cc open status 20ms */
	mdelay(20);
	ret = fusb_I2C_WriteData((FSC_U8)regReset, length, &reset);
	if (ret != 0)
		pr_err("device Reset failed, ret = %d\n", ret);
	pr_info("FUSB shutdown - FUSB30x device shutdown!\n");
}


/*******************************************************************************
 * Driver macros
 ******************************************************************************/
module_init(fusb30x_init);                                                      // Defines the module's entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Fairchild");                        								// Exposed on call to modinfo
