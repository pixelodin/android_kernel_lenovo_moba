// SPDX-License-Identifier: GPL-2.0
/**
 * USB Type-C Multiplexer/DeMultiplexer Switch support
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *         Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
<<<<<<< HEAD
#include <linux/usb/typec_mux.h>

static DEFINE_MUTEX(switch_lock);
static DEFINE_MUTEX(mux_lock);
static LIST_HEAD(switch_list);
static LIST_HEAD(mux_list);
=======
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/usb/typec_mux.h>

#include "bus.h"

static int name_match(struct device *dev, const void *name)
{
	return !strcmp((const char *)name, dev_name(dev));
}

static bool dev_name_ends_with(struct device *dev, const char *suffix)
{
	const char *name = dev_name(dev);
	const int name_len = strlen(name);
	const int suffix_len = strlen(suffix);

	if (suffix_len > name_len)
		return false;

	return strcmp(name + (name_len - suffix_len), suffix) == 0;
}

static int switch_fwnode_match(struct device *dev, const void *fwnode)
{
	return dev_fwnode(dev) == fwnode && dev_name_ends_with(dev, "-switch");
}
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

static void *typec_switch_match(struct device_connection *con, int ep,
				void *data)
{
<<<<<<< HEAD
	struct typec_switch *sw;

	list_for_each_entry(sw, &switch_list, entry)
		if (!strcmp(con->endpoint[ep], dev_name(sw->dev)))
			return sw;

	/*
	 * We only get called if a connection was found, tell the caller to
	 * wait for the switch to show up.
	 */
	return ERR_PTR(-EPROBE_DEFER);
=======
	struct device *dev;

	if (con->fwnode) {
		if (con->id && !fwnode_property_present(con->fwnode, con->id))
			return NULL;

		dev = class_find_device(&typec_mux_class, NULL, con->fwnode,
					switch_fwnode_match);
	} else {
		dev = class_find_device(&typec_mux_class, NULL,
					con->endpoint[ep], name_match);
	}

	return dev ? to_typec_switch(dev) : ERR_PTR(-EPROBE_DEFER);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

/**
 * typec_switch_get - Find USB Type-C orientation switch
 * @dev: The caller device
 *
 * Finds a switch linked with @dev. Returns a reference to the switch on
 * success, NULL if no matching connection was found, or
 * ERR_PTR(-EPROBE_DEFER) when a connection was found but the switch
 * has not been enumerated yet.
 */
struct typec_switch *typec_switch_get(struct device *dev)
{
	struct typec_switch *sw;

<<<<<<< HEAD
	mutex_lock(&switch_lock);
	sw = device_connection_find_match(dev, "typec-switch", NULL,
					  typec_switch_match);
	if (!IS_ERR_OR_NULL(sw)) {
		WARN_ON(!try_module_get(sw->dev->driver->owner));
		get_device(sw->dev);
	}
	mutex_unlock(&switch_lock);
=======
	sw = device_connection_find_match(dev, "orientation-switch", NULL,
					  typec_switch_match);
	if (!IS_ERR_OR_NULL(sw))
		WARN_ON(!try_module_get(sw->dev.parent->driver->owner));
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	return sw;
}
EXPORT_SYMBOL_GPL(typec_switch_get);

/**
 * typec_put_switch - Release USB Type-C orientation switch
 * @sw: USB Type-C orientation switch
 *
 * Decrement reference count for @sw.
 */
void typec_switch_put(struct typec_switch *sw)
{
	if (!IS_ERR_OR_NULL(sw)) {
<<<<<<< HEAD
		module_put(sw->dev->driver->owner);
		put_device(sw->dev);
=======
		module_put(sw->dev.parent->driver->owner);
		put_device(&sw->dev);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	}
}
EXPORT_SYMBOL_GPL(typec_switch_put);

<<<<<<< HEAD
/**
 * typec_switch_register - Register USB Type-C orientation switch
 * @sw: USB Type-C orientation switch
=======
static void typec_switch_release(struct device *dev)
{
	kfree(to_typec_switch(dev));
}

static const struct device_type typec_switch_dev_type = {
	.name = "orientation_switch",
	.release = typec_switch_release,
};

/**
 * typec_switch_register - Register USB Type-C orientation switch
 * @parent: Parent device
 * @desc: Orientation switch description
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 *
 * This function registers a switch that can be used for routing the correct
 * data pairs depending on the cable plug orientation from the USB Type-C
 * connector to the USB controllers. USB Type-C plugs can be inserted
 * right-side-up or upside-down.
 */
<<<<<<< HEAD
int typec_switch_register(struct typec_switch *sw)
{
	mutex_lock(&switch_lock);
	list_add_tail(&sw->entry, &switch_list);
	mutex_unlock(&switch_lock);

	return 0;
=======
struct typec_switch *
typec_switch_register(struct device *parent,
		      const struct typec_switch_desc *desc)
{
	struct typec_switch *sw;
	int ret;

	if (!desc || !desc->set)
		return ERR_PTR(-EINVAL);

	sw = kzalloc(sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return ERR_PTR(-ENOMEM);

	sw->set = desc->set;

	device_initialize(&sw->dev);
	sw->dev.parent = parent;
	sw->dev.fwnode = desc->fwnode;
	sw->dev.class = &typec_mux_class;
	sw->dev.type = &typec_switch_dev_type;
	sw->dev.driver_data = desc->drvdata;
	dev_set_name(&sw->dev, "%s-switch", dev_name(parent));

	ret = device_add(&sw->dev);
	if (ret) {
		dev_err(parent, "failed to register switch (%d)\n", ret);
		put_device(&sw->dev);
		return ERR_PTR(ret);
	}

	return sw;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}
EXPORT_SYMBOL_GPL(typec_switch_register);

/**
 * typec_switch_unregister - Unregister USB Type-C orientation switch
 * @sw: USB Type-C orientation switch
 *
 * Unregister switch that was registered with typec_switch_register().
 */
void typec_switch_unregister(struct typec_switch *sw)
{
<<<<<<< HEAD
	mutex_lock(&switch_lock);
	list_del(&sw->entry);
	mutex_unlock(&switch_lock);
}
EXPORT_SYMBOL_GPL(typec_switch_unregister);

/* ------------------------------------------------------------------------- */

static void *typec_mux_match(struct device_connection *con, int ep, void *data)
{
	struct typec_mux *mux;

	list_for_each_entry(mux, &mux_list, entry)
		if (!strcmp(con->endpoint[ep], dev_name(mux->dev)))
			return mux;

	/*
	 * We only get called if a connection was found, tell the caller to
	 * wait for the switch to show up.
	 */
	return ERR_PTR(-EPROBE_DEFER);
=======
	if (!IS_ERR_OR_NULL(sw))
		device_unregister(&sw->dev);
}
EXPORT_SYMBOL_GPL(typec_switch_unregister);

void typec_switch_set_drvdata(struct typec_switch *sw, void *data)
{
	dev_set_drvdata(&sw->dev, data);
}
EXPORT_SYMBOL_GPL(typec_switch_set_drvdata);

void *typec_switch_get_drvdata(struct typec_switch *sw)
{
	return dev_get_drvdata(&sw->dev);
}
EXPORT_SYMBOL_GPL(typec_switch_get_drvdata);

/* ------------------------------------------------------------------------- */

static int mux_fwnode_match(struct device *dev, const void *fwnode)
{
	return dev_fwnode(dev) == fwnode && dev_name_ends_with(dev, "-mux");
}

static void *typec_mux_match(struct device_connection *con, int ep, void *data)
{
	const struct typec_altmode_desc *desc = data;
	struct device *dev;
	bool match;
	int nval;
	u16 *val;
	int i;

	if (!con->fwnode) {
		dev = class_find_device(&typec_mux_class, NULL,
					con->endpoint[ep], name_match);

		return dev ? to_typec_switch(dev) : ERR_PTR(-EPROBE_DEFER);
	}

	/*
	 * Check has the identifier already been "consumed". If it
	 * has, no need to do any extra connection identification.
	 */
	match = !con->id;
	if (match)
		goto find_mux;

	/* Accessory Mode muxes */
	if (!desc) {
		match = fwnode_property_present(con->fwnode, "accessory");
		if (match)
			goto find_mux;
		return NULL;
	}

	/* Alternate Mode muxes */
	nval = fwnode_property_count_u16(con->fwnode, "svid");
	if (nval <= 0)
		return NULL;

	val = kcalloc(nval, sizeof(*val), GFP_KERNEL);
	if (!val)
		return ERR_PTR(-ENOMEM);

	nval = fwnode_property_read_u16_array(con->fwnode, "svid", val, nval);
	if (nval < 0) {
		kfree(val);
		return ERR_PTR(nval);
	}

	for (i = 0; i < nval; i++) {
		match = val[i] == desc->svid;
		if (match) {
			kfree(val);
			goto find_mux;
		}
	}
	kfree(val);
	return NULL;

find_mux:
	dev = class_find_device(&typec_mux_class, NULL, con->fwnode,
				mux_fwnode_match);

	return dev ? to_typec_switch(dev) : ERR_PTR(-EPROBE_DEFER);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

/**
 * typec_mux_get - Find USB Type-C Multiplexer
 * @dev: The caller device
<<<<<<< HEAD
 * @name: Mux identifier
=======
 * @desc: Alt Mode description
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 *
 * Finds a mux linked to the caller. This function is primarily meant for the
 * Type-C drivers. Returns a reference to the mux on success, NULL if no
 * matching connection was found, or ERR_PTR(-EPROBE_DEFER) when a connection
 * was found but the mux has not been enumerated yet.
 */
<<<<<<< HEAD
struct typec_mux *typec_mux_get(struct device *dev, const char *name)
{
	struct typec_mux *mux;

	mutex_lock(&mux_lock);
	mux = device_connection_find_match(dev, name, NULL, typec_mux_match);
	if (!IS_ERR_OR_NULL(mux)) {
		WARN_ON(!try_module_get(mux->dev->driver->owner));
		get_device(mux->dev);
	}
	mutex_unlock(&mux_lock);
=======
struct typec_mux *typec_mux_get(struct device *dev,
				const struct typec_altmode_desc *desc)
{
	struct typec_mux *mux;

	mux = device_connection_find_match(dev, "mode-switch", (void *)desc,
					   typec_mux_match);
	if (!IS_ERR_OR_NULL(mux))
		WARN_ON(!try_module_get(mux->dev.parent->driver->owner));
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	return mux;
}
EXPORT_SYMBOL_GPL(typec_mux_get);

/**
 * typec_mux_put - Release handle to a Multiplexer
 * @mux: USB Type-C Connector Multiplexer/DeMultiplexer
 *
 * Decrements reference count for @mux.
 */
void typec_mux_put(struct typec_mux *mux)
{
	if (!IS_ERR_OR_NULL(mux)) {
<<<<<<< HEAD
		module_put(mux->dev->driver->owner);
		put_device(mux->dev);
=======
		module_put(mux->dev.parent->driver->owner);
		put_device(&mux->dev);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	}
}
EXPORT_SYMBOL_GPL(typec_mux_put);

<<<<<<< HEAD
/**
 * typec_mux_register - Register Multiplexer routing USB Type-C pins
 * @mux: USB Type-C Connector Multiplexer/DeMultiplexer
=======
static void typec_mux_release(struct device *dev)
{
	kfree(to_typec_mux(dev));
}

static const struct device_type typec_mux_dev_type = {
	.name = "mode_switch",
	.release = typec_mux_release,
};

/**
 * typec_mux_register - Register Multiplexer routing USB Type-C pins
 * @parent: Parent device
 * @desc: Multiplexer description
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 *
 * USB Type-C connectors can be used for alternate modes of operation besides
 * USB when Accessory/Alternate Modes are supported. With some of those modes,
 * the pins on the connector need to be reconfigured. This function registers
 * multiplexer switches routing the pins on the connector.
 */
<<<<<<< HEAD
int typec_mux_register(struct typec_mux *mux)
{
	mutex_lock(&mux_lock);
	list_add_tail(&mux->entry, &mux_list);
	mutex_unlock(&mux_lock);

	return 0;
=======
struct typec_mux *
typec_mux_register(struct device *parent, const struct typec_mux_desc *desc)
{
	struct typec_mux *mux;
	int ret;

	if (!desc || !desc->set)
		return ERR_PTR(-EINVAL);

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	mux->set = desc->set;

	device_initialize(&mux->dev);
	mux->dev.parent = parent;
	mux->dev.fwnode = desc->fwnode;
	mux->dev.class = &typec_mux_class;
	mux->dev.type = &typec_mux_dev_type;
	mux->dev.driver_data = desc->drvdata;
	dev_set_name(&mux->dev, "%s-mux", dev_name(parent));

	ret = device_add(&mux->dev);
	if (ret) {
		dev_err(parent, "failed to register mux (%d)\n", ret);
		put_device(&mux->dev);
		return ERR_PTR(ret);
	}

	return mux;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}
EXPORT_SYMBOL_GPL(typec_mux_register);

/**
 * typec_mux_unregister - Unregister Multiplexer Switch
 * @mux: USB Type-C Connector Multiplexer/DeMultiplexer
 *
 * Unregister mux that was registered with typec_mux_register().
 */
void typec_mux_unregister(struct typec_mux *mux)
{
<<<<<<< HEAD
	mutex_lock(&mux_lock);
	list_del(&mux->entry);
	mutex_unlock(&mux_lock);
}
EXPORT_SYMBOL_GPL(typec_mux_unregister);
=======
	if (!IS_ERR_OR_NULL(mux))
		device_unregister(&mux->dev);
}
EXPORT_SYMBOL_GPL(typec_mux_unregister);

void typec_mux_set_drvdata(struct typec_mux *mux, void *data)
{
	dev_set_drvdata(&mux->dev, data);
}
EXPORT_SYMBOL_GPL(typec_mux_set_drvdata);

void *typec_mux_get_drvdata(struct typec_mux *mux)
{
	return dev_get_drvdata(&mux->dev);
}
EXPORT_SYMBOL_GPL(typec_mux_get_drvdata);

struct class typec_mux_class = {
	.name = "typec_mux",
	.owner = THIS_MODULE,
};
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
