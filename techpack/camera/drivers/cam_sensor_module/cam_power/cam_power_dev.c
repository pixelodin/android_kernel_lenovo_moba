#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include "cam_soc_util.h"
#include <cam_req_mgr_util.h>
#include "cam_req_mgr_dev.h"
#include "cam_debug_util.h"
#include "cam_power_dev.h"

#ifdef CONFIG_PRODUCT_MOBA

#define WL_SLAVE_ADDRESS 0x52

static cam_power_ctrl_t *power_ctrl = NULL;

//NOTE:Will swap ldo1 & ldo2 register value for dota project
//cam0 is main camera
static struct cam_sensor_i2c_reg_array cam0_reg_on_setting[] = {
	{0x03, 0x24, 0x1, 0x0}, //cam1 dvdd 1.2v LDO1
	{0x05, 0x80, 0x1, 0x0}, //cam1 vcm 2.8v LDO3
	{0x07, 0x80, 0x1, 0x0}, //cam0 2.8v LDO5
	{0x01, 0x2F, 0x1, 0x0}, //Higher current limit
	{0x0E, 0x15, 0x1, 0x0}, //enable ldo1,ldo3,ldo5
};

//cam1 is aux camera
static struct cam_sensor_i2c_reg_array cam1_reg_on_setting[] = {
	{0x08, 0x80, 0x1, 0x0}, //cam1 vcm 2.8v LDO6
	{0x01, 0x20, 0x1, 0x0}, //Higher current limit
	{0x0E, 0x20, 0x1, 0x0}, //enable ldo6
};

//cam2 is front camera
static struct cam_sensor_i2c_reg_array cam2_reg_on_setting[] = {
	{0x04, 0x24, 0x1, 0x0}, //cam2 1.05v LDO2
	{0x06, 0x80, 0x1, 0x0}, //cam2 2.80v LDO4
	{0x01, 0x0F, 0x1, 0x0}, //Higher current limit
	{0x0E, 0x0A, 0x1, 0x0}, //enable ldo2,ldo4
};

static struct cam_sensor_i2c_reg_array cam_reg_off_setting[] = {
	{0x0E, 0x00, 0x2, 0x0}, //disable all
};

static struct cam_sensor_i2c_reg_array cam0_reg_off_setting[] = {
	{0x0E, 0x00, 0x2, 0x0},
};

static struct cam_sensor_i2c_reg_array cam1_reg_off_setting[] = {
	{0x0E, 0x00, 0x2, 0x0},
};

static struct cam_sensor_i2c_reg_array cam2_reg_off_setting[] = {
	{0x0E, 0x00, 0x2, 0x0},
};

static struct cam_sensor_i2c_reg_setting wl_cam0_reg_on_setting = {
	.reg_setting = cam0_reg_on_setting,
	.size = 5,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam1_reg_on_setting = {
	.reg_setting = cam1_reg_on_setting,
	.size = 3,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam2_reg_on_setting = {
	.reg_setting = cam2_reg_on_setting,
	.size = 4,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_reg_off_setting = {
	.reg_setting = cam_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam0_reg_off_setting = {
	.reg_setting = cam0_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam1_reg_off_setting = {
	.reg_setting = cam1_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting wl_cam2_reg_off_setting = {
	.reg_setting = cam2_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static int set_cam0_vol_on(bool on)
{
	int rc = 0;
	uint32_t chiprevison = 0x10;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	CAM_DBG(CAM_POWER, "wl set cam0 vol %d sid:%d",
		on, io_master_info.cci_client->sid);

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_POWER, "cci_init failed: rc: %d", rc);
		return -EINVAL;
	}
	rc = camera_io_dev_read(
			&(io_master_info),
			0x00,
			&chiprevison, CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (on) {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl set cam0 return %d:", rc);
        } else {
            if (wl_cam0_reg_on_setting.reg_setting[4].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:0 [Main] <open> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam0_reg_on_setting.reg_setting[4].reg_data);
                wl_cam0_reg_on_setting.reg_setting[4].reg_data |= reg_val;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:0 [Main] <open> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam0_reg_on_setting.reg_setting[4].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam0_reg_on_setting);
        }
	} else {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl read reg return %d:", rc);
            rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
        } else {
            if (wl_cam0_reg_off_setting.reg_setting[0].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:0 [Main] <close> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam0_reg_off_setting.reg_setting[0].reg_data);
                wl_cam0_reg_off_setting.reg_setting[0].reg_data = reg_val & 0xEA;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:0 [Main] <close> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam0_reg_off_setting.reg_setting[0].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam0_reg_off_setting);
        }
	}

	if (rc < 0) {
		CAM_ERR(CAM_POWER, "wl set cam1 return %d:", rc);
	}

	camera_io_release(&io_master_info);

	return rc;
}

static int set_cam1_vol_on(bool on)
{
	int rc = 0;
	uint32_t chiprevison = 0x10;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	CAM_DBG(CAM_POWER, "wl set cam1 vol %d sid:%d",
		on, io_master_info.cci_client->sid);

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_POWER, "cci_init failed: rc: %d", rc);
		return -EINVAL;
	}
	rc = camera_io_dev_read(
			&(io_master_info),
			0x00,
			&chiprevison, CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	CAM_ERR(CAM_POWER, "WL2864 chiprevison=0x%x rc:%d", chiprevison, rc);
	if (on) {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl set cam1 return %d:", rc);
        } else {
            if (wl_cam1_reg_on_setting.reg_setting[2].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:1 [Aux] <open> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam1_reg_on_setting.reg_setting[2].reg_data);
                wl_cam1_reg_on_setting.reg_setting[2].reg_data |= reg_val;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:1 [Aux] <open> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam1_reg_on_setting.reg_setting[2].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam1_reg_on_setting);
        }
	} else {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl read reg return %d:", rc);
            rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
        } else {
            if (wl_cam1_reg_off_setting.reg_setting[0].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:1 [Aux] <close> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam1_reg_off_setting.reg_setting[0].reg_data);
                wl_cam1_reg_off_setting.reg_setting[0].reg_data = reg_val & 0xDF;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:1 [Aux] <close> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam1_reg_off_setting.reg_setting[0].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam1_reg_off_setting);
        }
	}

	if (rc < 0) {
		CAM_ERR(CAM_POWER, "wl set cam1 return %d:", rc);
	}

	camera_io_release(&io_master_info);

	return rc;
}

static int set_cam2_vol_on(bool on)
{
	int rc = 0;
	uint32_t chiprevison = 0x10;
	struct camera_io_master io_master_info =
		power_ctrl->io_master_info;

	CAM_DBG(CAM_POWER, "wl set cam2 vol %d sid:%d",
		on, io_master_info.cci_client->sid);

	rc = camera_io_init(&io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_POWER, "cci_init failed: rc: %d", rc);
		return -EINVAL;
	}
	rc = camera_io_dev_read(
			&(io_master_info),
			0x00,
			&chiprevison, CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	CAM_ERR(CAM_POWER, "WL2864 chiprevison=0x%x rc:%d", chiprevison, rc);
	if (on) {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl read reg return %d:", rc);
        } else {
            if (wl_cam2_reg_on_setting.reg_setting[3].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:2 [Front] <open> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam2_reg_on_setting.reg_setting[3].reg_data);
                wl_cam2_reg_on_setting.reg_setting[3].reg_data |= reg_val;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:2 [Front] <open> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam2_reg_on_setting.reg_setting[3].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam2_reg_on_setting);
        }
	} else {
        uint32_t reg_val = 0;
        rc = camera_io_dev_read(
                &(io_master_info),
                0x0E,
                &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE,
                CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (rc < 0) {
            CAM_ERR(CAM_POWER, "wl read reg return %d:", rc);
            rc = camera_io_dev_write(&(io_master_info), &wl_reg_off_setting);
        } else {
            if (wl_cam2_reg_off_setting.reg_setting[0].reg_addr == 0x0E) {
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:2 [Front] <close> before wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam2_reg_off_setting.reg_setting[0].reg_data);
                wl_cam2_reg_off_setting.reg_setting[0].reg_data = reg_val & 0xF5;
CAM_ERR(CAM_POWER, "[INFO] muzi--- cameraId:2 [Front] <close> after wl reg_val:0x%x,reg_data:0x%x",
	reg_val,wl_cam2_reg_off_setting.reg_setting[0].reg_data);
            }
            rc = camera_io_dev_write(&(io_master_info), &wl_cam2_reg_off_setting);
        }
	}

	if (rc < 0) {
		CAM_ERR(CAM_POWER, "wl set cam2 return %d:", rc);
	}

	camera_io_release(&io_master_info);

	return rc;
}

int cam_power_ldo_control(uint16_t cam_cell_id, bool enable)
{
	int rc = 0;

	switch(cam_cell_id) {
		case CAM_CELL_ID_0:
			rc = set_cam0_vol_on(enable);
			if (rc < 0)
				CAM_ERR(CAM_POWER, "set vol failed rc:%d", rc);
			break;
		case CAM_CELL_ID_1:
			rc = set_cam1_vol_on(enable);
			if (rc < 0)
				CAM_ERR(CAM_POWER, "set vol failed rc:%d", rc);
			break;
		case CAM_CELL_ID_2:
			rc = set_cam2_vol_on(enable);
			if (rc < 0)
				CAM_ERR(CAM_POWER, "set vol failed rc:%d", rc);
			break;
		case CAM_CELL_ID_3:
			break;
		default:
			break;
	}
	return rc;
}

cam_power_ctrl_t* get_cam_power_ctrl(void)
{
	return power_ctrl;
}

static int cam_power_parse_dt(cam_power_ctrl_t *p_ctrl)
{
	int rc = 0;
	struct cam_sensor_cci_client *cci_client = NULL;

	if (p_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = p_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_POWER, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = MASTER_1;
		cci_client->cci_device = CCI_DEVICE_1;
		cci_client->sid = (WL_SLAVE_ADDRESS >> 1);
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = I2C_FAST_MODE;
	}

	return rc;
}

static const struct v4l2_subdev_internal_ops cam_power_internal_ops = {
	.close = NULL,
};

static struct v4l2_subdev_core_ops cam_power_subdev_core_ops = {
	.ioctl = NULL,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = NULL,
#endif
};

static struct v4l2_subdev_ops cam_power_subdev_ops = {
	.core = &cam_power_subdev_core_ops,
};

static int cam_power_init_subdev(cam_power_ctrl_t *p_ctrl)
{
	int rc = 0;

	p_ctrl->v4l2_dev_str.internal_ops = &cam_power_internal_ops;
	p_ctrl->v4l2_dev_str.ops = &cam_power_subdev_ops;
	strlcpy(p_ctrl->device_name, CAM_POWER_NAME,
		sizeof(p_ctrl->device_name));
	p_ctrl->v4l2_dev_str.name = p_ctrl->device_name;
	p_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	p_ctrl->v4l2_dev_str.ent_function = CAM_POWER_DEVICE_TYPE;
	p_ctrl->v4l2_dev_str.token = p_ctrl;

	rc = cam_register_subdev(&(p_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_POWER, "Fail with cam_register_subdev");

	return rc;
}

static int32_t cam_power_platform_driver_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0;
	cam_power_ctrl_t *p_ctrl = NULL;

	p_ctrl = kzalloc(sizeof(cam_power_ctrl_t), GFP_KERNEL);
	if (!p_ctrl)
		return -ENOMEM;

	p_ctrl->soc_info.pdev = pdev;
	p_ctrl->soc_info.dev = &pdev->dev;
	p_ctrl->soc_info.dev_name = pdev->name;
	p_ctrl->power_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	p_ctrl->userspace_probe = false;

	p_ctrl->io_master_info.master_type = CCI_MASTER;

	p_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!p_ctrl->io_master_info.cci_client) {
		rc = -ENOMEM;
		goto free_p_ctrl;
	}

	mutex_init(&(p_ctrl->power_mutex));
	rc = cam_power_parse_dt(p_ctrl);
	if (rc) {
		CAM_ERR(CAM_POWER, "failed: soc init rc %d", rc);
		goto free_cci_client;
	}

	rc = cam_power_init_subdev(p_ctrl);
	if (rc)
		goto free_cci_client;

	p_ctrl->bridge_intf.device_hdl = -1;
	p_ctrl->bridge_intf.ops.get_dev_info = NULL;
	p_ctrl->bridge_intf.ops.link_setup = NULL;
	p_ctrl->bridge_intf.ops.apply_req = NULL;

	platform_set_drvdata(pdev, p_ctrl);
	v4l2_set_subdevdata(&p_ctrl->v4l2_dev_str.sd, p_ctrl);

	power_ctrl = p_ctrl;
	CAM_DBG(CAM_POWER, "probe success %d pctrl = %p", rc, (p_ctrl!=NULL)? p_ctrl:NULL);

	return rc;
free_cci_client:
	kfree(p_ctrl->io_master_info.cci_client);
free_p_ctrl:
	kfree(p_ctrl);
	return rc;
}

static int cam_power_platform_driver_remove(struct platform_device *pdev)
{
	cam_power_ctrl_t  *p_ctrl;

	p_ctrl = platform_get_drvdata(pdev);
	if (!p_ctrl) {
		CAM_ERR(CAM_POWER, "external power device is NULL");
		return -EINVAL;
	}

	kfree(p_ctrl->io_master_info.cci_client);
	platform_set_drvdata(pdev, NULL);
	kfree(p_ctrl);
	return 0;
}

static const struct of_device_id cam_power_dt_match[] = {
	{ .compatible = "qcom,ic_power" },
	{ }
};

MODULE_DEVICE_TABLE(of, cam_power_dt_match);

static struct platform_driver cam_power_platform_driver = {
	.driver = {
		.name = "qcom,ic_power",
		.owner = THIS_MODULE,
		.of_match_table = cam_power_dt_match,
	},
	.probe = cam_power_platform_driver_probe,
	.remove = cam_power_platform_driver_remove,
};

static int __init cam_power_driver_init(void)
{
	int rc = 0;
	CAM_DBG(CAM_POWER, "platform_driver_register");

	rc = platform_driver_register(&cam_power_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_POWER, "platform_driver_register failed rc = %d",
			rc);
		return rc;
	}
	return rc;
}

static void __exit cam_power_driver_exit(void)
{
	platform_driver_unregister(&cam_power_platform_driver);
}

module_init(cam_power_driver_init);
module_exit(cam_power_driver_exit);
MODULE_DESCRIPTION("CAM POWER driver");
MODULE_LICENSE("GPL v2");

#endif
