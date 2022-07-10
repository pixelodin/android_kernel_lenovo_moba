/*****************************************************************************
* File: customize.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include <linux/input.h>
#include "config.h"
#include "track_report.h"
#include "input_device.h"
#include "debug.h"
#include "memory.h"
#include "utils.h"
#include "sysfs.h"
#include "irq.h"

int cust_write_registers(void *dev, int reg, int num, void *value);
int cust_read_registers(void *dev, int reg, int num, void *value);

/* Sentons sensor report style */
#define PIEZO  0
#define STRAIN 1
#define ZUI_TAP     3
#define ZUI_SLIDER  4

#include <linux/input/mt.h>
#include "device.h"
/*==========================================================================*/
/* register_input_events()                                                  */
/* Customize to register which input events we'll be sending                */
/*==========================================================================*/
void register_input_events(struct input_dev *input_dev)
{
	/* Set Light Sensor input device */
	input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);

	//Trk_id
	input_set_capability(input_dev, EV_ABS, ABS_MT_TRACKING_ID);
	__set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 8, 0, 0);

	//Bar_id
	input_set_capability(input_dev, EV_ABS, ABS_MT_WIDTH_MAJOR);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 1, 0, 0);

	//PRESSUR
	input_set_capability(input_dev, EV_ABS, ABS_MT_WIDTH_MINOR);
	__set_bit(ABS_MT_WIDTH_MINOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 255, 0, 0);

	//Fr_nr
	input_set_capability(input_dev, EV_ABS, ABS_MT_BLOB_ID);
	__set_bit(ABS_MT_BLOB_ID, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_BLOB_ID, 0, 65535, 0, 0);

	//Gesture type
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_X);
	__set_bit(ABS_MT_TOOL_X, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_X, 0, PANEL_SIZE, 0, 0);

	//Sensor type(pizeo/strain)
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_Y);
	__set_bit(ABS_MT_TOOL_Y, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_Y, 0, 1, 0, 0);

	//Center
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_TYPE);
	__set_bit(ABS_MT_TOOL_TYPE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, PANEL_SIZE, 0, 0);

	//Top
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PANEL_SIZE, 0, 0);

	//Bottom
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOUCH_MINOR);
	__set_bit(ABS_MT_TOUCH_MINOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, PANEL_SIZE, 0, 0);
#ifdef KEY_REPORT
	input_set_capability(input_dev, EV_KEY, KEY_KBDILLUMDOWN);
	input_set_capability(input_dev, EV_KEY, KEY_KBDILLUMUP);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
#endif
	PRINT_DEBUG("done");
}
void input_event_report(int g_id, int sensor_style, int trk_id, int bar_id,
						int force, int fr_nr, int center, int top, int bottom)
{

	//Gesture Type
	input_report_abs(get_input_device(), ABS_MT_TOOL_X, g_id);
	//Length
	input_report_abs(get_input_device(), ABS_MT_TOOL_Y, sensor_style);
	//Trk_id
	input_report_abs(get_input_device(), ABS_MT_TRACKING_ID, trk_id);
	//Bar_id
	input_report_abs(get_input_device(), ABS_MT_WIDTH_MAJOR, bar_id);
	//PRESSUR
	input_report_abs(get_input_device(), ABS_MT_WIDTH_MINOR, force);
	//Fr_nr
	input_report_abs(get_input_device(), ABS_MT_BLOB_ID, fr_nr);
	//center
	input_report_abs(get_input_device(), ABS_MT_TOOL_TYPE, center);
	//Top
	input_report_abs(get_input_device(), ABS_MT_TOUCH_MAJOR, top);
	//Bottom
	input_report_abs(get_input_device(), ABS_MT_TOUCH_MINOR, bottom);

	input_event(get_input_device(), EV_SYN, SYN_REPORT, 9);
	input_sync(get_input_device());

#if 1
    if (snt8100fsr_g->raw_report) {
        VERBOSE("RAW ABS_MT_WIDTH_MAJOR (bar_id) : %d", bar_id); // 1 for top bar, 2 for bottom bar.
        VERBOSE("RAW ABS_MT_WIDTH_MINOR (force)  : %d", force);
    } else {
        /*VERBOSE("ABS_MT_TOOL_X (g_id)        : %d", g_id);*/
        /*VERBOSE("ABS_MT_TOOL_Y (sensor_style): %d", sensor_style);*/
        /*VERBOSE("ABS_MT_TRACKING_ID (trk_id) : %d", trk_id);*/
        VERBOSE("ABS_MT_WIDTH_MAJOR (bar_id) : %d", bar_id); // 1 for top bar, 2 for bottom bar.
        VERBOSE("ABS_MT_WIDTH_MINOR (force)  : %d", force);
        /*VERBOSE("ABS_MT_BLOB_ID (fr_nr)      : %d", fr_nr);*/
        /*VERBOSE("ABS_MT_TOOL_TYPE (center)   : %d", center);*/
        VERBOSE("ABS_MT_TOUCH_MAJOR (top)    : %d", top);   // absolute position.
        /*VERBOSE("ABS_MT_TOUCH_MINOR (bottom) : %d", bottom);*/
    }
#endif
}

/**
 * Support functions to set custom operational profiles
 *
 * cust_write_registers(dev, StartRegId, NumReg, *RegValues)
 *     write 1 or more registers to SNT8100FSR starting at specified register.
 *
 */
static void set_touch_enable(void *dev, uint16_t reg_val) {
    cust_write_registers(dev, 1, 1, &reg_val);
}

static void set_frame_rate(void *dev, uint16_t reg_val) {
    cust_write_registers(dev, 2, 1, &reg_val);

}

static void set_triggers_ctrl(void *dev, uint16_t *reg_val)
{
	cust_write_registers(dev, 0x10, 2, reg_val);
}



//static void set_triggers(void *dev, uint16_t *reg_val) {
//    cust_write_registers(dev, 0x10, 7, reg_val);
//}

static void set_trigger_pulse_duration(void *dev, uint16_t reg_val)
{
	cust_write_registers(dev, 0x1d, 1, &reg_val);
}

static void set_tai(void *dev, uint16_t reg_val)
{
	cust_write_registers(dev, 0x1a, 1, &reg_val);
}

static void set_gpiostatus(void *dev, uint16_t reg_val)
{
	cust_write_registers(dev, 0x40, 1, &reg_val);
}
static void set_tap_gesture(void *dev, uint16_t tap_id, uint16_t *reg_val) {
    uint16_t cfg_bank_write[3] = { tap_id*16, 16, 0x0802};
    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
    cust_write_registers(dev, 0x200, 8, reg_val);
    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

static void set_tap_gesture_enable(void *dev, uint16_t tap_id, uint16_t enable)
{
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0801};
	uint16_t tap_size = 0;
	uint16_t *snt_profile_tap;
	uint16_t cfg_bank_write[3] = { tap_id*16, 2, 0x0802};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
	uint16_t tap_ctrl = 0;


	cust_write_registers(dev, 0x2c, 3, cfg_bank_read);
	cust_read_registers(dev, 0x200, 1, &tap_size);
	snt_profile_tap = (uint16_t *)memory_allocate(tap_size, GFP_DMA);

	cust_read_registers(dev, 0x200, tap_size/2, snt_profile_tap);

	if (enable == 1) {
		tap_ctrl = snt_profile_tap[8*tap_id] | 0x1;
		PRINT_DEBUG("Enable: 0x%x -> 0x%x", snt_profile_tap[8*tap_id], tap_ctrl);
	} else {
		tap_ctrl = snt_profile_tap[8*tap_id] & 0xfffe;
		PRINT_DEBUG("Disable: 0x%x -> 0x%x", snt_profile_tap[8*tap_id], tap_ctrl);
	}

	cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
	cust_write_registers(dev, 0x200, 1, &tap_ctrl);
	cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);

}

//static void set_squeeze_gesture(void *dev, uint16_t tap_id, uint16_t *reg_val) {
//    uint16_t cfg_bank_write[3] = { tap_id*18, 18, 0x0a02};
//    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0a03};
//    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
//    cust_write_registers(dev, 0x200, 9, reg_val);
//    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
//}

static void set_slider_gesture(void *dev, uint16_t slider_id, uint16_t *reg_val) {
    uint16_t cfg_bank_write[3] = { slider_id*12, 12, 0x0b02};
    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
    cust_write_registers(dev, 0x200, 6, reg_val);
    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}
static void set_slider_gesture_enable(void *dev, uint16_t slider_id,
									  uint16_t enable)
{
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0b01};
	uint16_t slider_size = 0;
	uint16_t *snt_profile_slider;
	uint16_t cfg_bank_write[3] = { slider_id*12, 2, 0x0b02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
	uint16_t slider_ctrl = 0;


	cust_write_registers(dev, 0x2c, 3, cfg_bank_read);
	cust_read_registers(dev, 0x200, 1, &slider_size);
	snt_profile_slider = (uint16_t *)memory_allocate(slider_size, GFP_DMA);

	cust_read_registers(dev, 0x200, slider_size/2, snt_profile_slider);

	if (enable == 1) {
		slider_ctrl = snt_profile_slider[8*slider_id] | 0x8000;
		PRINT_DEBUG("Enable: 0x%x -> 0x%x", snt_profile_slider[8*slider_id],
					slider_ctrl);
	} else {
		slider_ctrl = snt_profile_slider[8*slider_id] & 0x7fff;
		PRINT_DEBUG("Disable: 0x%x -> 0x%x", snt_profile_slider[8*slider_id],
					slider_ctrl);
	}

	cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
	cust_write_registers(dev, 0x200, 1, &slider_ctrl);
	cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);

}

//static void set_swipe_gesture(void *dev, uint16_t swipe_id, uint16_t *reg_val) {
//    uint16_t cfg_bank_write[3] = { swipe_id*10, 10, 0x0902};
//    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0903};
//    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
//    cust_write_registers(dev, 0x200, 5, reg_val);
//    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
//}

static void set_dpc(void *dev, uint16_t hi_hz, uint16_t lo_hz, uint16_t ctl) {
    uint16_t reg_val[3] = { hi_hz, lo_hz, ctl};
    cust_write_registers(dev, 0x39, 3, reg_val);
}

/**
 * Customization function for the sysfs attribute file "profile". Use this
 * function to set up specific settings of registers for the various operating
 * modes desired for the SNT8100FSR.
 *
 * The profiles in this function are examples and not meant to be a
 * comprehensive set of profiles. Users should provide their on content
 * based on their product needs.
 *
 */
void set_operational_profile(void *dev, int profile)
{
	PRINT_FUNC("profile = %d", profile);

	switch (profile) {
	case 0: { /* BOOT Profile */
		uint16_t snt_profile_tap0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		uint16_t snt_profile_vol_dn_tap1[8] = {0x08e1, 0x800, 1, 1, 0xffff, 0xffff, 0, 0};
		uint16_t snt_profile_vol_up_tap2[8] = {0x08e1, 0xc00, 2, 2, 0xffff, 0xffff, 0, 0};
		uint16_t snt_profile_slider0[6] = {0xa000, 0, 0xffff, 0x4027, 0, 0};
		uint16_t snt_profile_tap3[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		uint16_t snt_trigger_ctrl[2] = {0x0002, 0x0000};

		set_touch_enable(dev, 0);// Disable touch
		set_frame_rate(dev, 50);// frame rate = 50
		set_tap_gesture(dev, 0, snt_profile_tap0);// Disable tap0
		/*# Tap1 Vol Down
		 *0x2c 3 16 16 0x0802 # Tap1 is offset 16, size=16 bytes
		 *0x200 8
		 *0x08e1         # init_force=8, bar_id=7, en=1
		 *0x0800         # trig_id = 4, gre=0, slope_window=0
		 *1 1            # set detection area to region 1 in bar 7 region report.
		 *0xffff         # delta tap/release force off
		 *0xffff         # force tap/release window off
		 *0 0            # palm_reject_thresh=0, drag_reject_thresh=0
		 */
		set_tap_gesture(dev, 1, snt_profile_vol_dn_tap1);// Set tap1 vol down key
		/*# Tap2 Vol Up
		 *0x2c 3 32 16 0x0802 # Tap2 is offset 32, size=16 bytes
		 *0x200 8
		 *0x08e1         # init_force=8, bar_id=7, en=1
		 *0x0c00         # trig_id = 6, gre=0, slope_window=0
		 *2 2            # set detection area to region 2 in bar 7 region report.
		 *0xffff         # delta tap/release force off
		 *0xffff         # force tap/release window off
		 *0 0            # palm_reject_thresh=0, drag_reject_thresh=0
		 */
		set_tap_gesture(dev, 2, snt_profile_vol_up_tap2);// Set tap2 vol up key

		set_tap_gesture(dev, 3, snt_profile_tap3);// Disable tap3

		set_slider_gesture(dev, 0, snt_profile_slider0);// Set slider0
		/*# Trigger Control registers
		 *#
		 *# Trigger1 ?\A1\ECC Level, nonrepeating
		 *# Trigger2 ?\A1\ECC Level, nonrepeating
		 *# Trigger3 ?\A1\ECC Pulse, nonrepeating
		 *# Trigger4 ?\A1\ECC Level, nonrepeating
		 *# Trigger5 ?\A1\ECC Level, nonrepeating
		 *# Trigger6 ?\A1\ECC Level, nonrepeating
		 */
		set_triggers_ctrl(dev, snt_trigger_ctrl);
		set_tai(dev, 0x0128);
		set_trigger_pulse_duration(dev, 10);

		/*# Turn off Track Reports in TFIFO (STR bit 4)*/
		set_gpiostatus(dev, 8);
		set_dpc(dev, 50, 50, 2000);
		set_touch_enable(dev, 1);// Enable touch
		break;
	}
	case 1: { /* IDLE Profile */
		set_touch_enable(dev, 0);//# Disable touch
		set_tap_gesture_enable(dev, 1, 1);//# Tap1 Turn On Vol Down
		set_tap_gesture_enable(dev, 2, 1);//# Tap2 Turn On Vol Up
		set_slider_gesture_enable(dev, 0, 1);//# Slider0 Turn On
		set_touch_enable(dev, 1);//# Enable touch
		break;
	}
	case 2: { /* SLEEP Profile */
		set_touch_enable(dev, 0);//# Disable touch
		set_tap_gesture_enable(dev, 1, 0);//# Tap1 Turn Off Vol Down
		set_tap_gesture_enable(dev, 2, 0);//# Tap2 Turn Off Vol Up
		set_slider_gesture_enable(dev, 0, 0);//# Slider0 Turn Off
		/*# Turn off Track Reports in TFIFO (STR bit 4)*/
		set_gpiostatus(dev, 8);

		break;
	}
	case 3: { /* PWROFF Profile */
		set_touch_enable(dev, 0);//# Disable touch
		set_tap_gesture_enable(dev, 1, 0);//# Tap1 Turn Off Vol Down
		set_tap_gesture_enable(dev, 2, 0);//# Tap2 Turn Off Vol Up
		set_slider_gesture_enable(dev, 0, 0);//# Slider0 Turn Off
		/*# Turn off Track Reports in TFIFO (STR bit 4)*/
		set_gpiostatus(dev, 8);

		break;
	}

	default:
		PRINT_CRIT("Unknown profile %d", profile);
		break;
	}

}

/*==========================================================================*/
/* process_track_report()                                                   */
/* Customize to process track reports from the device                       */
/*==========================================================================*/
void process_track_reports(uint16_t frame,
                           struct track_report *tr,
                           size_t count) {
    int i;
    int gs_found = 0;
    int gesture_id = 0;
#ifdef KEY_REPORT
    static int32_t slide_pos0[SLIDE_MAX_NUM] = {0, 0, 0};
    int32_t slide_pos0_move[SLIDE_MAX_NUM] = {0, 0, 0};
    /*static int32_t slide_pos1[SLIDE_MAX_NUM] = {0, 0, 0};*/
    /*int32_t slide_pos1_move[SLIDE_MAX_NUM] = {0, 0, 0};*/
#endif

    for(i = 0; i < count; i++) {
        // diagnostic section
        if (tr[i].bar_id == 0 && tr[i].trk_id == 0 && tr[i].force_lvl <= TR_DIAG_REC_VERS_MAX)
            break;

        // gesture section
        if (gs_found || (tr[i].bar_id == 0 && tr[i].trk_id == 0 && tr[i].force_lvl > TR_DIAG_REC_VERS_MAX)) {
            if (gs_found == 0) {
                // process gesture header data here
                struct gs_hdr_rec_s *p = (struct gs_hdr_rec_s *) &tr[i];
                VERBOSE("Gesture: tap3=[%d],tap2=[%d], tap1=[%d], tap0=[%d]",
                             GS_GET_TAP_START(p->tap,3),GS_GET_TAP_START(p->tap,2),GS_GET_TAP_START(p->tap,1),GS_GET_TAP_START(p->tap,0));

#ifdef KEY_REPORT
                if ((snt8100fsr_g->factory_mode % 2 == 0) && (!snt8100fsr_g->raw_report)) {
#if 0
                    /*added by frankie for top gaming key */ 
                    if((GS_GET_TAP_START(p->tap,0)> 0)) {
                        input_event_report(ZUI_TAP, PIEZO, tr[i].trk_id, TOP_BAR_ID, tr[i].force_lvl, frame, BTN_TOOL_FINGER, 55, tr[i].bottom);
                    } else if (GS_GET_TAP_STOP(p->tap,0) > 0) {
                        VERBOSE("now report top tap evt: tap0_stop=[%d]", GS_GET_TAP_STOP(p->tap,0));
                        input_event_report(ZUI_TAP, PIEZO, tr[i].trk_id, TOP_BAR_ID, tr[i].force_lvl, frame, BTN_TOOL_FINGER, 0, tr[i].bottom);
                    }
                    /* bottom gamingkey*/
                    if((GS_GET_TAP_START(p->tap,1)> 0)) {
                        input_event_report(ZUI_TAP, PIEZO, tr[i].trk_id, BOT_BAR_ID, tr[i].force_lvl, frame, BTN_TOOL_FINGER, 56, tr[i].bottom);
                    } else if (GS_GET_TAP_STOP(p->tap,1) > 0) {
                        VERBOSE("now report bottom tap evt: tap1_stop=[%d]", GS_GET_TAP_STOP(p->tap,1));
                        input_event_report(ZUI_TAP, PIEZO, tr[i].trk_id, BOT_BAR_ID, tr[i].force_lvl, frame, BTN_TOOL_FINGER, 0, tr[i].bottom);
                    }
                    //add ended
#endif 

                    if (p->tap & TAP2_DOWN) { // Vol down key pressed
                        input_event(get_input_device(), EV_KEY, KEY_KBDILLUMDOWN, 1);
                        input_sync(get_input_device());
                    }
                    if (p->tap & TAP3_DOWN) { // Vol up key pressed
                        input_event(get_input_device(), EV_KEY, KEY_KBDILLUMUP, 1);
                        input_sync(get_input_device());
                    }
                    if (p->tap & TAP2_UP) { // Vol down key released
                        input_event(get_input_device(), EV_KEY, KEY_KBDILLUMDOWN, 0);
                        input_sync(get_input_device());
                    }
                    if (p->tap & TAP3_UP) { // Vol up key released
                        input_event(get_input_device(), EV_KEY, KEY_KBDILLUMUP, 0);
                        input_sync(get_input_device());
                    }
                }
#endif
                if (snt8100fsr_g->factory_mode != 0) {
                    if ((GS_GET_TAP_START(p->tap,0) > 0) ||(GS_GET_TAP_STOP(p->tap,0) > 0)) {
                        gesture_id = 1; // TAP0
                        if (GS_GET_TAP_START(p->tap,0) > 0) {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 1,
                                               tr[i].top, tr[i].bottom);
                        } else {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 0,
                                               tr[i].top, tr[i].bottom);
                        }
                    } else if ((GS_GET_TAP_START(p->tap,1) > 0) || (GS_GET_TAP_STOP(p->tap,1) > 0)) {
                        gesture_id = 2; // TAP1
                        if (GS_GET_TAP_START(p->tap,1) > 0) {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 1,
                                               tr[i].top, tr[i].bottom);
                        } else {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 0,
                                               tr[i].top, tr[i].bottom);
                        }
                    } else if ((GS_GET_TAP_START(p->tap, 2) > 0) || (GS_GET_TAP_STOP(p->tap, 2) > 0)) {
                        gesture_id = 3; // TAP2
                        if (GS_GET_TAP_START(p->tap,2) > 0) {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 1,
                                               tr[i].top, tr[i].bottom);
                        } else {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 0,
                                               tr[i].top, tr[i].bottom);
                        }
                    } else if ((GS_GET_TAP_START(p->tap, 3) > 0) || (GS_GET_TAP_STOP(p->tap, 3) > 0)) {
                        gesture_id = 18; // TAP3
                        if (GS_GET_TAP_START(p->tap,3) > 0) {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 1,
                                               tr[i].top, tr[i].bottom);
                        } else {
                            input_event_report(gesture_id, PIEZO, 1, tr[i].bar_id, tr[i].force_lvl, frame, 0,
                                               tr[i].top, tr[i].bottom);
                        }
                    }
                }
                gs_found = 1;
            } else {
                // process slider records here
                struct gs_slider_rec_s *p = (struct gs_slider_rec_s *) &tr[i];
                if (p->escape0 == GS_RPT_TYPE_SLIDE) {
                    uint8_t id0 = GS_GET_SLIDER_ID0(p->slider_finger_id);
                    uint8_t id1 = GS_GET_SLIDER_ID1(p->slider_finger_id);
                    uint8_t fid0 = GS_GET_SLIDER_FID0(p->slider_finger_id);
                    uint8_t fid1 = GS_GET_SLIDER_FID1(p->slider_finger_id);
                    if (fid0) {
                        VERBOSE("SL0[%u,%u]: F%u, P%u", id0, fid0, p->slider_force0, p->slider_pos0);
                    }
                    if (fid1) {
                        VERBOSE("SL1[%u,%u]: F%u, P%u", id1, fid1, p->slider_force1, p->slider_pos1);
                    }

#ifdef KEY_REPORT
                    if(snt8100fsr_g->factory_mode % 2 == 0) {
                        if (fid0) {
                            VERBOSE("Slide%d", id0);
                            if (slide_pos0[id0] == 0) {
                                slide_pos0[id0] = p->slider_pos0;
                            } else {
                                slide_pos0_move[id0] = p->slider_pos0 - slide_pos0[id0];
#if 0
                                if (slide_pos0_move[id0] > 0 && p->slider_pos0 != 0) {
                                    PRINT_DEBUG("SL%d: bar_id=%d move=%d,Report KEY_UP", id0, slide_pos0_move[id0], tr[i].bar_id);
                                    input_report_key(get_input_device(), KEY_UP, 1);
                                    input_sync(get_input_device());
                                    input_report_key(get_input_device(), KEY_UP, 0);
                                    input_sync(get_input_device());
                                } else if (slide_pos0_move[id0] < 0 && p->slider_pos0 != 0) {
                                    PRINT_DEBUG("SL%d: bar_id=%d move=%d,Report KEY_DOWN", id0, slide_pos0_move[id0], tr[i].bar_id);
                                    input_report_key(get_input_device(), KEY_DOWN, 1);
                                    input_sync(get_input_device());
                                    input_report_key(get_input_device(), KEY_DOWN, 0);
                                    input_sync(get_input_device());
                                }
#endif
                                slide_pos0[id0] = p->slider_pos0;
                            }
                            if (!snt8100fsr_g->raw_report) 
                                input_event_report(ZUI_SLIDER, PIEZO, 1, id0+1, tr[i].force_lvl, frame, BTN_TOOL_FINGER, p->slider_pos0, tr[i].bottom);
                        }

#if 0
                        if (fid1) {
                            if (slide_pos1[id1] == 0) {
                                slide_pos1[id1] = p->slider_pos1;
                            } else {
                                slide_pos1_move[id1] = p->slider_pos1 - slide_pos1[id1];
                                if (slide_pos1_move[id1] > 0 && p->slider_pos1 != 0) {
                                    PRINT_DEBUG("SL%d: bar_id=%d move=%d,Report KEY_UP.", id1, slide_pos1_move[id1], tr[i].bar_id);
                                    input_report_key(get_input_device(), KEY_UP, 1);
                                    input_sync(get_input_device());
                                    input_report_key(get_input_device(), KEY_UP, 0);
                                    input_sync(get_input_device());
                                } else if (slide_pos1_move[id1] < 0 && p->slider_pos1 != 0) {
                                    PRINT_DEBUG("SL%d: bar_id=%d move=%d,Report KEY_DOWN.", id1, slide_pos1_move[id1], tr[i].bar_id);
                                    input_report_key(get_input_device(), KEY_DOWN, 1);
                                    input_sync(get_input_device());
                                    input_report_key(get_input_device(), KEY_DOWN, 0);
                                    input_sync(get_input_device());
                                }
                                slide_pos1[id1] = p->slider_pos1;
                            }
                        }
#endif
                    }
#endif

                    if(snt8100fsr_g->factory_mode != 0) {
                        if (fid0) {
                            if (id0 == 0) {
                                gesture_id = 12; // SLIDER0
                            } else {
                                gesture_id = 13; // SLIDER1
                            }
                            PRINT_INFO("Slider[%u,%u]: F%u, P%u, g_id=%d", id0, fid0,
                                        p->slider_force0, p->slider_pos0, gesture_id);
                            input_event_report(gesture_id, PIEZO, fid0, id0, p->slider_force0, frame,
                                               p->slider_pos0, 0, 0);
                        }
                        if (fid1) {
                            if (id1 == 0) {
                                gesture_id = 12; // SLIDER0
                            } else {
                                gesture_id = 13; // SLIDER1
                            }
                            PRINT_INFO("Slider[%u,%u]: F%u, P%u, g_id=%d", id1, fid1,
                                        p->slider_force1, p->slider_pos1, gesture_id);
                            input_event_report(gesture_id, PIEZO, fid1, id1, p->slider_force1, frame,
                                        p->slider_pos1, 0, 0);
                        }
                    }
                }
            }
        } else {
            if(snt8100fsr_g->factory_mode == 0) {
                // track report section
                if (IS_STG_TRACK_REPORT(tr[i].bar_id)) {
                    struct stg_track_report *tg = (struct stg_track_report*) tr;
                    // Timestamp, Frame, Bar, Frc0, Frc1, Frc2, Frc3, Frc4, Frc5, Frce6, Frc7
                    VERBOSE("StrainRpt %u: B%u, F0%u, F1%u, F2%u, F3%u",
                                 frame,
                                 tg[i].bar_id,
                                 tg[i].force_lvl[0],
                                 tg[i].force_lvl[1],
                                 tg[i].force_lvl[2],
                                 tg[i].force_lvl[3]);
                    VERBOSE("StrainRpt contd: F4%u, F5%u, F6%u",
                                 tg[i].force_lvl[4],
                                 tg[i].force_lvl[5],
                                 tg[i].force_lvl[6]);
                    /*input_event_report(0, STRAIN, tr[i].trk_id, tg[i].bar_id, tg[i].force_lvl[0], tg[i].force_lvl[1],*/
                            /*tg[i].force_lvl[2], tg[i].force_lvl[3], tg[i].force_lvl[4]);*/
                } else {
                    VERBOSE("Report %u: B%u, T%u, F%u, %u, %u, %u",
                                frame,
                                tr[i].bar_id,
                                tr[i].trk_id,
                                tr[i].force_lvl,
                                tr[i].top,
                                tr[i].center,
                                tr[i].bottom);
                    if (snt8100fsr_g->raw_report) {
                        switch (tr[i].bar_id) {
                        case 0: // bottom bar.
                            input_event_report(0, PIEZO, tr[i].trk_id, BOT_BAR_ID, tr[i].force_lvl, frame, tr[i].center, tr[i].top, tr[i].bottom);
                            break;
                        case 1: // top bar.
                            input_event_report(0, PIEZO, tr[i].trk_id, TOP_BAR_ID, tr[i].force_lvl, frame, tr[i].center, tr[i].top, tr[i].bottom);
                            break;
                        }
                    } 
                }
            }
        }
    }
}

#if USE_TRIG_IRQ
/*==========================================================================*/
/* process_trigger()                                                        */
/* Customize to process trigger interrupts from the device                  */
/* trig_id values: 0, 1, 2 correlate to TRIG0, TRIG1, TRIG2 in config.h     */
/*==========================================================================*/
void process_trigger(int trig_id)
{
    int trig_status;
    VERBOSE("process_trigger");

    if (snt8100fsr_g->raw_report)
        return;

    if (trig_id == 0) {
        trig_status = gpio_get_value(trig0_irq_db.gpio_num);
        VERBOSE("Gesture: Tap%d, finger %s!", trig_id, (trig_status == 1)?"down":"up");
    
        if(trig_status == 1){
            input_event_report(ZUI_TAP, PIEZO, 0, TOP_BAR_ID, 0, 0, BTN_TOOL_FINGER, 55, 0);
        } else {
            input_event_report(ZUI_TAP, PIEZO, 0, TOP_BAR_ID, 0, 0, BTN_TOOL_FINGER, 0, 0);
        }
    } else if (trig_id == 1) {
        trig_status = gpio_get_value(trig1_irq_db.gpio_num);
        VERBOSE("Gesture: Tap%d, finger %s!", trig_id, (trig_status == 1)?"down":"up");
    
        if (trig_status == 1) {
            input_event_report(ZUI_TAP, PIEZO, 0, BOT_BAR_ID, 0, 0, BTN_TOOL_FINGER, 56, 0);
        } else {
            input_event_report(ZUI_TAP, PIEZO, 0, BOT_BAR_ID, 0, 0, BTN_TOOL_FINGER, 0, 0);
        }
    }

    VERBOSE("process_trigger done.");
}
#endif
