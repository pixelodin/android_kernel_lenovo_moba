#ifndef _AW9106_H_
#define _AW9106_H_

#define MAX_I2C_BUFFER_SIZE 65536

#define AW9106_ID 0x23
#define HRTIMER_FRAME				(20)

#define AUD_EFFECT_HRTIMER_INTERVAL			2000 //unit (ns)
enum led_mode {
	LED_MODE_MANUALLY_CONTROL,
	LED_MODE_FULL_BREATH,
	LED_MODE_SMART_FADE,
	LED_MODE_MAX,
};


enum led_effect_mode {
	LED_EFFECT_NORMALLY_ON,
	LED_EFFECT_AUTO_BLINK,
	LED_EFFECT_DOUBLE_BLINK,
	LED_EFFECT_DOUBLE_BLINK_U,
	LED_EFFECT_COLOR_BLINK,
	LED_EFFECT_AUD_EFFECT,
	LED_EFFECT_MAX,
};


struct aw9106_aud_led_data{
	unsigned char cur_effect;
	bool stop_effect_flag;
	bool runing_effect_flag;

	unsigned char * aud_data;
	unsigned int aud_data_len;
	unsigned char effect_nums;
	unsigned int cur_idx; 
};

struct aw9106_color {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct aw9106_led_color_blink_data{
	unsigned int t_fade_on;
	unsigned int t_keep_on;
	unsigned int t_fade_out;
	unsigned int t_keep_off;
	unsigned frame_nums;
	unsigned int cur_phase;
	unsigned cur_frame;
	unsigned cur_color;
	unsigned char color_nums;
	struct aw9106_color *p_color_list; 
};

struct aw9106_led_double_blink_data{
	unsigned int t_fade_on;
	unsigned int t_keep_on;
	unsigned int t_fade_out;
	unsigned int t_keep_off;
	unsigned int sec_t_keep_off;
	unsigned frame_nums;
	unsigned int cur_phase;
	unsigned cur_frame;
};

struct aw9106_single_led {
	//struct led_classdev cdev;
	//struct work_struct brightness_work;
	unsigned int idx;
	unsigned int start_blink_delay;
	enum led_mode cur_led_mode;
	unsigned cur_brightness;
	unsigned int max_brightness;
	//struct aw9106 *priv;
};

struct aw9106_led {
	unsigned int led_mask;
	int imax;
	int rise_time;
	int on_time;
	int fall_time;
	int off_time;
	int led_nums;
	enum led_effect_mode cur_effect_mode;
	struct hrtimer led_timer;
	struct work_struct effect_work;
	struct aw9106_aud_led_data aud_led_data;
	struct aw9106_single_led * single_led_data;
	struct aw9106_led_color_blink_data color_blink_data;
	struct aw9106_led_double_blink_data double_blink_data;
};

struct aw9106 {
    struct i2c_client *i2c;
    struct device *dev;
    struct led_classdev cdev;
    struct delayed_work aw_brightness_work;
    struct work_struct brightness_work;

    int reset_gpio;
    int mode;
    int bns;
    struct pinctrl *active_pinctrl;
    struct pinctrl_state *pinctrl_reset_state;

    unsigned char chipid;

    int imax;
    int rise_time;
    int on_time;
    int fall_time;
    int off_time;
    struct hrtimer led_timer;
    struct work_struct effect_work;
    struct aw9106_led led_data;
};
#endif
