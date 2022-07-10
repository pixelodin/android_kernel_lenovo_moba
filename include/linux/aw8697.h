#ifndef _AW8697_H_
#define _AW8697_H_

/*********************************************************
 *
 * kernel version
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

/*********************************************************
 *
 * aw8697.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define AW8697_CHIPID                   0x97

#define MAX_I2C_BUFFER_SIZE                 65536

#define AW8697_SEQUENCER_SIZE               8
#define AW8697_SEQUENCER_LOOP_SIZE          4

#define AW8697_RTP_I2C_SINGLE_MAX_NUM       512

#define HAPTIC_MAX_TIMEOUT                  10000

#define AW8697_VBAT_REFER                   4200
#define AW8697_VBAT_MIN                     3000
#define AW8697_VBAT_MAX                     4500

/* trig config */
#define AW8697_TRIG_NUM                     3
#define AW8697_TRG1_ENABLE                  1
#define AW8697_TRG2_ENABLE                  0
#define AW8697_TRG3_ENABLE                  0

/*
 * trig default high level
 * ___________           _________________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |_________________
 *        first edge
 *                   second edge
 */
#define AW8697_TRG1_DEFAULT_LEVEL     0	/* 1: high level; 0: low level */
#define AW8697_TRG2_DEFAULT_LEVEL     1	/* 1: high level; 0: low level */
#define AW8697_TRG3_DEFAULT_LEVEL     1	/* 1: high level; 0: low level */

#define AW8697_TRG1_DUAL_EDGE         1	/* 1: dual edge; 0: first edge */
#define AW8697_TRG2_DUAL_EDGE         1	/* 1: dual edge; 0: first edge */
#define AW8697_TRG3_DUAL_EDGE         1	/* 1: dual edge; 0: first edge */

#define AW8697_TRG1_FIRST_EDGE_SEQ    5	/* trig1: first edge waveform seq */
#define AW8697_TRG1_SECOND_EDGE_SEQ   5	/* trig1: second edge waveform seq */
#define AW8697_TRG2_FIRST_EDGE_SEQ    1	/* trig2: first edge waveform seq */
#define AW8697_TRG2_SECOND_EDGE_SEQ   2	/* trig2: second edge waveform seq */
#define AW8697_TRG3_FIRST_EDGE_SEQ    1	/* trig3: first edge waveform seq */
#define AW8697_TRG3_SECOND_EDGE_SEQ   2	/* trig3: second edge waveform seq */

#if AW8697_TRG1_ENABLE
#define AW8697_TRG1_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG1_ENABLE
#else
#define AW8697_TRG1_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG1_DISABLE
#endif

#if AW8697_TRG2_ENABLE
#define AW8697_TRG2_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG2_ENABLE
#else
#define AW8697_TRG2_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG2_DISABLE
#endif

#if AW8697_TRG3_ENABLE
#define AW8697_TRG3_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG3_ENABLE
#else
#define AW8697_TRG3_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG3_DISABLE
#endif

#if AW8697_TRG1_DEFAULT_LEVEL
#define AW8697_TRG1_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG1_POLAR_POS
#else
#define AW8697_TRG1_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG
#endif

#if AW8697_TRG2_DEFAULT_LEVEL
#define AW8697_TRG2_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG2_POLAR_POS
#else
#define AW8697_TRG2_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG
#endif

#if AW8697_TRG3_DEFAULT_LEVEL
#define AW8697_TRG3_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG3_POLAR_POS
#else
#define AW8697_TRG3_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG
#endif

#if AW8697_TRG1_DUAL_EDGE
#define AW8697_TRG1_DEFAULT_EDGE      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG
#else
#define AW8697_TRG1_DEFAULT_EDGE      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS
#endif

#if AW8697_TRG2_DUAL_EDGE
#define AW8697_TRG2_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG
#else
#define AW8697_TRG2_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG2_EDGE_POS
#endif

#if AW8697_TRG3_DUAL_EDGE
#define AW8697_TRG3_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG
#else
#define AW8697_TRG3_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG3_EDGE_POS
#endif

enum aw8697_flags {
	AW8697_FLAG_NONR = 0,
	AW8697_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8697_haptic_read_write {
	AW8697_HAPTIC_CMD_READ_REG = 0,
	AW8697_HAPTIC_CMD_WRITE_REG = 1,
};

enum aw8697_haptic_work_mode {
	AW8697_HAPTIC_STANDBY_MODE = 0,
	AW8697_HAPTIC_RAM_MODE = 1,
	AW8697_HAPTIC_RTP_MODE = 2,
	AW8697_HAPTIC_TRIG_MODE = 3,
	AW8697_HAPTIC_CONT_MODE = 4,
	AW8697_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8697_haptic_bst_mode {
	AW8697_HAPTIC_BYPASS_MODE = 0,
	AW8697_HAPTIC_BOOST_MODE = 1,
};

enum aw8697_haptic_activate_mode {
	AW8697_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW8697_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw8697_haptic_cont_vbat_comp_mode {
	AW8697_HAPTIC_CONT_VBAT_SW_COMP_MODE = 0,
	AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw8697_haptic_ram_vbat_comp_mode {
	AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8697_haptic_f0_flag {
	AW8697_HAPTIC_LRA_F0 = 0,
	AW8697_HAPTIC_CALI_F0 = 1,
};

enum aw8697_haptic_pwm_mode {
	AW8697_PWM_48K = 0,
	AW8697_PWM_24K = 1,
	AW8697_PWM_12K = 2,
};

enum aw8697_haptic_play {
	AW8697_HAPTIC_PLAY_NULL = 0,
	AW8697_HAPTIC_PLAY_ENABLE = 1,
	AW8697_HAPTIC_PLAY_STOP = 2,
	AW8697_HAPTIC_PLAY_GAIN = 8,
};

enum aw8697_haptic_cmd {
	AW8697_HAPTIC_CMD_NULL = 0,
	AW8697_HAPTIC_CMD_ENABLE = 1,
	AW8697_HAPTIC_CMD_HAPTIC = 0x0f,
	AW8697_HAPTIC_CMD_TP = 0x10,
	AW8697_HAPTIC_CMD_SYS = 0xf0,
	AW8697_HAPTIC_CMD_STOP = 255,
};

enum aw8697_haptic_cali_lra {
	AW8697_HAPTIC_F0_CALI_LRA = 1,
	AW8697_HAPTIC_RTP_CALI_LRA = 2,
};

enum aw8697_haptic_tp_flag {
    AW8697_HAPTIC_TP_NULL = 0,
    AW8697_HAPTIC_TP_PRESS = 1,
    AW8697_HAPTIC_TP_PRESS_HOLD = 2,
    AW8697_HAPTIC_TP_RELEASE = 3,
    AW8697_HAPTIC_TP_RELEASE_HOLD = 4,
};
enum aw8697_haptic_tp_staus {
    AW8697_HAPTIC_TP_ST_RELEASE = 0,
    AW8697_HAPTIC_TP_ST_PRESS = 1,
};

enum aw8697_haptic_tp_play_flag {
    AW8697_HAPTIC_TP_PLAY_NULL = 0,
    AW8697_HAPTIC_TP_PLAY_ENABLE = 1,
    AW8697_HAPTIC_TP_PLAY_NOMORE= 2,
};

#define AW8697_HAPTIC_TP_ID_MAX     15

//Daniel 20200423 modify start
enum aw_game_dir_cmd {
	AW_GAME_DIR_L = 0,
	AW_GAME_DIR_R = 1,
	AW_GAME_DIR_DBL = 2,
};

//tp_flag bit4:0
enum aw_speed_game_cmd {
	AW_SPEED_GAME_VIB_ON = 0xc1,
	AW_SPEED_GAME_VIB_OFF = 0x01,
};

//tp_flag bit4:1
enum aw_shoot_game_cmd {
	AW_SHOOT_GAME_VIB_L_ON = 0xb1,
	AW_SHOOT_GAME_VIB_R_ON = 0x71,
	AW_SHOOT_GAME_VIB_DBL_ON = 0xd1,
	AW_SHOOT_GAME_VIB_OFF = 0x11,
};

enum aw_ram_vib_type {
	AW_RAM_SHORT_VIB = 0,
	AW_RAM_LONG_VIB = 1,
};
/*********************************************************
 *
 * struct
 *
 ********************************************************/
 struct shake_point {
    uint16_t x;
    uint16_t y;
    uint8_t  status;
};

struct tp_point_event {
    struct shake_point record_point[10];
    unsigned char i;
};

struct fileops {
	unsigned char cmd;
	unsigned char reg;
	unsigned char ram_addrh;
	unsigned char ram_addrl;
};

struct tp_id{
    struct shake_point pt_info;
    unsigned char tp_flag;
    unsigned char press_flag;
    unsigned char release_flag;
    struct timeval t_press;
    struct timeval t_release;
    unsigned char play_flag;
    unsigned int no_play_cnt;
};

struct tp{
    struct tp_id id[AW8697_HAPTIC_TP_ID_MAX];
    unsigned char id_index;
    unsigned int press_delay_min;
    unsigned int press_delay_max;
    unsigned int release_delay_max;
    unsigned char play_flag;
    unsigned int no_play_cnt_max;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct haptic_ctr {
	unsigned char cnt;
	unsigned char cmd;
	unsigned char play;
	unsigned char wavseq;
	unsigned char loop;
	unsigned char gain;
	struct list_head list;
};

struct haptic_audio {
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	int delay_val;
	int timer_val;
	struct haptic_ctr ctr;
	struct list_head ctr_list;
	struct tp tp;
	struct list_head list;
	/*  struct haptic_audio_tp_size tp_size; */
	/*   struct trust_zone_info output_tz_info[10]; */
	int tz_num;
	int tz_high_num;
	int tz_cnt_thr;
	int tz_cnt_max;
	unsigned int uevent_report_flag;
	unsigned int hap_cnt_outside_tz;
	unsigned int hap_cnt_max_outside_tz;
};

struct trig {
	unsigned char enable;
	unsigned char default_level;
	unsigned char dual_edge;
	unsigned char frist_seq;
	unsigned char second_seq;
};

struct aw8697_dts_info {
	unsigned int mode;
	unsigned int f0_pre;
	unsigned int f0_cali_percen;
	unsigned int cont_drv_lvl;
	unsigned int cont_drv_lvl_ov;
	unsigned int cont_td;
	unsigned int cont_zc_thr;
	unsigned int cont_num_brk;
	unsigned int f0_coeff;
	unsigned int f0_trace_parameter[4];
	unsigned int bemf_config[4];
	unsigned int sw_brake;
	unsigned int tset;
	unsigned int r_spare;
	unsigned int bstdbg[6];
	unsigned int parameter1;
};

struct aw8697 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct double_vibrator_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	//Daniel 20200327 modify start
	struct timeval current_time;
	struct timeval pre_enter_time;
	unsigned int interval_us;
	//Daniel 20200327 modify end
#ifdef TIMED_OUTPUT
	struct timed_output_dev to_dev;
#else
	struct led_classdev cdev;
#endif
	struct fileops fileops;
	struct ram ram;
	bool haptic_ready;
	bool audio_ready;
	int pre_haptic_number;
	struct timeval start, end;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned long int microsecond;
	unsigned int sys_frequency;
	unsigned int rtp_len;

	int reset_gpio;
	int irq_gpio;
	struct pinctrl *active_pinctrl;
	struct pinctrl_state *pinctrl_reset_state;
	struct pinctrl_state *pinctrl_int_state;

	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;

	unsigned char play_mode;

	unsigned char activate_mode;

	unsigned char auto_boost;

	int state;
	int duration;
	int amplitude;
	int index;
	int vmax;
	int gain;

	unsigned char seq[AW8697_SEQUENCER_SIZE];
	unsigned char loop[AW8697_SEQUENCER_SIZE];

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned char rtp_init;
	unsigned char ram_init;
	unsigned char rtp_routine_on;

	unsigned int f0;
	unsigned int cont_f0;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;
	unsigned int theory_time;

	unsigned char ram_vbat_comp;
	unsigned int vbat;
	unsigned int lra;
	char cali_lra;

	struct trig trig[AW8697_TRIG_NUM];

	struct haptic_audio haptic_audio;
	struct aw8697_dts_info info;
	//Daniel 20200429 modify start
	enum aw_game_dir_cmd dir;
	enum aw_ram_vib_type ram_vib_type;
	//Daniel 20200429 modify end
	unsigned int ramupdate_flag;
	unsigned int rtpupdate_flag;
    unsigned int osc_cali_run;
    unsigned int lra_calib_data;
	unsigned int f0_calib_data;
	struct notifier_block fb_notif;/*register to control tp report*/
	bool boost_en;
};

struct aw8697_container {
	int len;
	unsigned char data[];
};

/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw8697_seq_loop {
	unsigned char loop[AW8697_SEQUENCER_SIZE];
};

struct aw8697_que_seq {
	unsigned char index[AW8697_SEQUENCER_SIZE];
};

#define AW8697_HAPTIC_IOCTL_MAGIC         'h'

#define AW8697_HAPTIC_SET_QUE_SEQ         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 1, struct aw8697_que_seq*)
#define AW8697_HAPTIC_SET_SEQ_LOOP        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 2, struct aw8697_seq_loop*)
#define AW8697_HAPTIC_PLAY_QUE_SEQ        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8697_HAPTIC_SET_BST_VOL         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8697_HAPTIC_SET_BST_PEAK_CUR    _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8697_HAPTIC_SET_GAIN            _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8697_HAPTIC_PLAY_REPEAT_SEQ     _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 7, unsigned int)

extern int aw8697x_haptic_ram_vbat_comp(struct aw8697 *aw8697x, bool flag);
extern int aw8697x_haptic_stop(struct aw8697 *aw8697x);
extern void aw8697x_haptic_upload_lra(struct aw8697 *aw8697x, unsigned int flag);
extern int aw8697x_haptic_play_mode(struct aw8697 *aw8697x,
				   unsigned char play_mode);
extern int aw8697x_haptic_cont(struct aw8697 *aw8697x);
extern int aw8697x_haptic_play_go(struct aw8697 *aw8697x, bool flag);
extern void aw8697x_haptic_set_rtp_aei(struct aw8697 *aw8697x, bool flag);
extern void aw8697x_interrupt_clear(struct aw8697 *aw8697x);
extern int aw8697x_haptic_juge_RTP_is_going_on(struct aw8697 *aw8697x);
extern void aw8697x_op_clean_status(struct aw8697 *aw8697x);
extern void aw8697_double_ram_right(void);

#define AW8697_RTP_NAME_MAX        110
static char aw8697x_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697x_osc_rtp_24K_5s.bin"}, /*150 */
	{"151_AGT_RTP_x.bin"},
	{"152_Aircraft_Left_RTP.bin"}, /* 152, 0403 update */
	{"153_BaBa_RTP_x.bin"},
	{"154_Bloom_RTP_x.bin"},
	{"155_Bottle_RTP_x.bin"}, /* 155 */
	{"156_Bright_RTP_x.bin"},
	{"157_CountryGuitar_RTP_x.bin"},
	{"158_Di_RTP_x.bin"},
	{"159_Drip_RTP_x.bin"},
	{"160_Du_RTP_x.bin"}, /* 160 */
	{"161_Dudu_RTP_x.bin"},
	{"162_FleetingTime_RTP_x.bin"},
	{"163_Flowering_RTP_x.bin"},
	{"164_Fluting_RTP_x.bin"},
	{"165_FoodBell_RTP_x.bin"}, /* 165 */
	{"166_GoGo_RTP_x.bin"},
	{"167_Mechanical_Left_RTP.bin"}, /* 167, 0403 update */
	{"168_Mouse_RTP_x.bin"},
	{"169_Laser_Left_RTP.bin"},
	{"170_Ring_RTP_x.bin"}, /* 170 */
	{"171_Straight_Mallet_Left_RTP.bin"}, /* 171, 0509 update */
	{"172_Signal_RTP_x.bin"},
	{"173_Sword_Left_RTP.bin"},
	{"174_Sports_Car_Left_RTP.bin"},
	{"175_Race_Car_Left_RTP.bin"},/* 175, 0403 update */
	{"176_Stable_RTP_x.bin"},
	{"177_Strange_RTP_x.bin"},
	{"178_Stream_RTP_x.bin"},
	{"179_Success_RTP_x.bin"},
	{"180_Talk_RTP_x.bin"},/* 180 */
	{"181_Technology_Left_RTP.bin"},/* 181, 0403 update */
	{"182_Telly_RTP_x.bin"},
	{"183_tone_RTP_x.bin"},
	{"184_Tumbler_RTP_x.bin"},
	{"185_Footsteps_Left_RTP.bin"},/* 185, To be update */
	{"186_Wave_RTP_x.bin"},
	{"187_Blinking_RTP_x.bin"},
	{"188_Brisk_moment_RTP_x.bin"},
	{"189_ClassicPiano_RTP_x.bin"},
	{"190_Clear_RTP_x.bin"},/* 190 */
	{"191_DalaGuitar_RTP_x.bin"},
	{"192_DingDang_RTP_x.bin"},
	{"193_DoDaDa_RTP_x.bin"},
	{"194_Dreamtime_RTP_x.bin"},
	{"195_Early_morninng_RTP_x.bin"},/* 195 */
	{"196_Eighties_RTP_x.bin"},
	{"197_FunkyGuitar_RTP_x.bin"},
	{"198_Guitar_RTP_x.bin"},
	{"199_HappyTime_RTP_x.bin"},
	{"200_Home_RTP_x.bin"}, /* 200 */
	{"201_Jete_RTP_x.bin"},
	{"202_Kalimba_RTP_x.bin"},
	{"203_landscape_RTP_x.bin"},
	{"204_LullabyPiano_RTP_x.bin"},
	{"205_MaLing_RTP_x.bin"}, /* 205 */
	{"206_Moment_of_spirit_RTP_x.bin"},
	{"207_PianoNoise_RTP_x.bin"},
	{"208_PingPong_RTP_x.bin"},
	{"209_RainbowSweet_RTP_x.bin"},
	{"210_Travel_Left_RTP.bin"},/* 210, 0403 update */
	{"211_Soothing_Left_RTP.bin"},/* 211, 0403 update */
	{"212_Elves_Left_RTP.bin"},/* 212, 0403 update */
	{"213_Psychedelic_Left_RTP.bin"},/* 213, 0403 update */
	{"214_Dark_Left_RTP.bin"},/* 214, 0403 update */
	{"215_Promote_Left_RTP.bin"},/* 215, 0403 update */
	{"216_Lenovo_Electronic_Left_RTP.bin"},/* 216, 0403 update */
	{"217_Lenovo_Gaming_Left_RTP.bin"},/* 217, 0403 update */
	{"218_Lenovo_Guitar_Left_RTP.bin"},/* 218, 0403 update */
	{"219_Lenovo_Dynamic_Left_RTP.bin"},/* 219, 0403 update */
	{"220_Lenovo_Pure_Tone_Left_RTP.bin"},/* 220, 0403 update */
	{"221_Lenovo_Symphony_Left_RTP.bin"},/* 221, 0403 update */
	{"222_Teleport_Left_RTP.bin"},/* 222, 0403 update */
	{"223_GameMoment_Left_RTP.bin"},
	{"224_Silence_RTP_x.bin"},
	{"Ding_RTP_x_23.bin"},/* 225, To be update */
	{"226_Stars_RTP_x.bin"},
	{"227_SunnyDay_RTP_x.bin"},
	{"228_Heartbeat_RTP_x.bin"},/* 228, To be update */
	{"Ding_RTP_x_23.bin"},/* 229, To be update */
	{"Ding_RTP_x_23.bin"},/* 230, To be update */
	{"231_Awoken_RTP_x.bin"},
	{"232_EtherealMallets_RTP_x.bin"},
	{"233_Inception_RTP_x.bin"},
	{"234_Lullaby_RTP_x.bin"},
	{"235_Potpourri_RTP_x.bin"},/* 235 */
	{"236_Respire_RTP_x.bin"},
	{"237_Legion_RTP_x.bin"},
	{"Ding_RTP_x_23.bin"},
	{"239_SecretGarden_RTP_x.bin"},
	{"240_Streamer_RTP_x.bin"},/* 240 */
	{"241_Pulse_RTP_x.bin"},
	{"242_Starlight_RTP_x.bin"},
	{"243_Light_RTP_x.bin"},
	{"Ding_RTP_23.bin"},
	{"Ding_RTP_23.bin"}, /* 245 */
	{"Ding_RTP_23.bin"},
	{"Ding_RTP_23.bin"},
	{"Ding_RTP_23.bin"},
	{"Ding_RTP_23.bin"},
	{"Ding_RTP_23.bin"}, /* 250 */
	{"251_CQA_RTP_x.bin"},
};

#endif
