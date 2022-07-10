#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include "cam_debug_util.h"
#include <linux/pinctrl/pinctrl.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include "cam_sensor_io.h"
#include "mxm1120.h"
#include <linux/interrupt.h>
#include <asm-generic/io.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/input.h>

#define ENABLE_HALL_VIO
#define USE_GPIO_CLK
//#define GPIO_TEST


static int major;
static struct class *motor_class;
#define MOTOR_STATE_DEFAULT "cam_default"
#define MOTOR_STATE_SUSPEND "cam_suspend"
#define HALL_SLAVE_1 0x1E
#define HALL_SLAVE_2 0x18
#define LDO_ADRESS 0x29
#define M1120_VDD_MIN_UV       2960000
#define M1120_VDD_MAX_UV       2960000
#define M1120_VIO_MIN_UV       1800000
#define M1120_VIO_MAX_UV       1800000
#define CLK_GPIO_PIN 108

#ifndef USE_GPIO_CLK
struct clk *pclk;
#else
struct tasklet_struct clk_tasklet;
#endif

struct motor_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	bool use_pinctrl;
};
struct hall_i2c_data_t{
    uint32_t addr;
    uint32_t data;
};


struct gpio *cam_gpio_req_tbl = NULL;
enum{
    MOTOR_DEV_INIT = 0,
    MOTOR_DEV_FORWARD,
    MOTOR_DEV_BACK = 3,
    MOTOR_DEV_RELEASE,
    MOTOR_HAL_DEV_INIT,
    MOTOR_HAL_DEV_UNINIT,
    MOTOR_HAL_DEV_READ_HALL1,
    MOTOR_HAL_DEV_READ_HALL2,
    MOTOR_HAL_DEV_WRITE_HALL1,
    MOTOR_HAL_DEV_WRITE_HALL2,
    MOTOR_HAL_DEV_LDO_TEST,
    MOTOR_HAL_DEV_REC_CALI,
    MOTOR_HAL_DEV_INT_RST,
    MOTOR_DEV_WRITE_MSG,
    MOTOR_DEV_READ_MSG,
    MOTOR_DEV_REPORT_KEY,
};
enum{
    MOTOR_NORMAL,
    MOTOR_EJECT_ERROR,
    MOTOR_PRESS_ERROR,
    MOTOR_FALL_ERROR,
};
enum{
    MOTOR_SLEEP = 0,
    MOTOR_ENABLE,
    MOTOR_DIR,
    MOTOR_MODE0,
    MOTOR_MODE1,
    MOTOR_T0,
    MOTOR_T1,
    MOTOR_PWR,
#ifdef USE_GPIO_CLK
    GPIO_CLK,
#endif
    HALL_INT2,
    HALL_INT1,
};
enum hall_device{
    hall_device1,
    hall_device2,
};

struct motor_pinctrl_info motor_pinctrl;
struct camera_io_master io_master_info[2];
m1120_data_t m1120;
atomic_t v = ATOMIC_INIT(0);
atomic_t hall_int_enable = ATOMIC_INIT(0);
struct workqueue_struct *int_wq = NULL;
struct input_dev *input_hall_dev = NULL;
int pwm_time = 195;
struct fasync_struct *motor_sync;
int error_message = 0;
struct hrtimer gpio_timer;
int step_count;



void motor_power_down(void);
int motor_hall_write_i2c(enum hall_device device,uint32_t addr,uint32_t data);
int motor_hall_read_i2c(enum hall_device device, struct hall_i2c_data_t *i2c_data);
int mx1120_init(void);
int mx1120_uninit(void);
int calculateposition(void);
void step_clk_stop(void);




#ifdef USE_GPIO_CLK
int gpio_clk_enable;
enum hrtimer_restart gpio_clk_calibration(struct hrtimer *timer){
    //CAM_INFO(CAM_MOTOR,"gpio_clk start allow step:%d",allow_step);
    hrtimer_forward(timer,timer->base->get_time(),pwm_time*1000);
    if( step_count%2 == 0){
        gpio_set_value(cam_gpio_req_tbl[GPIO_CLK].gpio, 1);
    }else{
        gpio_set_value(cam_gpio_req_tbl[GPIO_CLK].gpio, 0);
    }
    step_count++;
    if(gpio_clk_enable){
        return HRTIMER_RESTART;
    }
    return HRTIMER_NORESTART;
}
void gpio_clk(struct work_struct *work){
    int i = 0;
    int allow_step = 3500;
    int timeout_count = 0;
    if(m1120.bcali){
        allow_step = 200000;
    }
    CAM_INFO(CAM_MOTOR,"gpio_clk start allow step:%d",allow_step);
    for(i = 0; i< allow_step;i++){
        //mutex_lock(&m1120.i2c_mutex);
        if(atomic_read(&v) ==0){
            timeout_count = 0;
        }else{
            timeout_count++;
        }

        if(timeout_count>20){
            i--;
            continue;
        }
        //atomic_inc(&v);
        if( i%2 == 0){
            gpio_set_value(cam_gpio_req_tbl[GPIO_CLK].gpio, 1);
        }else{
            gpio_set_value(cam_gpio_req_tbl[GPIO_CLK].gpio, 0);
        }
            //atomic_dec(&v);
        
        //mutex_unlock(&m1120.i2c_mutex);
        if(!gpio_clk_enable){
            break;
        }
        udelay(pwm_time);
    }
    motor_power_down();
    CAM_ERR(CAM_MOTOR,"gpio_clk end");
}
DECLARE_WORK(clk_work,gpio_clk);
#endif
void calcupos_thread(struct work_struct *work){
    int i = 0;
    int pos;
    int ret = 0;
    m1120.bret = false;
    for(i = 0;i < 100; i++){  
        pos = calculateposition();
        if(pos == -2000){
            continue;
        }
        if(m1120.bdirection){
            if(pos > 970){
                msleep(30);
                motor_power_down();
                step_clk_stop();
                ret = 1;
                break;
            }
        }else{
            if(pos <15){
                msleep(25);
                motor_power_down();
                step_clk_stop();
                //mx1120_uninit();
                ret = 1;
                break;
            }
        }
        
        msleep(13);
    }
    if(ret == 0){
        m1120.bret = true;
    }else{
        m1120.bret = false;
    }
    up(&m1120.finish_sem);
    motor_power_down();
    step_clk_stop();
     
}
DECLARE_WORK(cal_work,calcupos_thread);

void step_clk_start()
{

#ifndef USE_GPIO_CLK
    int32 ret;
    CAM_ERR(CAM_MOTOR, "%s: clk_prepare start prepare", __func__);
    ret = clk_set_rate(pclk, 25000000);
    
    CAM_ERR(CAM_MOTOR,"prepare end");
    if (ret){
        CAM_ERR(CAM_MOTOR, "clk set rate fail, ret = %d\n", ret);
    }
    ret = clk_prepare_enable(pclk);
    if (ret){
        CAM_ERR(CAM_MOTOR, "%s: clk_prepare error!!!\n", __func__);
    }
    else{
        CAM_ERR(CAM_MOTOR,"%s: clk_prepare success!\n", __func__);
    }
#else
    CAM_INFO(CAM_MOTOR, "%s: start schedule clk_tasklet", __func__);
    gpio_clk_enable = 1;
    if(m1120.bcali){
        schedule_work(&cal_work);
        step_count = 0;
        gpio_timer.function = gpio_clk_calibration;
        hrtimer_start(&gpio_timer,pwm_time*1000,HRTIMER_MODE_REL);
    }else{
    	schedule_work(&clk_work);
	}
    //tasklet_schedule(&clk_tasklet);
#endif
}
void step_clk_stop(){
#ifndef USE_GPIO_CLK
    clk_disable_unprepare(pclk);
#else
    gpio_clk_enable = 0;
#endif

}
int calculateposition(){
    int8_t H2,H1;
    int position = 0;
    int ret;
    struct hall_i2c_data_t i2c_data;
    i2c_data.addr = 0x11;
    //CAM_INFO(CAM_MOTOR,"start");
    //mutex_lock(&m1120.i2c_mutex);
    //CAM_INFO(CAM_MOTOR,"start1");
    atomic_inc(&v);
    ret = motor_hall_read_i2c(hall_device1,&i2c_data);
    if(ret < 0){
        position = -2000;
    }
    H1 = (int8_t)i2c_data.data;
    ret = motor_hall_read_i2c(hall_device2,&i2c_data);
    if(ret < 0){
        position = -2000;
    }
    atomic_dec(&v);
    H2 = (int8_t)i2c_data.data;
    //mutex_unlock(&m1120.i2c_mutex);
    CAM_INFO(CAM_MOTOR,"H1:%d,H2:%d",(int)H1,(int)H2);
    CAM_INFO_RATE_LIMIT(CAM_MOTOR,"C:%d,D:%d,B:%d,A:%d,CDBA:%d,H2H1BA:%d",
        m1120.calidata.C,m1120.calidata.D,m1120.calidata.B,m1120.calidata.A,
        m1120.calidata.C-m1120.calidata.D-m1120.calidata.B+m1120.calidata.A,
        ((int)H2-(int)H1)-(m1120.calidata.B-m1120.calidata.A));
    if(position != -2000){
        position = 1000*(((int)H2-(int)H1)-(m1120.calidata.B-m1120.calidata.A))/(m1120.calidata.C-m1120.calidata.D-m1120.calidata.B+m1120.calidata.A);
    }
    CAM_INFO(CAM_MOTOR,"position:%d",position);
    return position;
}
irqreturn_t handleHallInt1(int irq, void *data){
    //int ret;
    //uint32_t dat;
    //ret = camera_io_dev_read(&io_master_info[1], M1120_REG_INTSRS , &dat, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    if((atomic_read(&hall_int_enable) == 1) && m1120.bcali){
        CAM_INFO(CAM_MOTOR,"report key");
        kill_fasync(&motor_sync, SIGIO, POLL_IN);
        error_message |= 1 << MOTOR_PRESS_ERROR;
    }else{        
        CAM_INFO(CAM_MOTOR,"not care");
    }
    return IRQ_HANDLED;
}

static int m1120_power_ctl(m1120_data_t *data, bool on)
{
    int ret = 0;

#ifdef ENABLE_HALL_VIO
    int err = 0;
#endif

    if (!on && data->power_enabled) {
        ret = regulator_disable(data->vdd);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
                "Regulator vdd disable failed ret=%d\n", ret);
            return ret;
        }
#ifdef ENABLE_HALL_VIO
        ret = regulator_disable(data->vio);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
                "Regulator vio disable failed ret=%d\n", ret);
            err = regulator_enable(data->vdd);
            return ret;
        }
#endif
        data->power_enabled = on;
    } else if (on && !data->power_enabled) {
        ret = regulator_enable(data->vdd);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
                "Regulator vdd enable failed ret=%d\n", ret);
            return ret;
        }
#ifdef ENABLE_HALL_VIO
              msleep(8);////>=5ms OK.
        ret = regulator_enable(data->vio);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
                "Regulator vio enable failed ret=%d\n", ret);
            err = regulator_disable(data->vdd);
            return ret;
        }
#endif
        msleep(10); // wait 10ms
        data->power_enabled = on;
    } else {
        CAM_ERR(CAM_MOTOR,
                "Power on=%d. enabled=%d\n",
                on, data->power_enabled);
    }

    return ret;
}

static int m1120_power_init(m1120_data_t *data)
{
    int ret;

    data->vdd = regulator_get(data->dev, "vdd");
    if (IS_ERR(data->vdd)) {
        ret = PTR_ERR(data->vdd);
        CAM_ERR(CAM_MOTOR,
            "Regulator get failed vdd ret=%d\n", ret);
        return ret;
    }

    if (regulator_count_voltages(data->vdd) > 0) {
        ret = regulator_set_voltage(data->vdd,
                M1120_VDD_MIN_UV,
                M1120_VDD_MAX_UV);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
                "Regulator set failed vdd ret=%d\n",
                ret);
            goto reg_vdd_put;
        }
    }
#ifdef ENABLE_HALL_VIO
    data->vio = regulator_get(data->dev, "vio");
    if (IS_ERR(data->vio)) {
        ret = PTR_ERR(data->vio);
        CAM_ERR(CAM_MOTOR,
            "Regulator get failed vio ret=%d\n", ret);
        goto reg_vdd_set;
    }

    if (regulator_count_voltages(data->vio) > 0) {
        ret = regulator_set_voltage(data->vio,
                M1120_VIO_MIN_UV,
                M1120_VIO_MAX_UV);
        if (ret) {
            CAM_ERR(CAM_MOTOR,
            "Regulator set failed vio ret=%d\n", ret);
            goto reg_vio_put;
        }
    }
#endif
    data->power_enabled = false;
    return 0;
    
#ifdef ENABLE_HALL_VIO
reg_vio_put:
    regulator_put(data->vio);
reg_vdd_set:
    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, M1120_VDD_MAX_UV);
#endif
reg_vdd_put:
    regulator_put(data->vdd);
    return ret;
}
int motor_power_up(bool direction,int speed){
    
    int ret = pinctrl_select_state(motor_pinctrl.pinctrl,
             motor_pinctrl.gpio_state_active);
    if(speed > 3){
        speed = 3;
    }
#ifdef GPIO_TEST
    void __iomem *gpio_mem = NULL;
    u32 data = 0;
    if(!request_mem_region(0x0F96C000,4,"gpio108")){
         CAM_ERR(CAM_MOTOR,
                "request mem error");
         return -1;
    }
    gpio_mem =ioremap_nocache(0x0F96C000,4);
    data = readl_relaxed(gpio_mem);
    CAM_ERR(CAM_MOTOR,
                "read gpio register = 0x%x\n",
                data);
    iounmap(gpio_mem);
    release_mem_region(0x0F96C000,4);
#endif
    
    //ret = mx1120_init();
    if(ret){
         CAM_ERR(CAM_MOTOR,"mx1120 init fail");
         return ret;
    }

    gpio_set_value(cam_gpio_req_tbl[MOTOR_PWR].gpio, 1);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_ENABLE].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_SLEEP].gpio, 1);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_MODE0].gpio, speed & 0x1);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_MODE1].gpio, (speed >> 1) & 0x1);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_T0].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_T1].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_DIR].gpio, direction);
    return ret;
}
void motor_power_down(){
    
    gpio_set_value(cam_gpio_req_tbl[MOTOR_PWR].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_ENABLE].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_SLEEP].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_DIR].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_MODE0].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_MODE1].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_T0].gpio, 0);
    gpio_set_value(cam_gpio_req_tbl[MOTOR_T1].gpio, 0);
}

int mx1120_init(){
    int ret;
    uint32_t data;
    io_master_info[0].master_type = CCI_MASTER;
    io_master_info[0].client = NULL;
    
    io_master_info[0].cci_client->cci_device = CCI_DEVICE_0;
    io_master_info[0].cci_client->cci_i2c_master = MASTER_1;
	io_master_info[0].cci_client->i2c_freq_mode = I2C_FAST_MODE;
	io_master_info[0].cci_client->sid = HALL_SLAVE_1 >> 1;
    
    
    ret = camera_io_init(&io_master_info[0]);
    if(ret){
        CAM_ERR(CAM_MOTOR,"motor cci init fail");
        return ret;
    }
    ret = camera_io_dev_read(&io_master_info[0], 0x00 , &data, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    if(ret){
        ret = camera_io_release(&io_master_info[0]);
        CAM_ERR(CAM_MOTOR,"motor cci read fail");
        return -1;
    }
    CAM_INFO(CAM_MOTOR,"motor cci[0] read %d",data);
#if 0
    ret = camera_io_release(&io_master_info);

    if(ret){
        CAM_ERR(CAM_MOTOR,"motor cci release error");
        return ret;
    }
#endif
    io_master_info[1].master_type = CCI_MASTER;
    io_master_info[1].client = NULL;
    
    io_master_info[1].cci_client->cci_device = CCI_DEVICE_0;
    io_master_info[1].cci_client->cci_i2c_master = MASTER_1;
	io_master_info[1].cci_client->i2c_freq_mode = I2C_FAST_MODE;
	io_master_info[1].cci_client->sid = HALL_SLAVE_2 >> 1;
    ret = camera_io_init(&io_master_info[1]);
    if(ret){
        CAM_ERR(CAM_MOTOR,"motor cci init fail");
        return ret;
    }
    ret = camera_io_dev_read(&io_master_info[1], 0x00 , &data, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    if(ret){
        ret = camera_io_release(&io_master_info[1]);
        CAM_ERR(CAM_MOTOR,"motor cci read fail");
        return -1;
    }
    CAM_INFO(CAM_MOTOR,"motor cci[1] read %d",data);
    
    motor_hall_write_i2c(hall_device1, M1120_REG_PERSINT,M1120_VAL_PERSINT_COUNT(0)|M1120_VAL_PERSINT_INTCLR);
    motor_hall_write_i2c(hall_device1, M1120_REG_INTSRS,M1120_VAL_INTSRS_INT_ON|M1120_VAL_INTSRS_INTTYPE_WITHIN|M1120_VAL_INTSRS_SRS_8BIT_0_136mT);
    motor_hall_write_i2c(hall_device1, M1120_REG_OPF,M1120_VAL_OPF_FREQ_80HZ|M1120_VAL_OPF_BIT_8|M1120_VAL_OPF_HSSON_ON);
    if(m1120.calidata.A > 0){
        motor_hall_write_i2c(hall_device1, M1120_REG_LTHL,m1120.calidata.A/10);
        motor_hall_write_i2c(hall_device1, M1120_REG_LTHH,0);
        if(m1120.bcali == true){
            motor_hall_write_i2c(hall_device1, M1120_REG_HTHL,m1120.calidata.A*7/10);
        }
        else
        {
            motor_hall_write_i2c(hall_device1, M1120_REG_HTHL,60);
        }
        motor_hall_write_i2c(hall_device1, M1120_REG_HTHH,0);
        CAM_INFO(CAM_MOTOR,"cam calibration > 0");
    }else{
        motor_hall_write_i2c(hall_device1, M1120_REG_LTHL,m1120.calidata.A*7/10);
        motor_hall_write_i2c(hall_device1, M1120_REG_LTHH,0xC0);
        if(m1120.bcali == true){
            motor_hall_write_i2c(hall_device1, M1120_REG_HTHL,m1120.calidata.A/10);
        }
        else
        {
            motor_hall_write_i2c(hall_device1, M1120_REG_HTHL,60);
        }
        motor_hall_write_i2c(hall_device1, M1120_REG_HTHH,0xC0);
        CAM_INFO(CAM_MOTOR,"cam calibration < 0");
    }
    ret = camera_io_dev_read(&io_master_info[0], 0x00 , &data, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    CAM_INFO(CAM_MOTOR,"motor cci[0] read %d",data);

    motor_hall_write_i2c(hall_device2,M1120_REG_PERSINT,M1120_VAL_PERSINT_COUNT(0)|M1120_VAL_PERSINT_INTCLR);
    motor_hall_write_i2c(hall_device2,M1120_REG_INTSRS,M1120_VAL_INTSRS_INT_ON|M1120_VAL_INTSRS_INTTYPE_WITHIN|M1120_VAL_INTSRS_SRS_8BIT_0_136mT);
    motor_hall_write_i2c(hall_device2,M1120_REG_OPF,M1120_VAL_OPF_FREQ_80HZ|M1120_VAL_OPF_BIT_8|M1120_VAL_OPF_HSSON_ON);
    //motor_hall_write_i2c(hall_device2, M1120_REG_LTHL,10);
    //motor_hall_write_i2c(hall_device2, M1120_REG_LTHH,0);
    //motor_hall_write_i2c(hall_device2, M1120_REG_HTHL,60);
    //motor_hall_write_i2c(hall_device2, M1120_REG_HTHH,0);
    if(m1120.calidata.C > 0){
        motor_hall_write_i2c(hall_device2, M1120_REG_LTHL,m1120.calidata.C/10);
        motor_hall_write_i2c(hall_device2, M1120_REG_LTHH,0);
        if(m1120.bcali == true){
            motor_hall_write_i2c(hall_device2, M1120_REG_HTHL,m1120.calidata.C*7/10);
        }
        else
        {
            motor_hall_write_i2c(hall_device2, M1120_REG_HTHL,60);
        }
        motor_hall_write_i2c(hall_device2, M1120_REG_HTHH,0);
        CAM_INFO(CAM_MOTOR,"cam calibration > 0");
    }else{
        motor_hall_write_i2c(hall_device2, M1120_REG_LTHL,m1120.calidata.C*7/10);
        motor_hall_write_i2c(hall_device2, M1120_REG_LTHH,0xC0);
        if(m1120.bcali == true){
            motor_hall_write_i2c(hall_device2, M1120_REG_HTHL,m1120.calidata.C/10);
        }
        else
        {
            motor_hall_write_i2c(hall_device2, M1120_REG_HTHL,60);
        }
        motor_hall_write_i2c(hall_device2, M1120_REG_HTHH,0xC0);
        CAM_INFO(CAM_MOTOR,"cam calibration < 0");
    }
    gpio_direction_input(cam_gpio_req_tbl[HALL_INT2].gpio);
    gpio_direction_input(cam_gpio_req_tbl[HALL_INT1].gpio);
    
    return ret;
}
int mx1120_uninit(){
    gpio_direction_output(cam_gpio_req_tbl[HALL_INT2].gpio, 0);
    gpio_direction_output(cam_gpio_req_tbl[HALL_INT1].gpio, 0);
    camera_io_release(&io_master_info[0]);
    camera_io_release(&io_master_info[1]);
    //kfree(io_master_info[0].cci_client);
    //kfree(io_master_info[1].cci_client);
    return 0;
}
int motor_fasync(int fd, struct file *filp, int on){
    return fasync_helper(fd,filp,on,&motor_sync);
}

static int motor_open (struct inode *node, struct file *filp)
{
    CAM_INFO(CAM_MOTOR,"motor open");
    return 0;
}
int LDO_init(){
    int ret;
    uint32_t data;
    struct camera_io_master io_master_info;
    io_master_info.master_type = CCI_MASTER;
    io_master_info.client = NULL;
    io_master_info.cci_client = kcalloc(1, sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
    io_master_info.cci_client->cci_device = CCI_DEVICE_1;
    io_master_info.cci_client->cci_i2c_master = MASTER_1;
	io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;
	io_master_info.cci_client->sid = LDO_ADRESS;
    ret = camera_io_init(&io_master_info);
    if(ret){
        CAM_ERR(CAM_MOTOR,"motor cci init fail");
        return -1;
    }
    ret = camera_io_dev_read(&io_master_info, 0x00 , &data, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    if(ret){
        ret = camera_io_release(&io_master_info);
        CAM_ERR(CAM_MOTOR,"motor cci read fail %d",ret);
        return -1;
    }
    CAM_INFO(CAM_MOTOR,"motor ldo read %d",data);
    camera_io_release(&io_master_info);
    return ret;
}

static ssize_t motor_write (struct file *filp, const char __user *buf, size_t size, loff_t *off)
{
    return 0;
}

static int motor_release (struct inode *node, struct file *filp)
{
    fasync_helper(0,filp,0,&motor_sync);
    return 0;
}
int motor_hall_read_i2c(enum hall_device device, struct hall_i2c_data_t *i2c_data){
    int ret = camera_io_dev_read(&io_master_info[device], i2c_data->addr, &i2c_data->data, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
    //CAM_INFO(CAM_MOTOR,"read[%d]:%d",i2c_data->addr,i2c_data->data);
    return ret;
}
int motor_hall_write_i2c(enum hall_device device,uint32_t addr,uint32_t data){
    struct cam_sensor_i2c_reg_setting camera_i2c_setting;
    int32_t ret = 0;
    camera_i2c_setting.reg_setting = kcalloc(1, sizeof(struct cam_sensor_i2c_reg_array), GFP_KERNEL);
    camera_i2c_setting.reg_setting->reg_addr = addr;
    camera_i2c_setting.reg_setting->reg_data = data;
    camera_i2c_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    camera_i2c_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    camera_i2c_setting.reg_setting->delay = 0;
    camera_i2c_setting.reg_setting->data_mask = 0;
    camera_i2c_setting.delay = 0;
    camera_i2c_setting.size = 1; 
    camera_i2c_setting.read_buff = NULL;
    camera_i2c_setting.read_buff_len = 0;
    ret = camera_io_dev_write(&io_master_info[device],&camera_i2c_setting);
    //CAM_INFO(CAM_MOTOR,"ret:%d",ret);
    kfree(camera_i2c_setting.reg_setting);
    return ret;
}
long motor_compat_ioctl (struct file *fp, unsigned int cmd, unsigned long arg){
    int ret = 0;
    struct hall_i2c_data_t * i2c_hall_data = kcalloc(1,sizeof(struct hall_i2c_data_t),GFP_KERNEL);
    CAM_ERR(CAM_MOTOR,"enter ioctl %d",cmd);
    switch(cmd){
        case MOTOR_DEV_INIT:
        break;
        case MOTOR_DEV_FORWARD:
            if(arg != 0){
                pwm_time = arg;
            }
            m1120.bdirection = true;
            //m1120_power_ctl(&m1120,true);
            ret = motor_power_up(true,0);
            step_clk_start();
            if(m1120.bcali){
                down(&m1120.finish_sem);
                if(m1120.bret){
                    ret = -1;
                }
                CAM_INFO(CAM_MOTOR,"return");
            }
        break;
        case MOTOR_DEV_BACK:
            if(arg != 0){
                pwm_time = arg;
            }
            m1120.bdirection = false;
            ret = motor_power_up(false, 0);
            step_clk_start();
            if(m1120.bcali){
                down(&m1120.finish_sem);
                CAM_INFO(CAM_MOTOR,"return");
                if(m1120.bret){
                    ret = -1;
                }
            }
            //m1120_power_ctl(&m1120,false);
        break;
        case MOTOR_DEV_RELEASE:
            step_clk_stop();
            motor_power_down();
        break;
        case MOTOR_HAL_DEV_INIT:
            m1120_power_ctl(&m1120,true);
            if(m1120.binit == false){
                ret = mx1120_init();
            }
            if(ret){
                CAM_ERR(CAM_MOTOR,"mx1120 init fail");
                return ret;
            }
            m1120.binit = true;
        break;
        case MOTOR_HAL_DEV_UNINIT:
            mx1120_uninit();
            m1120.binit = false;
            m1120_power_ctl(&m1120,false);
        break;
        case MOTOR_HAL_DEV_READ_HALL1:
            copy_from_user((void *)i2c_hall_data,(void *)arg,sizeof(struct hall_i2c_data_t));
            motor_hall_read_i2c(hall_device1,i2c_hall_data);
            copy_to_user((void *)arg,(void *)i2c_hall_data,sizeof(struct hall_i2c_data_t));
        break;
        case MOTOR_HAL_DEV_READ_HALL2:
            copy_from_user((void *)i2c_hall_data,(void *)arg,sizeof(struct hall_i2c_data_t));
            motor_hall_read_i2c(hall_device2,i2c_hall_data);
            copy_to_user((void *)arg,(void *)i2c_hall_data,sizeof(struct hall_i2c_data_t));
        break;
        case MOTOR_HAL_DEV_WRITE_HALL1:
            copy_from_user((void *)i2c_hall_data,(void *)arg,sizeof(struct hall_i2c_data_t));
            ret = motor_hall_write_i2c(hall_device1,i2c_hall_data->addr,i2c_hall_data->data);
        break;
        case MOTOR_HAL_DEV_WRITE_HALL2:
            copy_from_user((void *)i2c_hall_data,(void *)arg,sizeof(struct hall_i2c_data_t));
            ret = motor_hall_write_i2c(hall_device2,i2c_hall_data->addr,i2c_hall_data->data);
        break;
        case MOTOR_HAL_DEV_LDO_TEST:
            m1120_power_ctl(&m1120,true);
            ret = LDO_init();
            m1120_power_ctl(&m1120,false);
        break;
        case MOTOR_HAL_DEV_REC_CALI:
            if(arg == 0){
                m1120.bcali = false;
                break;
            }
            copy_from_user((void *)(&m1120.calidata),(void *)arg,sizeof(struct MotorCalidata));
            CAM_INFO(CAM_MOTOR,"received A:%d,B:%d,C:%d,D:%d",
            m1120.calidata.A,m1120.calidata.B,m1120.calidata.C,m1120.calidata.D);
            m1120.bcali = true;
            if(m1120.binit == false){
                m1120_power_ctl(&m1120,true);
                ret = mx1120_init();
                if(ret){
                    CAM_ERR(CAM_MOTOR,"mx1120 init fail");
                }
                mx1120_uninit();
                m1120_power_ctl(&m1120,false);
            }
        break;
        case MOTOR_HAL_DEV_INT_RST:
            atomic_set(&hall_int_enable,(int)arg);
            ret = motor_hall_write_i2c(hall_device1, M1120_REG_PERSINT,M1120_VAL_PERSINT_COUNT(0)|M1120_VAL_PERSINT_INTCLR);
            if(ret < 0){
               CAM_ERR(CAM_MOTOR,"reset hall1 device INT error");
               break;
            }
            ret = motor_hall_write_i2c(hall_device2, M1120_REG_PERSINT,M1120_VAL_PERSINT_COUNT(0)|M1120_VAL_PERSINT_INTCLR);
            if(ret < 0){
               CAM_ERR(CAM_MOTOR,"reset hall2 device INT error"); 
               break;
            }
        break;
        case MOTOR_DEV_READ_MSG:
            ret = error_message;
            error_message = 0;
        break;
        case MOTOR_DEV_WRITE_MSG:
            error_message |= 1 << arg;
            kill_fasync(&motor_sync, SIGIO, POLL_IN);
        break;
        case MOTOR_DEV_REPORT_KEY:
            input_report_key(input_hall_dev,KEY_CAMHAL_HOME,1);
            input_sync(input_hall_dev);
            input_report_key(input_hall_dev,KEY_CAMHAL_HOME,0);
            input_sync(input_hall_dev);
        break;
        default:
        break;
    }
    kfree(i2c_hall_data);
    return ret;
}
__poll_t motor_poll(struct file *file, struct poll_table_struct *wait){

    //poll_wait(file,wait_queue_head_t * wait_address,wait);
    return POLLIN|POLLOUT;
}



static struct file_operations motor_oprs = {
    .owner = THIS_MODULE,
    .open  = motor_open,
    .write = motor_write,
    .release = motor_release,
    .poll = motor_poll,
    .compat_ioctl = motor_compat_ioctl,
    .unlocked_ioctl = motor_compat_ioctl,
    .fasync = motor_fasync,
};


static int motor_probe(struct platform_device *pdev)
{
    struct device_node* of_node = pdev->dev.of_node;
    
    uint32_t gpio_array_size = 0 ,i;
    uint32_t count = 0;
	uint32_t *val_array = NULL;
    int32_t rc = 0;
    
    CAM_ERR(CAM_MOTOR,"motor probe start");
    motor_pinctrl.pinctrl = NULL;
    motor_pinctrl.pinctrl = devm_pinctrl_get(&(pdev->dev));
    if (motor_pinctrl.pinctrl ==NULL) {
         CAM_DBG(CAM_MOTOR, "Getting pinctrl handle failed");
         return -EINVAL;
    }
    motor_pinctrl.gpio_state_active =
        pinctrl_lookup_state(motor_pinctrl.pinctrl,
        MOTOR_STATE_DEFAULT);
    if (motor_pinctrl.gpio_state_active == NULL) {
        CAM_ERR(CAM_MOTOR,
        "Failed to get the active state pinctrl handle");
        return -EINVAL;
    }
    motor_pinctrl.gpio_state_suspend
        = pinctrl_lookup_state(motor_pinctrl.pinctrl,
        MOTOR_STATE_SUSPEND);
    if (motor_pinctrl.gpio_state_suspend == NULL) {
         CAM_ERR(CAM_MOTOR,
            "Failed to get the suspend state pinctrl handle");
         return -EINVAL;
         }
    
                
    
    gpio_array_size = of_gpio_count(of_node);
    if (gpio_array_size <= 0){
         CAM_ERR(CAM_MOTOR,"array_size == 0 return");
         return 0;
    }
    
    CAM_INFO(CAM_MOTOR, "gpio count %d", gpio_array_size); 
    
    cam_gpio_req_tbl = kcalloc(gpio_array_size, sizeof(struct gpio), GFP_KERNEL);
    if (cam_gpio_req_tbl == NULL){
        CAM_ERR(CAM_MOTOR,"alloc req_tbl fail");
        return 0;
    }
    
    for(i = 0;i<gpio_array_size;i++){
         cam_gpio_req_tbl[i].gpio = of_get_gpio(of_node, i);
         CAM_INFO(CAM_MOTOR,"gpio:%d",cam_gpio_req_tbl[i].gpio);
    }
    count = gpio_array_size;
    
    val_array = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
    rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
              val_array, count);
    if(rc){
        CAM_ERR(CAM_MOTOR,"read gpio-req-tbl-flags fail %d",rc);
        return 0;
    }
    
    for(i = 0;i<gpio_array_size;i++){
        cam_gpio_req_tbl[i].flags = val_array[i];
    }
    
    for (i = 0; i < gpio_array_size; i++) {
        rc = of_property_read_string_index(of_node,
            "gpio-req-tbl-label", i,
            &cam_gpio_req_tbl[i].label);
        if (rc) {
            CAM_ERR(CAM_MOTOR, "Failed rc %d", rc);
            break;
        }
        CAM_INFO(CAM_MOTOR, "cam_gpio_req_tbl[%d].label = %s", i,
        cam_gpio_req_tbl[i].label);
    }
    if(rc){
        CAM_ERR(CAM_MOTOR,"gpio parse dts fail");
        return 0;
    }
    //cam_gpio_req_tbl[gpio_array_size - 1].flags = GPIOF_OPEN_DRAIN;
    for(i = 0; i < gpio_array_size; i++){
        rc = gpio_request_one(cam_gpio_req_tbl[i].gpio,cam_gpio_req_tbl[i].flags,cam_gpio_req_tbl[i].label);
        if(rc < 0){
            CAM_ERR(CAM_MOTOR,"cam_request error(gpio:%d,label[%s])",cam_gpio_req_tbl[i].gpio,cam_gpio_req_tbl[i].label);
        }
    }
    gpio_direction_input(cam_gpio_req_tbl[HALL_INT2].gpio);
    gpio_direction_input(cam_gpio_req_tbl[HALL_INT1].gpio);
    request_irq(gpio_to_irq(cam_gpio_req_tbl[HALL_INT2].gpio),handleHallInt1,IRQF_SHARED|IRQF_TRIGGER_FALLING,"CAM_MOTOR_HALL1",&(pdev->dev));
#ifndef USE_GPIO_CLK
    //probe pwm clock
    pclk = devm_clk_get(&pdev->dev, "cam-motor-clk");
    if(pclk == NULL){
        CAM_ERR(CAM_MOTOR,"clk_get NULL");
    }
#else
    hrtimer_init(&gpio_timer, CLOCK_MONOTONIC,
			 HRTIMER_MODE_REL);
    
    //tasklet_init(&clk_tasklet,gpio_clk,(unsigned long)NULL);
    //tasklet_enable(&clk_tasklet);
#endif

    //hall regulator
    m1120.dev = &pdev->dev;
    m1120_power_init(&m1120);
    m1120.bcali = false;
    m1120.bret  = false;
    m1120.binit = false;
    mutex_init(&m1120.i2c_mutex);
    sema_init(&m1120.finish_sem,0);
    io_master_info[0].cci_client = kcalloc(1, sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
    io_master_info[1].cci_client = kcalloc(1, sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
    //int_wq = create_workqueue
    
    major = register_chrdev(0, "cammotor", &motor_oprs);
    motor_class = class_create(THIS_MODULE, "cammotor");
    device_create(motor_class, NULL, MKDEV(major, 0), NULL, "cammotor"); 
    kfree(val_array);

    if(!(input_hall_dev = input_allocate_device())){
        CAM_ERR(CAM_MOTOR,"input device allocate fail");
        return 0;
    }
    input_hall_dev->name = "input_cam_hall";
    input_hall_dev->phys = "input_cam_hall/input0";
    input_hall_dev->id.bustype = BUS_HOST;
    input_hall_dev->id.vendor = 0x0001;
    input_hall_dev->id.product = 0x0002;
    input_hall_dev->id.version = 0x0100;
    input_hall_dev->evbit[0] = BIT_MASK(EV_KEY);

    set_bit(KEY_CAMHAL_HOME,input_hall_dev->keybit);
    rc = input_register_device(input_hall_dev);
    if (rc){
        CAM_ERR(CAM_MOTOR,"register input device error");
    }
    return 0;
}

static int motor_remove(struct platform_device *pdev)
{
#ifdef USE_GPIO_CLK
    //tasklet_kill(&clk_tasklet);
#endif
    input_unregister_device(input_hall_dev);
    input_free_device(input_hall_dev);
    kfree(cam_gpio_req_tbl);
    unregister_chrdev(major, "cammotor");
    device_destroy(motor_class,  MKDEV(major, 0));
    class_destroy(motor_class);
    
    return 0;
}


static const struct of_device_id of_match_motor[] = {
    { .compatible = "lenovo,cam-motor", .data = NULL },
    { /* sentinel */ }
};

struct platform_driver motor_drv = {
    .probe      = motor_probe,
    .remove     = motor_remove,
    .driver     = {
        .name   = "cammotor",
        .of_match_table = of_match_motor,
    }
};


static int cam_motor_init(void)
{
    platform_driver_register(&motor_drv);
    return 0;
}

static void cam_motor_exit(void)
{
    platform_driver_unregister(&motor_drv);
}

module_init(cam_motor_init);
module_exit(cam_motor_exit);
MODULE_LICENSE("GPL v2");
