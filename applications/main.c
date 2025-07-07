
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <errno.h>
#include <stdio.h>
#include <arm_math.h>
#include <drivers/pin.h>
#include <math.h>
#include <rtdbg.h>
#include <cJSON.h>       // 包含cJSON库
// DHT11相关头文件
#include <drivers/sensor.h>
#include "sensor_dallas_dht11.h"
#include "drv_common.h"
//OLED相关头文件
#include "ssd1306.h"        // OLED驱动头文件
#include "ssd1306_conf.h"   // OLED配置头文件
#include "ssd1306_fonts.h"  // 字体文件
//添加文件头文件
#include <dfs_fs.h>
#include <dfs_posix.h>  // 文件操作头文件
#include <sys/time.h>   // 时间戳支持

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#define RT_I2C_MAX_MSG_LEN 32
// SGP30 配置参数
#define SGP30_ADDR         0x58        // 7位I2C地址
#define SGP30_CMD_INIT     0x2003      // 初始化命令
#define SGP30_CMD_MEASURE  0x2008      // 测量命令
#define SGP30_CRC8_POLYNOM 0x31        // CRC多项式
#define I2C_TIMEOUT 1000
// 定义软件I2C使用的引脚 - 请根据实际电路修改
#define SCL_PIN           GET_PIN(B, 6)  // SCL 引脚
#define SDA_PIN           GET_PIN(B, 7)  // SDA 引脚

// DHT11引脚
#define DHT11_DATA_PIN    GET_PIN(C, 9)

//OLED
#define OLED_I2C_BUS_NAME "i2c2"  // 修改为i2c2
#define OLED_I2C_ADDR     0x3C    // 地址不变

// 定义全局传感器数据结构
struct sensor_data {
    int co2;          // 二氧化碳 (0~3000 ppm)
    int humi;         // 湿度 (0~100%)
    double light;      // 光照 (0.001~1.0)
    double noise;      // 噪声 (0~200 dB)
    int temp;         // 温度 (0~100°C)
    int tvoc;         // TVOC (0~1000 ppb)
} sensors;

// 定义全局传感器数据互斥锁
struct rt_mutex sensor_mutex;

// 全局变量
rt_adc_device_t dev;
rt_thread_t th_adc, th_gas, th_dht11;
struct rt_i2c_bus_device *i2c_bus = RT_NULL;

// ====== 传感器数据更新函数 ======
void update_sensor_data(int co2_val, int humi_val, double light_val,
                       double noise_val, int temp_val, int tvoc_val)
{
    rt_mutex_take(&sensor_mutex, RT_WAITING_FOREVER);

    // 只更新有效的数值
    if (co2_val >= 0) sensors.co2 = co2_val;
    if (humi_val >= 0) sensors.humi = humi_val;
    if (light_val >= 0) sensors.light = light_val;
    if (noise_val >= 0) sensors.noise = noise_val;
    if (temp_val >= 0) sensors.temp = temp_val;
    if (tvoc_val >= 0) sensors.tvoc = tvoc_val;

    rt_mutex_release(&sensor_mutex);
}

// ====== 软件I2C实现 ======

// 自定义软件I2C操作函数集
static const struct rt_i2c_bus_device_ops soft_i2c_ops = {
    // 这些函数将在下面实现
    NULL,
    NULL,
    NULL,
    NULL
};

// 软件I2C延时函数 - 调整延时时间可以控制I2C速度
static void i2c_delay(void) {
    // 根据实际时钟频率调整延时
    for (volatile int i = 0; i < 10; i++); // 约1us延时
}

// SCL线操作
static void scl_set(rt_base_t value) {
    rt_pin_write(SCL_PIN, value);
}

// SDA线操作
static void sda_set(rt_base_t value) {
    rt_pin_write(SDA_PIN, value);
}

// SDA线读取
static int sda_get(void) {
    return rt_pin_read(SDA_PIN);
}

// I2C起始信号
static void i2c_start(void) {
    sda_set(PIN_HIGH);
    scl_set(PIN_HIGH);
    i2c_delay();
    sda_set(PIN_LOW);
    i2c_delay();
    scl_set(PIN_LOW);
}

// I2C停止信号
static void i2c_stop(void) {
    sda_set(PIN_LOW);
    scl_set(PIN_LOW);
    i2c_delay();
    scl_set(PIN_HIGH);
    i2c_delay();
    sda_set(PIN_HIGH);
    i2c_delay();
}

// I2C发送ACK
static void i2c_ack(void) {
    scl_set(PIN_LOW);
    sda_set(PIN_LOW);
    i2c_delay();
    scl_set(PIN_HIGH);
    i2c_delay();
    scl_set(PIN_LOW);
}

// I2C发送NACK
static void i2c_nack(void) {
    scl_set(PIN_LOW);
    sda_set(PIN_HIGH);
    i2c_delay();
    scl_set(PIN_HIGH);
    i2c_delay();
    scl_set(PIN_LOW);
}

// 检查ACK响应
static rt_bool_t i2c_check_ack(void) {
    rt_base_t ack;

    scl_set(PIN_LOW);
    rt_pin_mode(SDA_PIN, PIN_MODE_INPUT_PULLUP); // 设置为输入模式
    i2c_delay();
    scl_set(PIN_HIGH);
    i2c_delay();
    ack = sda_get();
    scl_set(PIN_LOW);
    rt_pin_mode(SDA_PIN, PIN_MODE_OUTPUT_OD); // 恢复为输出模式

    return (ack == PIN_LOW); // 低电平表示ACK
}

// I2C发送一个字节
static rt_bool_t i2c_write_byte(rt_uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        scl_set(PIN_LOW);
        sda_set(byte & 0x80 ? PIN_HIGH : PIN_LOW);
        i2c_delay();
        scl_set(PIN_HIGH);
        i2c_delay();
        scl_set(PIN_LOW);
        byte <<= 1;

    }
    return i2c_check_ack();
}

// I2C读取一个字节
static rt_uint8_t i2c_read_byte(rt_bool_t ack) {
    rt_uint8_t byte = 0;

    scl_set(PIN_LOW);
    rt_pin_mode(SDA_PIN, PIN_MODE_INPUT_PULLUP); // 设置为输入模式
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        scl_set(PIN_HIGH);
        i2c_delay();
        if (sda_get() == PIN_HIGH) {
            byte |= 0x01;
        }
        scl_set(PIN_LOW);
        i2c_delay();
    }
    rt_pin_mode(SDA_PIN, PIN_MODE_OUTPUT_OD); // 恢复为输出模式

    if (ack) {
        i2c_ack();
    } else {
        i2c_nack();
    }

    return byte;
}

// 自实现的I2C传输函数
static rt_size_t soft_i2c_xfer(struct rt_i2c_msg msgs[], rt_uint32_t num) {
    for (rt_uint32_t i = 0; i < num; i++) {
        if (msgs[i].len > RT_I2C_MAX_MSG_LEN) {  // 定义RT_I2C_MAX_MSG_LEN=32
                    LOG_E("I2C msg too long!");
                    return 0;
                }
        struct rt_i2c_msg *msg = &msgs[i];

        i2c_start();

        rt_uint8_t addr = (msg->addr << 1) | (msg->flags & RT_I2C_RD ? 1 : 0);
        if (!i2c_write_byte(addr)) {
            i2c_stop();
            return 0;
        }

        if (msg->flags & RT_I2C_RD) {
            for (rt_uint32_t j = 0; j < msg->len; j++) {
                msg->buf[j] = i2c_read_byte(j < msg->len - 1);
            }
        } else {
            for (rt_uint32_t j = 0; j < msg->len; j++) {
                if (!i2c_write_byte(msg->buf[j])) {
                    i2c_stop();
                    return 0;
                }
            }
        }
    }

    i2c_stop();
    return num;
}

// CRC8校验计算（SGP30要求）
static rt_uint8_t sgp30_crc(rt_uint8_t *data, int len) {
    rt_uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SGP30_CRC8_POLYNOM;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// I2C写命令（带CRC）
static rt_err_t sgp30_write_cmd(rt_uint16_t cmd) {
    rt_uint8_t buf[3];
    buf[0] = (cmd >> 8) & 0xFF;
    buf[1] = cmd & 0xFF;
    buf[2] = sgp30_crc(buf, 2);

    struct rt_i2c_msg msgs[1] = {
        {
            .addr  = SGP30_ADDR,
            .flags = RT_I2C_WR,
            .buf   = buf,
            .len   = 3
        }
    };

    if (soft_i2c_xfer(msgs, 1) != 1) {
        return -RT_ERROR;
    }
    return RT_EOK;
}

// 读取测量结果（带CRC校验）
static rt_err_t sgp30_read_data(rt_uint16_t *co2, rt_uint16_t *tvoc) {
    rt_uint8_t buf[6];
    struct rt_i2c_msg msgs[1] = {
        {
            .addr  = SGP30_ADDR,
            .flags = RT_I2C_RD,
            .buf   = buf,
            .len   = 6
        }
    };

    if (soft_i2c_xfer(msgs, 1) != 1) {
        return -RT_ERROR;
    }

    rt_uint8_t crc = sgp30_crc(buf, 2);
    if (crc != buf[2]) {
        LOG_E("CO2 CRC error: calc=0x%02X, recv=0x%02X", crc, buf[2]);
        return -RT_ERROR;
    }

    crc = sgp30_crc(buf + 3, 2);
    if (crc != buf[5]) {
        LOG_E("TVOC CRC error: calc=0x%02X, recv=0x%02X", crc, buf[5]);
        return -RT_ERROR;
    }

    *co2  = (buf[0] << 8) | buf[1];
    *tvoc = (buf[3] << 8) | buf[4];

    return RT_EOK;
}

// ====== 传感器线程 ======

// 气体传感器线程
static void sgp30_thread_entry(void *param) {
    rt_uint16_t co2 = 0, tvoc = 0;

    // 初始化I2C引脚
    rt_pin_mode(SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(SDA_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(SCL_PIN, PIN_HIGH);
    rt_pin_write(SDA_PIN, PIN_HIGH);

    LOG_I("Software I2C pins initialized");

    // 传感器初始化
    if (sgp30_write_cmd(SGP30_CMD_INIT) != RT_EOK) {
        LOG_E("SGP30 init failed!");
        return;
    }
    rt_thread_mdelay(1000); // 等待初始化完成

    LOG_I("SGP30 initialized");

    while (1) {
        // 发送测量指令
        if (sgp30_write_cmd(SGP30_CMD_MEASURE) != RT_EOK) {
            LOG_W("Measure cmd send failed");
            rt_thread_mdelay(1000);
            continue;
        }

        rt_thread_mdelay(30); // 等待测量完成（典型12ms，最大25ms）

        // 读取数据
        if (sgp30_read_data(&co2, &tvoc) == RT_EOK) {
            // 更新全局数据
            update_sensor_data(co2, -1, -1, -1, -1, tvoc);
            //LOG_D("Gas data updated: CO2=%d ppm, TVOC=%d ppb", co2, tvoc);
        } else {
            LOG_W("Read data failed");
        }

        rt_thread_mdelay(1000); // 1秒采样周期
    }
}

// ADC读取线程
void read_adc1_entry(void *parameter) {
    rt_uint32_t valA = 0;
    double light_val = 0.0;
    rt_uint32_t valB = 0;
    double noise_val = 0.0;

    while(1){
        valA = rt_adc_read(dev, 5);
        light_val = 100.0 - (double)valA / 40.0; // 转换为光照值

        valB = rt_adc_read(dev, 6);
        noise_val = 20 * log10((double)valB / 4095.0 * 3.3 + 0.00002) * 5; // 转换为噪声值

        // 更新全局数据
        update_sensor_data(-1, -1, light_val, noise_val, -1, -1);
       // LOG_D("ADC data updated: light=%.3f, noise=%.1f dB", light_val, noise_val);

        rt_thread_mdelay(1000);
    }
}




// DHT11读取线程
static void read_temp_entry(void *parameter) {
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;
    rt_uint8_t get_data_freq = 1; /* 1Hz */

    dev = rt_device_find("temp_dht11");
    if (dev == RT_NULL) {
        LOG_E("DHT11 device not found");
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        LOG_E("open DHT11 device failed");
        return;
    }

    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)(&get_data_freq));

    while (1) {
        res = rt_device_read(dev, 0, &sensor_data, 1);

        if (res != 1) {
            LOG_W("DHT11 read data failed! result is %d", res);
        } else {
            uint8_t temp = (sensor_data.data.temp & 0xffff) >> 0;
            uint8_t humi = (sensor_data.data.temp & 0xffff0000) >> 16;

            // 更新全局数据
            update_sensor_data(-1, humi, -1, -1, temp, -1);
           // LOG_D("DHT11 data updated: temp=%d°C, humi=%d%%", temp, humi);
        }

        rt_thread_delay(1000);
    }
}

//OLED显示线程函数
static void oled_display_entry(void *param) {
    // 初始化OLED（驱动已封装硬件细节）
    ssd1306_Init();  // 关键修改：使用无参初始化[1,2](@ref)
    ssd1306_Fill(Black); // 清屏
    ssd1306_UpdateScreen();

    char buf[4][24]; // 4行文本缓冲区
    while (1) {
        // 加锁获取传感器数据
        rt_mutex_take(&sensor_mutex, RT_WAITING_FOREVER);
        snprintf(buf[0], sizeof(buf[0]), " T:%dC H:%d%%", sensors.temp, sensors.humi);
        snprintf(buf[1], sizeof(buf[1]), " L:%.2f N:%.2fdB", sensors.light, sensors.noise);
        snprintf(buf[2], sizeof(buf[2]), " CO2:%dppm", sensors.co2);
        snprintf(buf[3], sizeof(buf[3]), " TVOC:%dppb", sensors.tvoc);
        rt_mutex_release(&sensor_mutex);

        // 清屏并显示（使用驱动原生API）
        ssd1306_Fill(Black);  // 关键修改：移除设备句柄参数[1](@ref)

        // 第1行：温度/湿度
        ssd1306_SetCursor(0, 0);  // 设置光标位置(x=0, y=0)
        ssd1306_WriteString(buf[0], Font_7x10, White);  // 关键修改：直接调用驱动函数

        // 第2行：光照/噪声
        ssd1306_SetCursor(0, 12); // y=12 (第2行)
        ssd1306_WriteString(buf[1], Font_7x10, White);

        // 第3行：CO2
        ssd1306_SetCursor(0, 24); // y=24 (第3行)
        ssd1306_WriteString(buf[2], Font_7x10, White);

        // 第4行：TVOC
        ssd1306_SetCursor(0, 36); // y=36 (第4行)
        ssd1306_WriteString(buf[3], Font_7x10, White);

        // 刷新屏幕
        ssd1306_UpdateScreen();  // 关键修改：统一调用驱动更新函数

        rt_thread_mdelay(500); // 0.5秒刷新
    }
}


// ====== 数据上传线程 ======
static void data_upload_entry(void *parameter) {
    while (1) {
        rt_mutex_take(&sensor_mutex, RT_WAITING_FOREVER);

        // 获取当前传感器值
        int co2_value = sensors.co2;
        int humi_value = sensors.humi;
        double light_value = sensors.light;
        double noise_value = sensors.noise;
        int temp_value = sensors.temp;
        int tvoc_value = sensors.tvoc;

        rt_mutex_release(&sensor_mutex);

        // 打印当前所有传感器值
//        LOG_I("Current sensor values:");
//        LOG_I("CO2: %d ppm", co2_value);
//        LOG_I("Humidity: %d%%", humi_value);
//        LOG_I("Light: %.3f", light_value);
//        LOG_I("Noise: %.1f dB", noise_value);
//        LOG_I("Temperature: %d°C", temp_value);
//        LOG_I("TVOC: %d ppb", tvoc_value);

        rt_thread_delay(rt_tick_from_millisecond(2000)); // 2秒上传周期
    }
}

//onenet_mqtt_init
//onenet_upload_cycle
// ====== 主函数 ======
int main(void) {

    rt_err_t ret = RT_EOK;

    // 初始化传感器互斥锁
    if (rt_mutex_init(&sensor_mutex, "sensor_mutex", RT_IPC_FLAG_FIFO) != RT_EOK) {
        LOG_E("Failed to initialize sensor mutex");
        return -RT_ERROR;
    }

    // 初始化传感器数据结构
    sensors.co2 = 0;
    sensors.humi = 0;
    sensors.light = 0.0;
    sensors.noise = 0.0;
    sensors.temp = 0;
    sensors.tvoc = 0;

    LOG_I("Sensor data structure initialized");

    // 1. 初始化ADC
    dev = (rt_adc_device_t)rt_device_find("adc1");
    if(dev == RT_NULL){
        LOG_E("ADC1 device not found");
        return -RT_ENOSYS;
    }

    ret = rt_adc_enable(dev, 5);
    if(ret != RT_EOK){
        LOG_E("Enable ADC ch5 failed: %d", ret);
        return ret;
    }
    ret = rt_adc_enable(dev, 6);
    if(ret != RT_EOK){
        LOG_E("Enable ADC ch6 failed: %d", ret);
        return ret;
    }

    // 2. 创建ADC线程
    th_adc = rt_thread_create("adc_th", read_adc1_entry, RT_NULL,
                          2048, RT_THREAD_PRIORITY_MAX/2, 5);
    if(th_adc != RT_NULL){
        rt_thread_startup(th_adc);
        LOG_I("ADC thread started");
    } else {
        LOG_E("ADC thread create failed");
    }

    // 3. 创建气体传感器线程
    th_gas = rt_thread_create("gas_th", sgp30_thread_entry, RT_NULL,
                            4096, RT_THREAD_PRIORITY_MAX/2, 5);
    if (th_gas != RT_NULL) {
        rt_thread_startup(th_gas);
        LOG_I("SGP30 thread started");
    } else {
        LOG_E("Failed to create SGP30 thread");
    }

    // 4. 创建DHT11线程
    th_dht11 = rt_thread_create("dht_th", read_temp_entry, RT_NULL,
                                     2048, RT_THREAD_PRIORITY_MAX / 2, 20);
    if (th_dht11 != RT_NULL) {
        rt_thread_startup(th_dht11);
        LOG_I("DHT11 thread started");
    } else {
        LOG_E("Failed to create DHT11 thread");
    }

    // 5. 创建数据上传线程
    rt_thread_t upload_thread = rt_thread_create("upload_th", data_upload_entry, RT_NULL,
                                                2048, RT_THREAD_PRIORITY_MAX/3, 5);
    if (upload_thread != RT_NULL) {
        rt_thread_startup(upload_thread);
        LOG_I("Data upload thread started");
    } else {
        LOG_E("Failed to create upload thread");
    }

    //6.OLED线程创建：

    rt_thread_t oled_thread = rt_thread_create("oled", oled_display_entry, RT_NULL,
                                             2048, RT_THREAD_PRIORITY_MAX/2-10, 10);
    if (oled_thread) {
            rt_thread_startup(oled_thread);
            LOG_I("OLED display thread started");
        } else {
            LOG_E("Failed to create OLED thread");
        }

    return RT_EOK;
}
