#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "driver/uart.h"

#define UART_NUM_0  ((uart_port_t)0)

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          100000
#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B

static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init() {
    i2c_master_init();

    // Wake up sensor
    uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_cmd, 2, pdMS_TO_TICKS(100));
}

static int16_t read_word(uint8_t reg) {
    uint8_t data[2];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(100));
    return (int16_t)((data[0] << 8) | data[1]);
}

void mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz)
{
    *ax = read_word(MPU6050_ACCEL_XOUT_H);
    *ay = read_word(MPU6050_ACCEL_XOUT_H + 2);
    *az = read_word(MPU6050_ACCEL_XOUT_H + 4);

    *gx = read_word(MPU6050_ACCEL_XOUT_H + 8);
    *gy = read_word(MPU6050_ACCEL_XOUT_H + 10);
    *gz = read_word(MPU6050_ACCEL_XOUT_H + 12);
}

static imu_data_t current_imu_data;  // Holds the latest scaled values

void mpu6050_update() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);

    current_imu_data.ax = ax / 16384.0f;
    current_imu_data.ay = ay / 16384.0f;
    current_imu_data.az = az / 16384.0f;

    current_imu_data.gx = gx / 131.0f;
    current_imu_data.gy = gy / 131.0f;
    current_imu_data.gz = gz / 131.0f;
}


#define SAMPLE_HZ      1000               
#define UART_BAUD      115200             

void imu_log_task(void *arg)
{
    uart_set_baudrate(UART_NUM_0, UART_BAUD);   
    printf("ts_us,ax,ay,az,gx,gy,gz\n");          

    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);

    while (true) {
        int64_t ts = esp_timer_get_time();       
        mpu6050_update();                         
        const imu_data_t *d = mpu6050_get_data();

        printf("%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
               (long long)ts,
               d->ax, d->ay, d->az,
               d->gx, d->gy, d->gz);

        vTaskDelay(period);                   
    }
}

const imu_data_t* mpu6050_get_data() {
    return &current_imu_data;
}