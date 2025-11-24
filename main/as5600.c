#include "as5600.h"

#include <esp_timer.h>
#include <driver/gpio.h>
#include <math.h>

static char* TAG = "AS5600";

esp_err_t as5600_i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len);
esp_err_t as5600_i2c_read_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data);
esp_err_t as5600_i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t length);
esp_err_t as5600_i2c_write_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t value);

// export function
void as5600_update(struct Sensor* sensor);
float as5600_angle_without_track(struct Sensor* sensor);
float as5600_read_angle(struct Sensor* sensor);
float as5600_read_velocity(struct Sensor* sensor);

struct Sensor* new_as5600(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t port) {
    i2c_master_dev_handle_t i2c_dev_handle;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = pin_scl,
        .sda_io_num = pin_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDRESS,
        .scl_speed_hz = 800*1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_dev_handle));
    gpio_set_level((gpio_num_t)pin_sda, 1);
    gpio_set_level((gpio_num_t)pin_scl, 1);

    uint8_t status = 0;
    ESP_ERROR_CHECK(as5600_i2c_read_reg(i2c_dev_handle, AS5600_REG_STATUS, &status));

    ESP_LOGI(TAG, "I2C bus initialized");

    struct Sensor* sensor = emalloc(sizeof(struct Sensor));
    sensor->sensor = emalloc(sizeof(struct AS5600));
    ((struct AS5600*)sensor->sensor)->i2c_dev_handle = i2c_dev_handle;
     ((struct AS5600*)sensor->sensor)->full_ratations = 0;
    ((struct AS5600*)sensor->sensor)->prev_angle = 0;
    ((struct AS5600*)sensor->sensor)->velocity = 0;
    ((struct AS5600*)sensor->sensor)->prev_angle = 0;

    sensor->update = as5600_update;
    sensor->read_angle_without_track = as5600_angle_without_track;
    sensor->read_angle = as5600_read_angle;
    sensor->read_velocity = as5600_read_velocity;
    return sensor;
    /*
    for (int i = 0; i < 128; i += 16) {
        for (int j = 0; j < 16; j++) {
            int addr = i + j;
            if (addr <= 0x03 || addr > 0x77) {
                continue;
            }
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 400*1000,
            };
            i2c_master_dev_handle_t dev_handle;
            esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                uint8_t status = 0;
                ret = as5600_i2c_read_reg(AS5600_REG_STATUS, &status);
                if (ret == ESP_OK) {
                    printf("%02x: %02x\n", addr, status);
                }
            }
            i2c_master_bus_rm_device(dev_handle);
        }
        printf("\n");
    }
    */
}

esp_err_t as5600_i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, len, -1);
}

esp_err_t as5600_i2c_read_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, 1, -1);
}

esp_err_t as5600_i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t length)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_transmit(i2c_dev_handle, data, length, -1);
    
    return ret;
}

esp_err_t as5600_i2c_write_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {reg, value};
    return i2c_master_transmit(i2c_dev_handle, cmd, 2, -1);
}

void as5600_update(struct Sensor* sensor) {
    i2c_master_dev_handle_t dev_handle = ((struct AS5600*)sensor->sensor)->i2c_dev_handle;
    uint8_t data[2] = {0};
    esp_err_t ret = as5600_i2c_read_bytes(dev_handle, AS5600_REG_RAW_ANGLE, data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error");
        return;
    }
    float raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF;
    // 0.087890625其实就是360/4096(as5600编码器的角度范围是0-4096，所以要先转换成角度值，然后再转换成弧度)
    float angle_without_track = (float)raw_angle * 0.087890625 * M_PI / 180;
    // 计算angle
    float d_angle = angle_without_track - ((struct AS5600*)sensor->sensor)->angle_without_track;
    if (fabs(d_angle) > (0.8f * 2 * M_PI)) {
        ((struct AS5600*)sensor->sensor)->full_ratations += (d_angle > 0 ? -1 : 1);
    }
    float angle = (float) ((struct AS5600*)sensor->sensor)->full_ratations * 2 * M_PI + angle_without_track;
    // 计算velocity
    int64_t cur_us = esp_timer_get_time();
    float dt = (float)(cur_us - ((struct AS5600*)sensor->sensor)->prev_us) * 1e-6;
    float dangle = angle - ((struct AS5600*)sensor->sensor)->prev_angle;
    float velocity = dangle / dt;
    float TF = 0.009; //速度的低通滤波
    float alpha = TF / (TF + dt);
    velocity = (1 - alpha) * velocity + alpha * ((struct AS5600*)sensor->sensor)->velocity;
    // 更新计算的值
    ((struct AS5600*)sensor->sensor)->angle_without_track = angle_without_track;
    ((struct AS5600*)sensor->sensor)->angle = angle;
    ((struct AS5600*)sensor->sensor)->velocity = velocity;
    // update prev变量
    ((struct AS5600*)sensor->sensor)->prev_angle = angle;
    ((struct AS5600*)sensor->sensor)->prev_us = cur_us;
}
float as5600_angle_without_track(struct Sensor* sensor) {
    return ((struct AS5600*)sensor->sensor)->angle_without_track;
}
float as5600_read_angle(struct Sensor* sensor) {
    return ((struct AS5600*)sensor->sensor)->angle;
}
float as5600_read_velocity(struct Sensor* sensor) {
    return ((struct AS5600*)sensor->sensor)->velocity;
}