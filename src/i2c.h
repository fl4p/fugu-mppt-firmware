#pragma once

#include "driver/i2c.h"

#define ACK_CHECK_EN			0x1
#define ACK_CHECK_DIS			0x0

#define ACK_VAL				I2C_MASTER_ACK
#define NACK_VAL			I2C_MASTER_NACK

esp_err_t i2c_write_short(uint8_t i2c_master_port, uint8_t address, uint8_t command, uint16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, (data & 0xFF00) >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        //printf("i2c_write successful\r\n");
    } else {
        ESP_LOGE("i2c", "i2c_write_short(addr=0x%02hhX,cmd=0x%02hhX) failed", address, command);
    }

    return(ret);
}

esp_err_t i2c_write_buf(uint8_t i2c_master_port, uint8_t address, uint8_t command, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command, ACK_CHECK_EN);
    if (len) {
        for (int i = 0; i < len; i++) {
            i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
        }
    }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        //printf("i2c_write successful\r\n");
    } else {
        printf("i2c_write_buf failed\r\n");
    }

    return(ret);
}

uint16_t i2c_read_short(uint8_t i2c_master_port, uint8_t address, uint8_t command)
{
    i2c_write_buf(i2c_master_port, address, command, NULL, 0);

    uint16_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, (uint8_t *)&data, 2, ACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    if (ret == ESP_OK) {

    } else if (ret == ESP_ERR_TIMEOUT) {
        //ESP_LOGW(TAG, "Bus is busy");
    } else {
        //ESP_LOGW(TAG, "Read failed");
    }
    return(__bswap16(data));
}


esp_err_t i2c_read_buf(uint8_t i2c_master_port, uint8_t address, uint8_t command, uint8_t *buffer, uint8_t len)
{
    i2c_write_buf(i2c_master_port, address, command, NULL, 0);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, buffer, len, ACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        for (int i = 0; i < len; i++) {
            //printf("0x%02x ", data[i]);
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        //ESP_LOGW(TAG, "Bus is busy");
    } else {
        //ESP_LOGW(TAG, "Read failed");
    }
    return(ret);
}

bool i2c_test_address(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}