#pragma once

#include "driver/i2c.h"

#define ACK_CHECK_EN            0x1
#define ACK_CHECK_DIS            0x0

#define ACK_VAL                I2C_MASTER_ACK
#define NACK_VAL            I2C_MASTER_NACK

//#ifndef portTICK_RATE_MS
//#define portTICK_RATE_MS ( 1000 / configTICK_RATE_HZ )
//#endif

esp_err_t i2c_write_short(i2c_port_t i2c_master_port, uint8_t address, uint8_t command, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, (data & 0xFF00) >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        //printf("i2c_write successful\r\n");
    } else {
        ESP_LOGE("i2c", "i2c_write_short(addr=0x%02hhX,cmd=0x%02hhX) failed", address, command);
    }

    return (ret);
}

esp_err_t i2c_write_buf(i2c_port_t i2c_master_port, uint8_t address, uint8_t command, uint8_t *data, uint8_t len) {
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

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        //printf("i2c_write successful\r\n");
    } else {
        printf("i2c_write_buf failed\r\n");
    }

    return (ret);
}

uint16_t i2c_read_short(i2c_port_t i2c_master_port, uint8_t address, uint8_t command, bool write_ack = true,
                        uint32_t timeoutMs = 1000) {

    // TODO espidf v5.2
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html
    //i2c_master_transmit_receive()

    uint16_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // write register pointer:
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, write_ack);
    i2c_master_write_byte(cmd, command, write_ack);

    // read register data:
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, write_ack);
    i2c_master_read(cmd, (uint8_t *) &data, 2, ACK_VAL);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, timeoutMs * configTICK_RATE_HZ / 1000);
    i2c_cmd_link_delete(cmd);


    if (ret == ESP_OK) {

    } else if (ret == ESP_ERR_TIMEOUT) {
        //ESP_LOGW(TAG, "Bus is busy");
    } else {
        //ESP_LOGW(TAG, "Read failed");
    }
    return (__bswap16(data));
}


bool i2c_read_short2(i2c_port_t i2c_master_port, uint8_t address, std::array<uint8_t, 2> command,
                     std::array<uint16_t, 2> &out, uint32_t timeoutMs) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    for (auto i = 0; i < 2; ++i) {
        // write register pointer:
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, command[i], true);

        // read register data:
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, (uint8_t *) &out[i], 2, ACK_VAL);
    }


    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, timeoutMs * configTICK_RATE_HZ / 1000);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        //if (ret == ESP_ERR_TIMEOUT)
        return false;
    }

    for (auto i = 0; i < 2; ++i)
        out[i] = __bswap16(out[i]);

    return true;
}


/*
esp_err_t i2c_dev_readmulti(i2c_port_t i2c_master_port, uint8_t address, const void *out_data1,const void *out_data2,const void *out_data3, size_t out_size, void *in_data1, void *in_data2,void *in_data3, size_t in_size)
{
    if (!dev || !in_data1 || !in_data2 || !in_data3 || !in_size)
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev->port, &dev->cfg);

    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//1st
        i2c_master_start(cmd);//start byte
        i2c_master_write_byte(cmd, dev->addr << 1, true); ////master is going to write to this adress
        i2c_master_write(cmd, (void *)out_data1, out_size, true);//data

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true); //master is going to read at this adress
        i2c_master_read(cmd, in_data1, in_size, I2C_MASTER_ACK ); //MODIFY LENGHT
//2nd
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true); ////master is going to write to this adress
        i2c_master_write(cmd, (void *)out_data2, out_size, true);//data

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true); //master is going to read at this adress
        i2c_master_read(cmd, in_data2, in_size, I2C_MASTER_ACK); //MODIFY LENGHT
//3rd
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true); ////master is going to write to this adress
        i2c_master_write(cmd, (void *)out_data3, out_size, true);//data

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true); //master is going to read at this adress
        i2c_master_read(cmd, in_data3, in_size, I2C_MASTER_LAST_NACK); //MODIFY LENGHT

        i2c_master_stop(cmd);//stop byte

        res = i2c_master_cmd_begin(dev->port, cmd, CONFIG_I2CDEV_TIMEOUT / portTICK_RATE_MS);//Execution of command
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", dev->addr, dev->port, res);



        i2c_cmd_link_delete(cmd);//link are released
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}
 */


esp_err_t i2c_read_buf(i2c_port_t i2c_master_port, uint8_t address, uint8_t command, uint8_t *buffer, uint8_t len) {
    i2c_write_buf(i2c_master_port, address, command, NULL, 0);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, buffer, len, ACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, configTICK_RATE_HZ);
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
    return (ret);
}

bool i2c_test_address(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}