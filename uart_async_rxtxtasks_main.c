#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 17        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 18        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 1000000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define HTU21D_address 0x80
#define HTU21D_Temp_slave 0xf3
#define HTU21D_Humid_slave 0xf5
#define HTU21D_Reset 0xfe

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void DHT21_Temperature_Measure()
{

}

void DHT21_Humidty_Measure()
{

}

void DHT21_Temperature_Write()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HTU21D_address, 1);
    i2c_master_write_byte(cmd, HTU21D_Temp_slave, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void DHT21_Humidty_Write()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HTU21D_address, 1);
    i2c_master_write_byte(cmd, HTU21D_Humid_slave, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void DHT21_Temperature_Read()
{
    static uint8_t MSB, LSB;
    MSB = 0;
    LSB = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x81, 1);
    i2c_master_read_byte(cmd, &MSB, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &LSB, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    LSB &= 0xfe;
    int sum = MSB * 256 + LSB;
    float sum2 = 175.72 * (sum * 1.0 / 65535) - 46.85;
    printf("MSB = %d,LSB = %d,sum = %d,sum2 = %f\n", MSB, LSB, sum, sum2);
}

void DHT21_Humidty_Read()
{
    static uint8_t MSB, LSB;
    MSB = 0;
    LSB = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x81, 1);
    i2c_master_read_byte(cmd, &MSB, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &LSB, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    LSB &= 0xfe;
    int sum = MSB * 256 + LSB;
    float sum3 = 125 * (sum * 1.0 / 65535) - 6;
    printf("MSB = %d,LSB = %d,sum = %d,sum3 = %f\n", MSB, LSB, sum, sum3);
}

// void HTU_DataSave()
// {

// }

void DHT21_Reset()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x80, 1);
    i2c_master_write_byte(cmd, 0xfe, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI("i2c_init", "I2C initialized successfully");
    // DHT21_Reset();

    while (1)
    {
        DHT21_Temperature_Write();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        DHT21_Temperature_Read();
        DHT21_Humidty_Write();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        DHT21_Humidty_Read();
    }
    // i2c_driver_delete(I2C_MASTER_NUM);
}
