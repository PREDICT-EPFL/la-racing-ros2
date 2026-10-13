#include "bno08x.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_20         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_22         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BNO08X_PACKET_READ_TIMEOUT      2000         //ms
#define BNO08X_FEATURE_ENABLE_TIMEOUT   2000         //ms
#define BNO08X_DEFAULT_REPORT_INTERVAL  8000        //us

#define BNO08X_SENSOR_ADDR              0x4A       /*!< Slave address of the BNO08X sensor */
#define BNO08X_DATA_BUFFER_SIZE         512
#define BNO08X_CHANNEL_EXE              0x01       /*!< Channel 1: executable */
#define BNO08X_CHANNEL_CONTROL          0x02       /*!< Channel 2: sensor hub control channel*/
#define BNO08X_CHANNEL_SENSOR           0x03       /*!< Channel 3: input sensor reports*/

#define BNO08X_REPORT_ACCELEROMETER     0x01       /*!< Calibrated acceleration */
#define BNO08X_REPORT_GYROSCOPE         0x02       /*!< Calibrated gyroscope */

#define BNO08X_ME_CALIBRATE             0x07
#define BNO08X_ME_CAL_CONFIG            0x00
    
#define _SHTP_REPORT_PRODUCT_ID_REQUEST     0xF9
#define _SHTP_REPORT_PRODUCT_ID_RESPONSE    0xF8
#define _SHTP_REPORT_COMMAND_REQUEST        0xF2
#define _SHTP_REPORT_COMMAND_RESPONSE       0xF1

#define BNO08X_GET_FEATURE_COMMAND          0xFC
#define BNO08X_SET_FEATURE_COMMAND          0xFD

static const char *TAG = "BNO08X";

uint8_t bno08x_data_buffer[BNO08X_DATA_BUFFER_SIZE];
uint8_t bno08x_seq_number[6] = {0, 0, 0, 0, 0, 0};
uint8_t bno08x_seq_number_send[6] = {0, 0, 0, 0, 0, 0};
uint8_t bno08x_cal_seq_num = 0;

esp_err_t bno_send_package(uint8_t channel, uint16_t data_length)
{
    data_length += 4;

    bno08x_data_buffer[0] = data_length & 0xFF;
    bno08x_data_buffer[1] = (data_length >> 8) & 0xFF;
    bno08x_data_buffer[2] = channel;
    bno08x_data_buffer[3] = bno08x_seq_number_send[channel]++;

    return i2c_master_write_to_device(I2C_MASTER_NUM, BNO08X_SENSOR_ADDR, bno08x_data_buffer, data_length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t bno08x_read_packet()
{
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, BNO08X_SENSOR_ADDR, bno08x_data_buffer, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    uint16_t packet_byte_count = (bno08x_data_buffer[1] << 8 | bno08x_data_buffer[0]) & ~0x8000;
    uint8_t channel = bno08x_data_buffer[2];
    uint8_t seq_num = bno08x_data_buffer[3];
    
    if (packet_byte_count >= BNO08X_DATA_BUFFER_SIZE) {
        return ESP_FAIL;
    }

    bno08x_seq_number[channel] = seq_num;

    if (packet_byte_count > 4) {
        err = i2c_master_read_from_device(I2C_MASTER_NUM, BNO08X_SENSOR_ADDR, bno08x_data_buffer, packet_byte_count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

    return err;
}

esp_err_t handle_report_bno08x(raw_imu_t* imu)
{
    //ESP_LOGI(TAG, "Reading packet.");
    uint16_t packet_byte_count = (bno08x_data_buffer[1] << 8 | bno08x_data_buffer[0]) & ~0x8000;
    uint8_t channel = bno08x_data_buffer[2];

    if(channel != 3) return ESP_ERR_NOT_FOUND;

    uint32_t base_delta = 0;
        
    for(int i = 5; i < 9; i++){
        base_delta = base_delta | bno08x_data_buffer[i] << (8*(i-5));
    }
    //ESP_LOGI(TAG, "Time reference: %d", base_delta);
    //reads latest acc and latest gyro reports out of all received reports

    bool acc_read = false;
    bool gyro_read = false;
    
    /* Goes through the packet from the end to the front, assumes that the newer reports will be at the end, until it finds the latest accelerometer and gyroscope reports */
    for(int i = packet_byte_count - 10; i > 8 && (!acc_read || !gyro_read); i = i - 10){ 

        uint16_t delay = (bno08x_data_buffer[i+2] >> 2) << 8 | bno08x_data_buffer[i+3];

        uint16_t x = bno08x_data_buffer[i+5] << 8 | bno08x_data_buffer[i+4];
        uint16_t y = bno08x_data_buffer[i+7] << 8 | bno08x_data_buffer[i+6];
        uint16_t z = bno08x_data_buffer[i+9] << 8 | bno08x_data_buffer[i+8];

        if(bno08x_data_buffer[i] == BNO08X_REPORT_ACCELEROMETER && !acc_read){
            imu->status[0] = bno08x_data_buffer[i+2]&3;
            imu->delay[0] = base_delta - delay;
            imu->acc[0] = (int16_t)x; 
            imu->acc[1] = (int16_t)y; 
            imu->acc[2] = (int16_t)z;
            acc_read = true;
            //ESP_LOGI(TAG, "     acc delay: %d", delay);
            //ESP_LOGI(TAG, "     acc delay after subtraction: %d", imu->delay[0]);
        }
        else if(bno08x_data_buffer[i] == BNO08X_REPORT_GYROSCOPE && !gyro_read){
            imu->status[1] = bno08x_data_buffer[i+2]&3;
            imu->delay[1] = base_delta - delay;
            imu->gyro[0] = (int16_t)x; 
            imu->gyro[1] = (int16_t)y; 
            imu->gyro[2] = (int16_t)z;
            gyro_read = true;
            //ESP_LOGI(TAG, "     gyro delay: %d", delay);
            //ESP_LOGI(TAG, "     gyro delay after subtraction: %d", imu->delay[1]);
        }
        else continue;             
    }
    if(acc_read && gyro_read) return ESP_OK;
    
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t wait_for_packet_bno08x()
{
    TickType_t init = xTaskGetTickCount();
    while(xTaskGetTickCount() - init < pdMS_TO_TICKS(BNO08X_PACKET_READ_TIMEOUT)){
        bno08x_read_packet();
        uint16_t packet_byte_count = (bno08x_data_buffer[1] << 8 | bno08x_data_buffer[0]) & ~0x8000;
        if (packet_byte_count > 4) {
            return  ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t wait_for_packet_type_bno08x(uint8_t channel, uint8_t report_id)
{    
    TickType_t init = xTaskGetTickCount();
    while(xTaskGetTickCount() - init < pdMS_TO_TICKS(5 * BNO08X_PACKET_READ_TIMEOUT)){
        esp_err_t err = wait_for_packet_bno08x();
        if(err != ESP_OK) continue;
        if(bno08x_data_buffer[2] == channel){
            if(bno08x_data_buffer[4] == report_id){
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t check_id_bno08x()
{
    bno08x_data_buffer[4] = _SHTP_REPORT_PRODUCT_ID_REQUEST;
    bno08x_data_buffer[5] = 0; //padding

    ESP_LOGI(TAG, "Sending ID request report to BNO08x");
    ESP_ERROR_CHECK(bno_send_package(BNO08X_CHANNEL_CONTROL, 2));

    esp_err_t err;

    for(int i =  0; i < 10; i++){
        
        err = wait_for_packet_type_bno08x(BNO08X_CHANNEL_CONTROL, _SHTP_REPORT_PRODUCT_ID_RESPONSE);
        
        if(err == ESP_OK) {
            int partNum = 0;
            for(int i = 8; i < 12; i++){
                partNum= bno08x_data_buffer[i] << ((i-8)*8) | partNum;
            }
    
            ESP_LOGI(TAG, "Part number is: %d", partNum);
            ESP_LOGI(TAG, "ID checked successfully");
            break;
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t enable_feature_bno08x(int8_t feature)
{
    ESP_LOGI(TAG, "Enabling report: %d", feature);
    memset(bno08x_data_buffer, 0, 21);
    bno08x_data_buffer[4] = BNO08X_SET_FEATURE_COMMAND;
    bno08x_data_buffer[5] = feature;
    for(int i = 9; i < 13; i++){
        bno08x_data_buffer[i] = BNO08X_DEFAULT_REPORT_INTERVAL >> ((i-9)*8) & 0xFF;
    }
    //if the report required sensor specific configuration that too would need to be sent
    bno_send_package(BNO08X_CHANNEL_CONTROL, 17); 

    esp_err_t err = wait_for_packet_type_bno08x(BNO08X_CHANNEL_CONTROL, BNO08X_GET_FEATURE_COMMAND);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Successfully enabled report: %d", bno08x_data_buffer[5]);
        uint32_t report_interval = 0;
        for(int i = 9; i < 13; i++){
            report_interval = report_interval | bno08x_data_buffer[i] << (8*(i-9));
        }
        ESP_LOGI(TAG, "     Report interval: %d", report_interval);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Error enabling report: %d", feature); 
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t soft_reset_bno08x()
{
    ESP_LOGI(TAG, "Starting soft restart");

    // leave 4 bytes space for header
    bno08x_data_buffer[4] = 0x01;

    ESP_ERROR_CHECK_WITHOUT_ABORT(bno_send_package(BNO08X_CHANNEL_EXE, 1));
    vTaskDelay(500 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(bno_send_package(BNO08X_CHANNEL_EXE, 1));
    vTaskDelay(500 / portTICK_RATE_MS);

    for (int i = 0; i < 3; i++)
    {
        esp_err_t err = bno08x_read_packet();
        
        if (err == ESP_ERR_INVALID_RESPONSE) {
            vTaskDelay(500 / portTICK_RATE_MS);
        } else {
            ESP_ERROR_CHECK(err);
        }
        if(bno08x_data_buffer[2] == 1){
            ESP_LOGI(TAG, "BNO08X sensor successfully reset.");
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t calibrate_bno08x()
{
    bno08x_data_buffer[4] = _SHTP_REPORT_COMMAND_REQUEST;
    bno08x_data_buffer[5] =  bno08x_cal_seq_num++;
    bno08x_data_buffer[6] = BNO08X_ME_CALIBRATE;
    bno08x_data_buffer[7] = 1; //calibrate accel
    bno08x_data_buffer[8] = 1; //calibrate gyro
    bno08x_data_buffer[9] = 1; //calibrate mag
    bno08x_data_buffer[10] = BNO08X_ME_CAL_CONFIG;
    bno08x_data_buffer[11] = 0; //calibrate planar acceleration
    bno08x_data_buffer[12] = 0; //'on table' calibration
    bno08x_data_buffer[13] = 0; //reserved
    bno08x_data_buffer[14] = 0; //reserved
    bno08x_data_buffer[15] = 0; //reserved

    bno_send_package(BNO08X_CHANNEL_CONTROL, 12);
    esp_err_t err = wait_for_packet_type_bno08x(BNO08X_CHANNEL_CONTROL, _SHTP_REPORT_COMMAND_RESPONSE);
    if(err == ESP_OK){
        if(bno08x_data_buffer[9] == 0){
            ESP_LOGI(TAG, "Calibrated accelerometer, gyroscope and magnometer.");
            return ESP_OK;
        }
    }
    ESP_LOGI(TAG, "Error calibrating accelerometer, gyroscope and magnometer."); 
    return ESP_ERR_INVALID_RESPONSE;
}

void configure_bno08x()
{
    //initializing master

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
    i2c_set_timeout(i2c_master_port, 400000);

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));


    memset(bno08x_data_buffer, 0, 512);

    //reading bno08x advertisement 
    ESP_ERROR_CHECK(wait_for_packet_type_bno08x(BNO08X_CHANNEL_EXE, 0x01));

    ESP_ERROR_CHECK(soft_reset_bno08x());

    //check_id_bno08x();

    calibrate_bno08x();
    
    //enabling needed reports
    ESP_ERROR_CHECK(enable_feature_bno08x(BNO08X_REPORT_ACCELEROMETER));
    ESP_ERROR_CHECK(enable_feature_bno08x(BNO08X_REPORT_GYROSCOPE));
}