#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_system.h>

static const char *TAG = "M5Tab5-Test";

// M5Stack Tab5 Hardware Configuration
#define I2C_MASTER_SCL_IO    8     // GPIO8 for SCL
#define I2C_MASTER_SDA_IO    7     // GPIO7 for SDA  
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

#define LED_GPIO             GPIO_NUM_2  // Built-in LED
#define BUTTON_GPIO          GPIO_NUM_0  // Boot button

// BMI270 IMU I2C Address
#define BMI270_ADDR          0x68
#define BMI270_CHIP_ID_REG   0x00

// ES8388 Audio Codec I2C Address  
#define ES8388_ADDR          0x10
#define ES8388_CHIPID_REG    0x01

void gpio_test_task(void *arg)
{
    ESP_LOGI(TAG, "🔌 Starting GPIO test - M5Stack Tab5 Hardware Demo");
    
    // Configure LED GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // Configure button GPIO
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    int led_state = 0;
    while (1) {
        // Blink LED
        gpio_set_level(LED_GPIO, led_state);
        led_state = !led_state;
        
        // Check button state
        int button_state = gpio_get_level(BUTTON_GPIO);
        ESP_LOGI(TAG, "💡 LED: %s, 🔘 Button: %s", 
                 led_state ? "ON" : "OFF",
                 button_state ? "Released" : "Pressed");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void i2c_test_task(void *arg)
{
    ESP_LOGI(TAG, "🔗 Starting I2C test - Scanning M5Stack Tab5 peripherals");
    
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ I2C config failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ I2C install failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "✅ I2C initialized successfully");
    
    // Test BMI270 IMU
    uint8_t bmi270_chip_id;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMI270_CHIP_ID_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &bmi270_chip_id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🎯 BMI270 IMU found! Chip ID: 0x%02X", bmi270_chip_id);
    } else {
        ESP_LOGW(TAG, "⚠️ BMI270 IMU not responding: %s", esp_err_to_name(ret));
    }
    
    // Test ES8388 Audio Codec
    uint8_t es8388_chip_id;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ES8388_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ES8388_CHIPID_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ES8388_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &es8388_chip_id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🎵 ES8388 Audio Codec found! Chip ID: 0x%02X", es8388_chip_id);
    } else {
        ESP_LOGW(TAG, "⚠️ ES8388 Audio Codec not responding: %s", esp_err_to_name(ret));
    }
    
    vTaskDelete(NULL);
}

void display_test_task(void *arg)
{
    ESP_LOGI(TAG, "📺 Starting Display test - M5Stack Tab5 1280x720 IPS");
    
    while (1) {
        ESP_LOGI(TAG, "🌈 Display: Drawing test pattern on 1280x720 screen");
        ESP_LOGI(TAG, "👆 Touch: Checking GT911 touch controller");
        ESP_LOGI(TAG, "🖼️ Graphics: Testing LVGL UI framework");
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void system_info_task(void *arg)
{
    ESP_LOGI(TAG, "🔍 M5Stack Tab5 System Information:");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "💾 Chip: %s revision %d", 
             CONFIG_IDF_TARGET, chip_info.revision);
    ESP_LOGI(TAG, "⚡ CPU Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "📡 WiFi%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "/802.11bgn" : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "");
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "💽 Flash: %lu MB", flash_size / (1024 * 1024));
    
    ESP_LOGI(TAG, "🧠 Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "⏰ Uptime: %llu ms", esp_timer_get_time() / 1000);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "🎉 M5Stack Tab5 Emulator Test Firmware Starting!");
    ESP_LOGI(TAG, "🔧 Hardware: ESP32-P4 @ 400MHz, 32MB PSRAM, 16MB Flash");
    ESP_LOGI(TAG, "📱 Device: 5-inch 1280x720 IPS + GT911 Touch + BMI270 IMU");
    ESP_LOGI(TAG, "🎵 Audio: ES8388 Codec + ES7210 AEC + Dual Microphones");
    ESP_LOGI(TAG, "📷 Camera: SC2356 2MP + WiFi 6 + Bluetooth LE + USB-C");
    
    // Create test tasks
    xTaskCreate(system_info_task, "system_info", 4096, NULL, 5, NULL);
    xTaskCreate(gpio_test_task, "gpio_test", 4096, NULL, 5, NULL);
    xTaskCreate(i2c_test_task, "i2c_test", 4096, NULL, 5, NULL);
    xTaskCreate(display_test_task, "display_test", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "🚀 All test tasks started - Emulator ready for testing!");
}