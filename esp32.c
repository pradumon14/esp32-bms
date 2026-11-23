#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BMS_ESP32";

/* Hardware and Measurement Configuration */
#define BAT_ADC_UNIT        ADC_UNIT_1
#define BAT_ADC_CHANNEL     ADC_CHANNEL_6      /* GPIO34 (Input-only) */
#define BAT_ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_NUM_SAMPLES     64
#define VDIV_R1_OHM         100000.0f
#define VDIV_R2_OHM         100000.0f

#define DS18B20_GPIO        GPIO_NUM_4

#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_21
#define I2C_SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ         400000
#define SSD1306_I2C_ADDR    0x3C

/* Safety Thresholds with Hysteresis */
#define LOW_BAT_WARN_V      3.30f
#define LOW_BAT_CLEAR_V     3.40f
#define LOW_BAT_WARN_SOC    15.0f
#define LOW_BAT_CLEAR_SOC   18.0f
#define HIGH_TEMP_WARN_C    45.0f
#define HIGH_TEMP_CLEAR_C   43.0f
#define BMS_CYCLE_PERIOD_MS 2000

/* Data Structures */
typedef struct {
    float voltage_v;
    float soc_pct;
} soc_point_t;

typedef struct {
    float adc_voltage_v;
    float battery_voltage_v;
    float soc_pct;
    float temp_c;
    bool  temp_valid;
    bool  low_bat_warn;
    bool  high_temp_warn;
} bms_data_t;

/* 11-Point Resting OCV-SOC Lookup Table (3.7 V Nominal Li-ion) */
static const soc_point_t soc_lut[] = {
    {3.00f,   0.0f}, {3.50f,  10.0f}, {3.61f,  20.0f},
    {3.66f,  30.0f}, {3.70f,  40.0f}, {3.74f,  50.0f},
    {3.84f,  60.0f}, {3.93f,  70.0f}, {4.02f,  80.0f},
    {4.11f,  90.0f}, {4.20f, 100.0f}
};
#define SOC_LUT_SIZE (sizeof(soc_lut) / sizeof(soc_lut[0]))

/* Peripheral Handles */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_adc_calibrated = false;
static i2c_master_bus_handle_t   s_i2c_bus = NULL;
static i2c_master_dev_handle_t   s_oled_dev = NULL;
static uint8_t                   s_oled_fb[1024];
static portMUX_TYPE              s_ow_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------- ADC Initialization & Voltage Measurement ---------------- */
static esp_err_t bms_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) return err;

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
        s_adc_calibrated = true;
        ESP_LOGI(TAG, "ADC line-fitting calibration enabled");
    } else {
        s_adc_calibrated = false;
        ESP_LOGW(TAG, "ADC eFuse calibration unavailable; using nominal scaling");
    }
    return ESP_OK;
}

static esp_err_t bms_read_battery_voltage(bms_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    int64_t sum_mv = 0;
    for (int i = 0; i < ADC_NUM_SAMPLES; i++) {
        int raw = 0;
        int mv = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw);
        if (err != ESP_OK) return err;

        if (s_adc_calibrated) {
            err = adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
            if (err != ESP_OK) return err;
        } else {
            mv = (int)((raw / 4095.0f) * 3100.0f);
        }
        sum_mv += mv;
    }
    data->adc_voltage_v = ((float)sum_mv / (float)ADC_NUM_SAMPLES) / 1000.0f;
    data->battery_voltage_v = data->adc_voltage_v *
                              ((VDIV_R1_OHM + VDIV_R2_OHM) / VDIV_R2_OHM);
    return ESP_OK;
}

/* ---------------- SOC Lookup & Piecewise Linear Interpolation ------------- */
static float bms_estimate_soc(float v_bat)
{
    if (v_bat <= soc_lut[0].voltage_v) return 0.0f;
    if (v_bat >= soc_lut[SOC_LUT_SIZE - 1].voltage_v) return 100.0f;

    for (size_t i = 0; i < SOC_LUT_SIZE - 1; i++) {
        float v1 = soc_lut[i].voltage_v;
        float v2 = soc_lut[i + 1].voltage_v;
        if (v_bat >= v1 && v_bat <= v2) {
            float s1 = soc_lut[i].soc_pct;
            float s2 = soc_lut[i + 1].soc_pct;
            return s1 + ((v_bat - v1) / (v2 - v1)) * (s2 - s1);
        }
    }
    return 0.0f;
}

/* ---------------- DS18B20 1-Wire Temperature Driver ----------------------- */
static void ds18b20_init_gpio(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << DS18B20_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,   /* Internal pull-up enabled as backup */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(DS18B20_GPIO, 1);
}

static bool ow_reset(void)
{
    gpio_set_level(DS18B20_GPIO, 0);
    esp_rom_delay_us(480);

    portENTER_CRITICAL(&s_ow_mux);
    gpio_set_level(DS18B20_GPIO, 1);
    esp_rom_delay_us(70);
    int presence = (gpio_get_level(DS18B20_GPIO) == 0);
    portEXIT_CRITICAL(&s_ow_mux);

    esp_rom_delay_us(410);
    return (presence != 0);
}

static void ow_write_bit(uint8_t bit)
{
    portENTER_CRITICAL(&s_ow_mux);
    gpio_set_level(DS18B20_GPIO, 0);
    if (bit) {
        esp_rom_delay_us(6);
        gpio_set_level(DS18B20_GPIO, 1);
        portEXIT_CRITICAL(&s_ow_mux);
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        gpio_set_level(DS18B20_GPIO, 1);
        portEXIT_CRITICAL(&s_ow_mux);
        esp_rom_delay_us(10);
    }
}

static uint8_t ow_read_bit(void)
{
    uint8_t bit = 0;
    portENTER_CRITICAL(&s_ow_mux);
    gpio_set_level(DS18B20_GPIO, 0);
    esp_rom_delay_us(6);
    gpio_set_level(DS18B20_GPIO, 1);
    esp_rom_delay_us(9);
    bit = (uint8_t)gpio_get_level(DS18B20_GPIO);
    portEXIT_CRITICAL(&s_ow_mux);
    esp_rom_delay_us(55);
    return bit;
}

static void ow_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit((byte >> i) & 0x01);
    }
}

static uint8_t ow_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (uint8_t)(ow_read_bit() << i);
    }
    return byte;
}

static uint8_t ds18b20_crc8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = buf[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static esp_err_t ds18b20_read_temp(float *temp_c)
{
    if (!temp_c) return ESP_ERR_INVALID_ARG;

    if (!ow_reset()) return ESP_ERR_NOT_FOUND;
    ow_write_byte(0xCC);  /* Skip ROM */
    ow_write_byte(0x44);  /* Convert T (12-bit default) */

    vTaskDelay(pdMS_TO_TICKS(750));

    if (!ow_reset()) return ESP_ERR_NOT_FOUND;
    ow_write_byte(0xCC);  /* Skip ROM */
    ow_write_byte(0xBE);  /* Read Scratchpad */

    uint8_t scratch[9];
    for (int i = 0; i < 9; i++) {
        scratch[i] = ow_read_byte();
    }
    if (ds18b20_crc8(scratch, 8) != scratch[8]) {
        return ESP_ERR_INVALID_CRC;
    }
    int16_t raw = (int16_t)((scratch[1] << 8) | scratch[0]);
    *temp_c = (float)raw / 16.0f;
    return ESP_OK;
}

/* ---------------- SSD1306 128x64 I2C OLED Driver -------------------------- */
static esp_err_t ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t payload[2] = {0x00, cmd};
    return i2c_master_transmit(s_oled_dev, payload, sizeof(payload), 100);
}

static esp_err_t ssd1306_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_oled_dev);
    if (err != ESP_OK) return err;

    static const uint8_t init_cmds[] = {
        0xAE,         /* Display OFF */
        0xD5, 0x80,   /* Set Display Clock Divide Ratio */
        0xA8, 0x3F,   /* Set Multiplex Ratio (1 to 64) */
        0xD3, 0x00,   /* Set Display Offset */
        0x40,         /* Set Display Start Line */
        0x8D, 0x14,   /* Enable Charge Pump */
        0x20, 0x02,   /* Set Memory Addressing Mode: Page Addressing Mode */
        0xA1,         /* Set Segment Re-map */
        0xC8,         /* Set COM Output Scan Direction */
        0xDA, 0x12,   /* Set COM Pins Hardware Configuration */
        0x81, 0xCF,   /* Set Contrast Control */
        0xD9, 0xF1,   /* Set Pre-charge Period */
        0xDB, 0x40,   /* Set VCOMH Deselect Level */
        0xA4,         /* Entire Display ON (Resume to RAM) */
        0xA6,         /* Set Normal Display */
        0xAF          /* Display ON */
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = ssd1306_write_cmd(init_cmds[i]);
        if (err != ESP_OK) return err;
    }
    memset(s_oled_fb, 0, sizeof(s_oled_fb));
    return ESP_OK;
}

static void get_glyph_5x7(char c, uint8_t g[5])
{
    static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const struct { char ch; uint8_t b[5]; } font[] = {
        {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
        {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
        {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
        {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
        {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
        {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
        {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
        {'E',{0x7F,0x49,0x49,0x49,0x41}}, {'F',{0x7F,0x09,0x09,0x09,0x01}},
        {'G',{0x3E,0x41,0x49,0x49,0x7A}}, {'H',{0x7F,0x08,0x08,0x08,0x7F}},
        {'I',{0x00,0x41,0x7F,0x41,0x00}}, {'J',{0x20,0x40,0x41,0x3F,0x01}},
        {'K',{0x7F,0x08,0x14,0x22,0x41}}, {'L',{0x7F,0x40,0x40,0x40,0x40}},
        {'M',{0x7F,0x02,0x0C,0x02,0x7F}}, {'N',{0x7F,0x04,0x08,0x10,0x7F}},
        {'O',{0x3E,0x41,0x41,0x41,0x3E}}, {'P',{0x7F,0x09,0x09,0x09,0x06}},
        {'Q',{0x3E,0x41,0x51,0x21,0x5E}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
        {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
        {'U',{0x3F,0x40,0x40,0x40,0x3F}}, {'V',{0x1F,0x20,0x40,0x20,0x1F}},
        {'W',{0x3F,0x40,0x38,0x40,0x3F}}, {'X',{0x63,0x14,0x08,0x14,0x63}},
        {'Y',{0x07,0x08,0x70,0x08,0x07}}, {'Z',{0x61,0x51,0x49,0x45,0x43}},
        {'.',{0x00,0x60,0x60,0x00,0x00}}, {':',{0x00,0x36,0x36,0x00,0x00}},
        {'%',{0x23,0x13,0x08,0x64,0x62}}, {'+',{0x08,0x08,0x3E,0x08,0x08}},
        {'-',{0x08,0x08,0x08,0x08,0x08}}, {'/',{0x20,0x10,0x08,0x04,0x02}},
        {'=',{0x14,0x14,0x14,0x14,0x14}}, {'!',{0x00,0x00,0x5F,0x00,0x00}}
    };
    for (size_t i = 0; i < sizeof(font) / sizeof(font[0]); i++) {
        if (font[i].ch == c) {
            memcpy(g, font[i].b, 5);
            return;
        }
    }
    memcpy(g, blank, 5);
}

static void ssd1306_draw_str(uint8_t page, uint8_t col, const char *str)
{
    if (page >= 8) return;
    while (*str && col <= 122) {
        uint8_t g[5];
        get_glyph_5x7(*str++, g);
        memcpy(&s_oled_fb[page * 128 + col], g, 5);
        col += 6;
    }
}

static esp_err_t ssd1306_flush(void)
{
    uint8_t tx[129];
    tx[0] = 0x40;  /* Data stream control byte */
    for (uint8_t p = 0; p < 8; p++) {
        ssd1306_write_cmd(0xB0 + p);
        ssd1306_write_cmd(0x00);
        ssd1306_write_cmd(0x10);
        memcpy(&tx[1], &s_oled_fb[p * 128], 128);
        esp_err_t err = i2c_master_transmit(s_oled_dev, tx, sizeof(tx), 100);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

/* ---------------- Alert Evaluation & Display Update ----------------------- */
static void bms_evaluate_warnings(bms_data_t *st)
{
    if (!st->low_bat_warn) {
        if (st->battery_voltage_v <= LOW_BAT_WARN_V ||
            st->soc_pct <= LOW_BAT_WARN_SOC) {
            st->low_bat_warn = true;
        }
    } else if (st->battery_voltage_v >= LOW_BAT_CLEAR_V &&
               st->soc_pct >= LOW_BAT_CLEAR_SOC) {
        st->low_bat_warn = false;
    }

    if (st->temp_valid) {
        if (!st->high_temp_warn && st->temp_c >= HIGH_TEMP_WARN_C) {
            st->high_temp_warn = true;
        } else if (st->high_temp_warn && st->temp_c <= HIGH_TEMP_CLEAR_C) {
            st->high_temp_warn = false;
        }
    } else {
        st->high_temp_warn = false;
    }
}

static void bms_update_oled(const bms_data_t *st)
{
    char line[22];
    memset(s_oled_fb, 0, sizeof(s_oled_fb));

    ssd1306_draw_str(0, 0, "BMS MONITOR");

    snprintf(line, sizeof(line), "V:%.2fV  %.0f%%",
             st->battery_voltage_v, st->soc_pct);
    ssd1306_draw_str(2, 0, line);

    if (st->temp_valid) {
        snprintf(line, sizeof(line), "T:%.1fC", st->temp_c);
    } else {
        snprintf(line, sizeof(line), "T:ERR");
    }
    ssd1306_draw_str(4, 0, line);

    if (st->low_bat_warn && st->high_temp_warn) {
        ssd1306_draw_str(6, 0, "WARN:LOW+TMP");
    } else if (st->low_bat_warn) {
        ssd1306_draw_str(6, 0, "WARN:LOW BAT");
    } else if (st->high_temp_warn) {
        ssd1306_draw_str(6, 0, "WARN:HI TEMP");
    } else {
        ssd1306_draw_str(6, 0, "STATUS: OK");
    }
    ssd1306_flush();
}

/* ---------------- Main Application Entry Point ---------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(bms_adc_init());
    ds18b20_init_gpio();
    ESP_ERROR_CHECK(ssd1306_init());

    bms_data_t bms = {0};
    ESP_LOGI(TAG, "Battery Monitoring System initialized");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(BMS_CYCLE_PERIOD_MS);

    while (1) {
        if (bms_read_battery_voltage(&bms) == ESP_OK) {
            bms.soc_pct = bms_estimate_soc(bms.battery_voltage_v);
        } else {
            ESP_LOGE(TAG, "ADC voltage acquisition failed");
        }

        esp_err_t t_err = ds18b20_read_temp(&bms.temp_c);
        bms.temp_valid = (t_err == ESP_OK);
        if (!bms.temp_valid) {
            ESP_LOGW(TAG, "DS18B20 read fault: %s", esp_err_to_name(t_err));
        }

        bms_evaluate_warnings(&bms);
        bms_update_oled(&bms);

        ESP_LOGI(TAG, "Vbat=%.2fV | SOC=%.1f%% | T=%.2fC (valid=%d)",
                 bms.battery_voltage_v, bms.soc_pct,
                 bms.temp_c, (int)bms.temp_valid);

        if (bms.low_bat_warn) {
            ESP_LOGW(TAG, "ALERT: Low battery threshold reached!");
        }
        if (bms.high_temp_warn) {
            ESP_LOGW(TAG, "ALERT: High battery temperature detected!");
        }

        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
