#include <FreeRTOS.h>
#include <task.h>
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "ssd1306_i2c.h"




#define LED_PIN 25
#define I2C_PORT i2c0
#define SDA_PIN  4
#define SCL_PIN  5






// globaler ADC-Wert
static uint16_t adc_value = 0;




// Display-Buffer
static uint8_t oled_buf[SSD1306_BUF_LEN];
static SemaphoreHandle_t oled_mutex;

// Funktionsdeklarationen
void oled_adc_write(uint16_t adc_val);
void oled_write(char *text);
void oled_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl);








// -------------------------------------
// Blink Task
void blink_task(void *pvParameters) {
    (void)pvParameters;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}






// -------------------------------------
// OLED Initialisierung
void oled_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl) {
    ssd1306_init(i2c, gpio_sda, gpio_scl);

    oled_mutex = xSemaphoreCreateMutex();
    if (!oled_mutex) while(1);

    memset(oled_buf, 0, sizeof(oled_buf));
    struct render_area area = {0, SSD1306_WIDTH-1, 0, SSD1306_NUM_PAGES-1};
    ssd1306_calc_render_area_buflen(&area);
    ssd1306_render(oled_buf, &area);
}

// OLED Text schreiben
void oled_write(char *text) {
    if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(100))) {
        memset(oled_buf, 0, sizeof(oled_buf));
        ssd1306_write_string(oled_buf, 0, 0, text);

        struct render_area area = {0, SSD1306_WIDTH-1, 0, SSD1306_NUM_PAGES-1};
        ssd1306_calc_render_area_buflen(&area);
        ssd1306_render(oled_buf, &area);

        xSemaphoreGive(oled_mutex);
    }
}

// ADC-Wert auf OLED schreiben
void oled_adc_write(uint16_t val) {
    adc_value = val; // global aktualisieren

    if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(100))) {
        memset(oled_buf, 0, SSD1306_WIDTH); // obere Zeile löschen

        char buf[16];
        snprintf(buf, sizeof(buf), "ADC: %u", val);
        ssd1306_write_string(oled_buf, 0, 0, buf);

        struct render_area area = {0, SSD1306_WIDTH-1, 0, SSD1306_NUM_PAGES-1};
        ssd1306_calc_render_area_buflen(&area);
        ssd1306_render(oled_buf, &area);

        xSemaphoreGive(oled_mutex);
    }
}


// -------------------------------------
// OLED Task für Balkenanzeige
void oled_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(100))) {
            // gesamten Framebuffer löschen
            memset(oled_buf, 0, sizeof(oled_buf));

            // Balkenanzeige
            int bar_width = (adc_value * SSD1306_WIDTH) / 4095;
            for (int x = 0; x < bar_width; x++) {
                for (int y = SSD1306_HEIGHT/2 - 4; y < SSD1306_HEIGHT/2 + 4; y++) {
                    ssd1306_set_pixel(oled_buf, x, y, true);
                }
            }

            // Text oben
            char buf[16];
            snprintf(buf, sizeof(buf), "ADC: %u", adc_value);
            ssd1306_write_string(oled_buf, 0, 0, buf);

            // rendern
            struct render_area area = {0, SSD1306_WIDTH-1, 0, SSD1306_NUM_PAGES-1};
            ssd1306_calc_render_area_buflen(&area);
            ssd1306_render(oled_buf, &area);

            xSemaphoreGive(oled_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}


// -------------------------------------
// ADC Task
void adc_task(void *pvParameters) {
    (void)pvParameters;
    adc_init();
    adc_gpio_init(26);    // GPIO26 -> ADC0
    adc_select_input(0);

    while (1) {
        uint16_t result = adc_read();  // 12-bit ADC
        printf("ADC: %u\n", result);
        fflush(stdout);

        oled_adc_write(result);        // OLED aktualisieren
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}



// -------------------------------------
int main() {
    stdio_init_all();
    oled_init(I2C_PORT, SDA_PIN, SCL_PIN);

    xTaskCreate(blink_task, "BlinkTask", 256, NULL, 1, NULL);
    xTaskCreate(adc_task, "ADCTask", 512, NULL, 1, NULL);
    xTaskCreate(oled_task, "OLEDTask", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        // sollte nie erreicht werden
    }
}
