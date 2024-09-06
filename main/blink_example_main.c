
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#include "esp_system.h"
#include "esp_log.h"


#define LED_GPIO             48
#define BRIGHTNESS_STEP      50

//LED ON OFF BITS
#define LED_ON_BIT           (1 << 2)
#define LED_OFF_BIT          (1 << 3)

//LED BRIGHTNESS ADJUSTMENT BITS
#define INCREASE_BRIGHTNESS_BIT (1 << 4)
#define DECREASE_BRIGHTNESS_BIT (1 << 5)

//LED COLOR STATE BITS
#define RED_ACTIVE    (1 << 6)
#define GREEN_ACTIVE  (1 << 7)
#define BLUE_ACTIVE   (1 << 8)



//DECLARING VARIABLES TO HOLD THE CREATED EVENT GROUP
static EventGroupHandle_t led_state_group;


static void led_on(void)
{
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    xEventGroupSetBits(led_state_group, LED_ON_BIT);
}
static void led_off(void)
{
    /* Set all LED off to clear all pixels */
    xEventGroupSetBits(led_state_group, LED_OFF_BIT);
}
static void increase_brightness(void)
{
    xEventGroupSetBits(led_state_group, INCREASE_BRIGHTNESS_BIT);
}
static void decrease_brightness(void)
{
    /* Set all LED off to clear all pixels */
    xEventGroupSetBits(led_state_group, DECREASE_BRIGHTNESS_BIT);
}


//LED CONFIGURATION
static led_strip_handle_t led_strip;
static void configure_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)

    };
    #if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // RMT resolution
        .flags.with_dma = false
        
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    #elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
        led_strip_spi_config_t spi_config = {
            .spi_bus = SPI2_HOST,
            .flags.with_dma = true
        };
        ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    #else
    #error "unsupported LED strip backend"
    #endif
  

    led_strip_clear(led_strip);
}


void toggle_color_mode(void) {
    static uint32_t current_mode = RED_ACTIVE;
    xEventGroupClearBits(led_state_group, RED_ACTIVE | GREEN_ACTIVE | BLUE_ACTIVE);

    switch (current_mode) {
        case RED_ACTIVE:
            current_mode = GREEN_ACTIVE;
            xEventGroupSetBits(led_state_group, GREEN_ACTIVE);
            break;
        case GREEN_ACTIVE:
            current_mode = BLUE_ACTIVE;
            xEventGroupSetBits(led_state_group, BLUE_ACTIVE);
            break;
        case BLUE_ACTIVE:
            current_mode = RED_ACTIVE;
            xEventGroupSetBits(led_state_group, RED_ACTIVE);
            break;
    }
}

//LED CONTROL TASK
void led_control_task(void *pvParameters) {
    uint8_t red = 0, green = 0, blue = 255; // Initial color and brightness levels
    uint32_t active_color = BLUE_ACTIVE; // Start with blue active

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            led_state_group,
            LED_ON_BIT | LED_OFF_BIT | INCREASE_BRIGHTNESS_BIT | DECREASE_BRIGHTNESS_BIT | RED_ACTIVE | GREEN_ACTIVE | BLUE_ACTIVE,
            pdTRUE,  // Clear on exit
            pdFALSE, // Wait for any bit
            portMAX_DELAY
        );

        esp_err_t err;
        if (bits & LED_ON_BIT) {
            err = led_strip_set_pixel(led_strip, 0, red, green, blue); 
            if (err != ESP_OK) {ESP_LOGE("LED", "Failed to set pixel: %s", esp_err_to_name(err));} 
            else {led_strip_refresh(led_strip);}
            printf("LED ON\n");

        }

        if (bits & LED_OFF_BIT) {
            err = led_strip_clear(led_strip);
            if (err != ESP_OK) {ESP_LOGE("LED", "Failed to clear strip: %s", esp_err_to_name(err));}
            else {led_strip_refresh(led_strip);}
            printf("LED OFF\n");
        }
        if (bits & INCREASE_BRIGHTNESS_BIT) {
            switch (active_color) {
                case RED_ACTIVE:
                    red = (red + BRIGHTNESS_STEP > 255) ? 255 : red + BRIGHTNESS_STEP;
                    break;
                case GREEN_ACTIVE:
                    green = (green + BRIGHTNESS_STEP > 255) ? 255 : green + BRIGHTNESS_STEP;
                    break;
                case BLUE_ACTIVE:
                    blue = (blue + BRIGHTNESS_STEP > 255) ? 255 : blue + BRIGHTNESS_STEP;
                    break;
            }
            // Apply changes only if LED is on
            if (bits & LED_ON_BIT) {
                led_strip_set_pixel(led_strip, 0, red, green, blue);
                led_strip_refresh(led_strip);
            }
            printf("Brightness increased\n");
        }
        
        if (bits & DECREASE_BRIGHTNESS_BIT) {
            switch (active_color) {
                    case RED_ACTIVE:
                        red = (red < BRIGHTNESS_STEP) ? 0 : red - BRIGHTNESS_STEP;
                        break;
                    case GREEN_ACTIVE:
                        green = (green < BRIGHTNESS_STEP) ? 0 : green - BRIGHTNESS_STEP;
                        break;
                    case BLUE_ACTIVE:
                        blue = (blue < BRIGHTNESS_STEP) ? 0 : blue - BRIGHTNESS_STEP;
                        break;
            // Apply changes only if LED is on
            if (bits & LED_ON_BIT) {
                led_strip_set_pixel(led_strip, 0, red, green, blue);
                led_strip_refresh(led_strip);
            }
            printf("Brightness decreased\n");
        }
    }
}
}






void app_main(void)
{   
    configure_led();

    //LED STATE GROUP CREATION 
    led_state_group = xEventGroupCreate();
    if (led_state_group == NULL) {ESP_LOGE("LED", "Failed to create the event group");}   
    else {ESP_LOGI("LED", "Successfully created the event group");}  


    BaseType_t xLEDStateReturned;
    TaskHandle_t xLEDStateHandle = NULL;
    xLEDStateReturned = xTaskCreate(led_control_task,   // Task function
                            "LEDControlTask",   // Name of the task
                            4096,               // Stack size in words
                            NULL,               // Task input parameter
                            2,                  // Priority of the task
                            &xLEDStateHandle);          // Task handle

    if (xLEDStateReturned == pdPASS) {ESP_LOGI("LED", "Task created successfully");} 
    else {ESP_LOGE("LED", "Task creation failed");}


    while(1){
        vTaskDelay(1000/portTICK_PERIOD_MS);
        led_on();
        vTaskDelay(1000/portTICK_PERIOD_MS);
        decrease_brightness();

        }

    return;
}
