/*
 * HomeKit DHT22 Temperature and Humidity Sensor Example
 *
 * This example is based on examples from Espressif's Homekit SDK.
 * It makes use of DHT22 subroutines from https://github.com/Andrey-m/DHT22-lib-for-esp-idf
 *
 */

/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include <DHT.h>

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "HAP Weather Sensor";

#define HAP_TASK_PRIORITY  1
#define HAP_TASK_STACKSIZE 4 * 1024
#define HAP_TASK_NAME      "hap_weather_sensor"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0
#define DHT_GPIO  GPIO_NUM_4
#define LED_GPIO  GPIO_NUM_2

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* LED and DHT GPIO initialisation
 */
void accessory_init()
{
    ESP_LOGI(TAG, "Initialising LED and DHT sensor\n");
    gpio_set_direction( LED_GPIO, GPIO_MODE_OUTPUT );
    gpio_set_level( LED_GPIO, 0 );
    setDHTgpio(DHT_GPIO);
}

/* Mandatory identify routine for the accessory.
 */
static int accessory_identify(hap_acc_t *ha)
{
    // Blink LED
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            gpio_set_level( LED_GPIO, 1 );
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level( LED_GPIO, 0 );
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void accessory_hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
        case HAP_EVENT_PAIRING_MODE_TIMED_OUT :
            ESP_LOGI(TAG, "Pairing Mode timed out. Please reboot the device.");
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* Get readings from DHT22 and update Homekit characterisitics
 */
static int update_readings()
{
    hap_val_t new_val;
    hap_serv_t *hs;
    hap_char_t *hc;
    float dht_temperature, dht_humidity;

    int response = readDHT();
    switch(response) {
        case DHT_TIMEOUT_ERROR :
            ESP_LOGE(TAG, "Timeout error, skipping");
            break;
        case DHT_CHECKSUM_ERROR:
            ESP_LOGE(TAG, "Checksum error, skipping");
            break;
        case DHT_OK:
            dht_temperature = getTemperature();
            dht_humidity = getHumidity();
            ESP_LOGI(TAG, "Temperature = %.1f°C, Humidity = %.1f%%", dht_temperature , dht_humidity);
            hs = hap_acc_get_serv_by_uuid(hap_get_first_acc(), HAP_SERV_UUID_TEMPERATURE_SENSOR);
            hc = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
            new_val.f = dht_temperature;
            hap_char_update_val(hc, &new_val);
            hs = hap_acc_get_serv_by_uuid(hap_get_first_acc(), HAP_SERV_UUID_HUMIDITY_SENSOR);
            hc = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
            new_val.f = dht_humidity;
            hap_char_update_val(hc, &new_val);
            break;
        default :
            ESP_LOGE(TAG, "Unknown error, skipping");
    }
    return response;
}

/* Callback for handling a read request.
 */
static int sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    update_readings();
    *status_code = HAP_STATUS_SUCCESS;
    return HAP_SUCCESS;
}

/* The main thread for handling the accessory */
static void accessory_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Esp-Weather",
        .manufacturer = "Espressif",
        .model = "EspWeather01",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = accessory_identify,
        .cid = HAP_CID_SENSOR,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Temperature Sensor Service. */
    service = hap_serv_temperature_sensor_create(0);
    hap_serv_add_char(service, hap_char_name_create("My Temperature Sensor"));
    hap_serv_set_read_cb(service, sensor_read);
    hap_acc_add_serv(accessory, service);

    /* Create the Humidity Sensor Service. */
    service = hap_serv_humidity_sensor_create(0);
    hap_serv_add_char(service, hap_char_name_create("My Humidity Sensor"));
    hap_serv_set_read_cb(service, sensor_read);
    hap_acc_add_serv(accessory, service);

    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    service = hap_serv_fw_upgrade_create(&ota_config);
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

    /* TODO: Do the actual hardware initialization here */
    accessory_init();

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);
    /* Initialize Wi-Fi */
    app_wifi_init();
    /* Register an event handler for HomeKit specific events.
     * All event handlers should be registered only after app_wifi_init()
     */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &accessory_hap_event_handler, NULL);
    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    while(1)
    {
        update_readings();
        vTaskDelay( 60000 / portTICK_RATE_MS );
    }
    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
}

void app_main()
{
    xTaskCreate(accessory_thread_entry, HAP_TASK_NAME, HAP_TASK_STACKSIZE, NULL, HAP_TASK_PRIORITY, NULL);
}
