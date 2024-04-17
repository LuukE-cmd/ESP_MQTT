#include <stdio.h>	
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// staticly defined wifi SSID and password. These are used to connect to the wifi network. In real applications these should be stored in a secure location.
#define WIFI_SSID "MSC Intern"
#define WIFI_PASS "%V0lt@2020"
#define WIFI_MAX_RETRY 8

// MQTT broker and topic definitions. These are used to connect to the MQTT broker and publish data to the MQTT topic.
#define MQTT_BROKER "mqtt://broker.hivemq.com"
#define MQTT_TOPIC "zuyd/autom/1642820"

// Placeholder MQTT topics for testing.
#define MQTT_SPEED_TOPIC "zuyd/autom/1642820/wind_speed"
#define MQTT_AMBIENT_TOPIC "zuyd/autom/1642820/ambient_temp"
#define MQTT_HUMIDITY_TOPIC "zuyd/autom/1642820/humidity"

// Define the tag for logging. This is used to identify the source of the log messages. In this case it is "mqtt tester".
static const char *TAG = "mqtt tester";

// Define the mqtt client, so it can be used in every scope of the file. This is currently used so it can be accessed in sensor reading functions.
esp_mqtt_client_handle_t client;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// GPIO PIN DEFINITIONS
#define GPIO_INTR_PIN 25
#define GPIO_INTR_PIN_SEL (1ULL<<GPIO_INTR_PIN)
#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_TIME 175 // 225 microseconds debounce

// GPIO AVERAGE VARIABLE DEFINITIONS
#define ARRAY_SIZE 10 // Size of the array to store the time differences between the last interrupt and the current interrupt.
static uint32_t time_diff_array[ARRAY_SIZE] = {0}; // Array to store the time differences between the last interrupt and the current interrupt.
static int array_index = 0; // Index to keep track of the current position in the array.

// Translation to km/h variable definitions
#define CIRCUMFERENCE 12 // Circumference of the measure point circle in cm devided by 2 (2 measurements per rotation)

// I2C SENSOR DEFINITIONS
#define I2C_SENSOR_PORT I2C_NUM_0
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 12

static int64_t last_interrupt_time = 0; 

static QueueHandle_t gpio_evt_queue = NULL; // Create a global queue to store the gpio events.


// Structure to store the gpio number and the time difference between the last interrupt and the current interrupt.
// This is used to send data to the gpio event queue.
typedef struct {
    uint32_t gpio_num;
    uint64_t time_diff;
} gpio_info_t;

typedef struct {
    double speed;
    double temp;
    double humidity;
} sensor_data_t;

static sensor_data_t sensor_data;

// this function is called when an interrupt is triggered on the GPIO pin
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint64_t current_time = esp_timer_get_time();       // Get the current time in microseconds.
    
    uint64_t time_diff = current_time - last_interrupt_time; // Set the time difference between the last interrupt and the current interrupt in the gpio info struct.
    if (time_diff < DEBOUNCE_TIME * 1000) { // Check if the time between the last interrupt and the current interrupt is less than the debounce time.
        
        return; // If the time between the last interrupt and the current interrupt is less than the debounce time, return and ignore the interrupt.
    }
    gpio_info_t *gpio_info = (gpio_info_t *)pvPortMalloc(sizeof(gpio_info_t));                              // Create a gpio info struct.
    gpio_info->gpio_num = (uint32_t) arg;                // Set the gpio number in the gpio info struct.
    gpio_info->time_diff = time_diff;                    // Set the time difference between the last interrupt and the current interrupt in the gpio info struct.
    if (xQueueSendFromISR(gpio_evt_queue, &gpio_info, NULL) != pdTRUE)  {
        vPortFree(gpio_info); // Free the gpio info struct.
        ESP_LOGI(TAG, "Failed to send gpio event to queue. freed memory.."); // Log that the gpio event failed to send to the queue.
    } 
    last_interrupt_time = current_time;                 // Set the last interrupt time to the current time.
}

// this function is called when an interrupt is triggered on the GPIO pin
static void gpio_task_example(void* arg)
{
    gpio_info_t *gpio_info; // Create a gpio info struct.
    for (;;) {  
        if (xQueueReceive(gpio_evt_queue, &gpio_info, portMAX_DELAY)) {  // Receive the gpio number from the gpio event queue. when the queue is empty, wait indefinitely. (takes no resources)
            
            // This is essentially a stack, the oldest value is overwritten by the newest value.
            // The average of this is the rolling average of the last 10 interrupt timedifferences.
            time_diff_array[array_index] = gpio_info->time_diff; 
            array_index = (array_index + 1) % ARRAY_SIZE; 
            uint64_t sum = 0; 
            for (int i = 0; i < ARRAY_SIZE; i++) { 
                sum += time_diff_array[i]; 
            }
            uint64_t average = sum / ARRAY_SIZE; // Calculate the average time difference.
            double speed = ((double)CIRCUMFERENCE) / ((double)average/36000.0); // Calculate the speed in km/h.

            printf("GPIO[%"PRIu32"] intr, val: %d\n", gpio_info->gpio_num, gpio_get_level(gpio_info->gpio_num)); // Print the gpio number and the level of the gpio pin.
            printf("current time: %lld\n", gpio_info->time_diff); // Print the current time.
            printf("average time: %lld\n", average); // Print the average time difference.
            printf("speed: %.2f\n\n", speed); // Print the speed in km/h.
            sensor_data.speed = speed; // Set the speed in the sensor data struct.
            vPortFree(gpio_info); // Free the gpio info struct.
        }
    }
}



// retry number for wifi connection
static int s_retry_num = 0;

// Event handler for the wifi station. This function will be called when an event occurs in the wifi station. It will log the event and take action based on the event.
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
// Initialize the wifi station. This function will initialize the wifi, create the event group, create the default event loop, create the default wifi station,
void wifi_init_sta(void);

// Event handler for the mqtt client. This function will be called when an event occurs in the mqtt client. It will log the event and take action based on the event.
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
// Initialize the mqtt client and start the mqtt client.
static void mqtt_start(void);
// Helper function to log errors in the mqtt event handler.
static void log_error_if_nonzero(const char *message, int error_code);

//publishing sensor data
static void publish_sensor_data(void* pvParameters);

static void i2c_master_init();


void app_main(void) {

    ESP_LOGI(TAG, "[APP] Startup..");                        // Log that the application is starting up.

////////////////////I2C CONFIGURATION//////////////////////
    ESP_LOGI(TAG, "I2C master initialization");                     // Log that the i2c master is being initialized.
    i2c_master_init();                                          // Initialize the i2c master bus and device.

///////////////////GPIO INTERRUPT CONFIGURATION//////////////////////
    printf("\n");
    ESP_LOGI(TAG, "GPIO interrupt configuration");                  // Log that the gpio interrupt is being configured.
    gpio_config_t io_conf = {};                                   // Create a gpio configuration struct.
    io_conf.intr_type = GPIO_INTR_NEGEDGE;                        // Set the interrupt type to negative edge. This means that the interrupt will be triggered when the pin goes from high to low.
    io_conf.pin_bit_mask = GPIO_INTR_PIN_SEL;                 // Set the pin bit mask to the GPIO_INTR_PIN. This is the pin that the interrupt will be triggered on.
    io_conf.mode = GPIO_MODE_INPUT;                               // Set the mode to input. This means that the pin will be used as an input.
    io_conf.pull_up_en = 0;                                       // Enable the pull-up resistor. This means that the pin will be pulled up to 3.3V when it is not connected to anything.
    gpio_config(&io_conf);                                        // Configure the gpio with the gpio configuration struct.

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));          // Create a queue to store the gpio events. This is used to store the gpio pin number when an interrupt is triggered.

    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL); // Create a task to handle the gpio events.

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);              // Install the gpio isr service. This is used to handle the gpio interrupts.
    gpio_isr_handler_add(GPIO_INTR_PIN, gpio_isr_handler, (void*) GPIO_INTR_PIN); // Add the gpio isr handler. This is used to handle the gpio interrupts on the GPIO_INTR_PIN.

    //printf("minimum free heap size: %u bytes\n", esp_get_minimum_free_heap_size()); // Print the minimum free heap size. This is the minimum amount of memory that is available to the application.

///////////////////WIFI AND MQTT CONFIGURATION//////////////////////

    printf("\n");
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");                                             // Log that the wifi mode is set to station.

    esp_err_t ret = nvs_flash_init();                       // Initialize the non-volatile storage (NVS) flash. This is used to store the wifi configuration.
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { // Check if the NVS flash is full or if a new version is found.
      ESP_ERROR_CHECK(nvs_flash_erase());                                           // Erase the NVS flash if it is full or a new version is found.
      ret = nvs_flash_init();                                                       // Initialize the NVS flash again.
    }
    ESP_ERROR_CHECK(ret);                                                           // Check if the NVS flash initialization was successful.

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");                                             // Log that the wifi mode is set to station.
    wifi_init_sta();                                                                // Initialize the wifi station and connect to the AP defined in the WIFI_SSID and WIFI_PASS macros.

    mqtt_start();                                                                   // Initialize the mqtt client and connect to the MQTT broker defined in the MQTT_BROKER macro.
    xTaskCreate(&publish_sensor_data, "publish_sensor_data", 4096, NULL, 5, NULL);  // Create a task to simulate sensor data and publish it to the MQTT topic.
}

// This code is based on the example code provided by Espressif Systems in the ESP-IDF documentation. I used it as a reference to implement the WiFi client in this project.
// It can be found at https://github.com/espressif/esp-idf/blob/v5.2.1/examples/wifi/getting_started/softAP/main/softap_example_main.c

// What it does: the event handler gets called on these events: WIFI_EVENT_STA_START, WIFI_EVENT_STA_DISCONNECTED, IP_EVENT_STA_GOT_IP
// The event handler will try to connect to the AP, if it fails it will retry until the maximum number of retries is reached.
// If the connection is successful, the event handler will set the WIFI_CONNECTED_BIT in the event group.

// esp_event_base_t is a type that represents the base of an event. It is used to identify the source of the event. in this case it will check if it is a WIFI_EVENT or IP_EVENT.
// event_id is the id of the event. In this case it will check if it is WIFI_EVENT_STA_START, WIFI_EVENT_STA_DISCONNECTED or IP_EVENT_STA_GOT_IP.
// event_data is a pointer to the data of the event. In this case it will be a pointer to the ip_event_got_ip_t struct that contains the IP address of the device.

// Using events is better than continuously polling the status of the connection because it is more efficient and uses less resources.

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // Check if the event is a Wifi event and if so, check if it is the start event. When true, connect to the AP.
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {   // Check if event is wifi event and is the disconnect event. 
        if (s_retry_num < WIFI_MAX_RETRY) {                                             // Check if the number of retries is less than the maximum number of retries.
            esp_wifi_connect();                                                         // Try to connect to the AP again.
            s_retry_num++;                                                              // Increment the number of retries.
            ESP_LOGI(TAG, "retry to connect to the AP");                                // Log the retry.
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);                      // If the maximum number of retries is reached, set the WIFI_FAIL_BIT in the event group.
        }
        ESP_LOGI(TAG,"connect to the AP fail");                                         // Log that the connection to the AP failed.
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {             // Check if event is wifi event and got IP event.
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;                     // Get the IP address from the event data.
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));                     // Log the IP address.
        s_retry_num = 0;                                                                // Reset the number of retries.
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);                     // Set the WIFI_CONNECTED_BIT in the event group.
    }
}

// This is the wifi initialization function. It will initialize the wifi, create the event group, create the default event loop, create the default wifi station,
// create the default wifi configuration, set the wifi mode to station, set the wifi configuration, start the wifi, 
// register the event handler, and wait for the WIFI_CONNECTED_BIT or WIFI_FAIL_BIT to be set in the event group.

// The default event loop will be used in the mqtt client as well, so it is required to run this function before starting the mqtt client. (in future should be moved to its own function)

void wifi_init_sta(void) {
        s_wifi_event_group = xEventGroupCreate();                   // Create the event group. This is used to signal when we are connected to the AP with an IP or when we failed to connect after the maximum amount of retries.

    ESP_ERROR_CHECK(esp_netif_init());                              // Initialize the tcp/ip adapter. This is required for the wifi to work.

    ESP_ERROR_CHECK(esp_event_loop_create_default());               // Create the default event loop. used in wifi eventhandler and mqtt eventhandler.
    esp_netif_create_default_wifi_sta();                            // Create the default wifi station.

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();            // Create the default wifi configuration. This is enough for most use cases.
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                           // Initialize the wifi with the default configuration.

    esp_event_handler_instance_t instance_any_id;                   // Create the event handler instances...
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));     // Register the event handler for any wifi event.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));    // Register the event handler for the got IP event.

    wifi_config_t wifi_config = {                           // Create the wifi configuration. This is used to set the SSID, password, and authentication mode.
        .sta = {                                            // The station configuration struct.
            .ssid = WIFI_SSID,                              // The SSID and password are staticly defined in the code, in real applications these should be stored in a secure location.
            .password = WIFI_PASS,
            .bssid_set = false,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,       // The authentication mode is set to WPA2 PSK.
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );                 // Set the wifi mode to station. This is defined in the library. not changed in this project.
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );   // Set the wifi configuration. This is the configuration that was created earlier.
    ESP_ERROR_CHECK(esp_wifi_start() );                                 // Start the wifi. This will start the wifi station.

    ESP_LOGI(TAG, "wifi_init_sta finished.");                           // Log that the wifi initialization is finished.

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,          // Wait for the WIFI_CONNECTED_BIT or WIFI_FAIL_BIT to be set in the event group.
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,                                                    // pdFALSE means that the function will return when either bit is set. 
            pdFALSE,                                                    // pdTRUE means that the function will return when both bits are set.
            portMAX_DELAY);                                             // maximum delay is set to portMAX_DELAY. This is the maximum value of a 32 bit unsigned integer.


    if (bits & WIFI_CONNECTED_BIT) {                                    // Check if the WIFI_CONNECTED_BIT is set. If so, log that the device is connected to the AP.
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {                                  // Check if the WIFI_FAIL_BIT is set. If so, log that the device failed to connect to the AP.
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {                                                            // If neither bit is set, log that an unexpected event occured.
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

}  

// MQTT CODE BASED ON https://github.com/espressif/esp-idf/blob/v5.2.1/examples/protocols/mqtt/tcp/main/app_main.c
// This is an example of a simple MQTT client that sends and receives data from an MQTT broker. I used it as a reference to implement the MQTT client in this project.

// mqtt_event_handler takes the following arguments:
// handler_args: a pointer to the handler arguments. In this case it is NULL.
// base: the base of the event. In this case it is ESP_EVENT_ANY_ID.
// event_id: the id of the event. In this case it is the event id of the mqtt event.
// event_data: a pointer to the data of the event. In this case it is a pointer to the esp_mqtt_event_handle_t struct.

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    //esp_mqtt_client_handle_t client = event->client;
    //int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {                                            // Cast the event_id to the esp_mqtt_event_id_t type and check the event_id.
    case MQTT_EVENT_CONNECTED:                                                          // Check if the event is the connected event: log connection, sub to topic (for testing).             
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:                                                       // Check if the event is the disconnected event: log disconnection.
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:                                                         // Check if the event is the subscribed event: log subscription.
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:                                                       // Check if the event is the unsubscribed event: log unsubscription.
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:                                                          // Check if the event is the published event: log publication.
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:                                                               // Check if the event is the data event: log topic and data.
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:                                                              // Check if the event is the error event: log error. (different error types are logged separately)
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:                                                                            // If the event is not any of the above, log the event id.
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// mqtt_start initializes the mqtt client with the mqtt configuration and starts the mqtt client. It also registers the mqtt event handler.

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {                           // Create the mqtt configuration struct.
        .broker.address.uri = MQTT_BROKER,                          // Set the broker address to the MQTT_BROKER defined at the top of the file. (mqtt://broker.hivemq.com)
        .broker.address.port = 1883,                                // Set the broker port to 1883. This is the default port for MQTT. (unencrypted and unauthenticated)
    };

    client = esp_mqtt_client_init(&mqtt_cfg);                       // Initialize the mqtt client with the mqtt configuration.
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); // Register the mqtt event handler.
    esp_mqtt_client_start(client);                                  // Start the mqtt client.
}

// log_error_if_nonzero logs an error message if the error code is not zero. used as a helper function in the mqtt event handler when an error event occurs. (prevent repeating code)

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// This method sends the sensor data to the MQTT broker every 10 seconds.
// The sensor data is read from the static memory location sensor_data.

static void publish_sensor_data(void* pvParameters) {
    ESP_LOGI(TAG, "Publishing sensor data...");                             // Log that the sensor data is being published.
    while (1) {                                                             // Initialize loop for simulating sensor reading.
    
    char str_speed[12];                                                      // Create a char array to store the speed as a string. MQTT messages must be a string (max 12 characters)
    sprintf(str_speed, "%.2f", sensor_data.speed);                            // Convert the speed to a string.
    sensor_data.speed = 0;                                                  // Reset the speed to 0.
    char str_amibient[12];                                                   // Create a char array to store the ambient temperature as a string. MQTT messages must be a string (max 12 characters)
    sprintf(str_amibient, "%.2f", sensor_data.temp);                          // Convert the ambient temperature to a string.
    char str_humidity[12];                                                   // Create a char array to store the humidity as a string. MQTT messages must be a string (max 12 characters)
    sprintf(str_humidity, "%.2f", sensor_data.humidity);                      // Convert the humidity to a string.
    esp_mqtt_client_publish(client, MQTT_SPEED_TOPIC, str_speed, 0, 1, 0);    // Publish the speed to the MQTT topic. (zuyd/autom/1642820/pub1)
    esp_mqtt_client_publish(client, MQTT_AMBIENT_TOPIC, str_amibient, 0, 1, 0); // Publish the ambient temperature to the MQTT topic. (zuyd/autom/1642820/pub2)
    esp_mqtt_client_publish(client, MQTT_HUMIDITY_TOPIC, str_humidity, 0, 1, 0); // Publish the humidity to the MQTT topic. (zuyd/autom/1642820/pub3)
    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10 seconds
  }
}

static void i2c_master_init() {

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_SENSOR_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = 7,
        .device_address = 0x44,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    esp_err_t ret = i2c_master_probe(bus_handle, 0x44, 300);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C device found");
    } else {
        ESP_LOGE(TAG, "I2C device not found");
    }
}