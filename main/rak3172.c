#include <stdio.h>

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "am2302_rmt.h"


#define UART_PORT UART_NUM_1   // UART1
#define TX_PIN    4            // Pino TX
#define RX_PIN    5            // Pino RX
#define RST_PIN   6
#define BUF_SIZE (1024)

#define AM2302_GPIO  0

const char *TAG = "UART1";


void gpio_init(void)
{
    gpio_reset_pin(RST_PIN);
    gpio_set_pull_mode(RST_PIN, GPIO_PULLUP_ENABLE);
    gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RST_PIN, 0); 
}

void send_at(const char *cmd, int wait_ms) {
    ESP_LOGI(TAG, ">> %s", cmd);
    uart_flush(UART_PORT);
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));

    vTaskDelay(pdMS_TO_TICKS(wait_ms));

    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
    if (len > 0) {
        data[len] = '\0';
        ESP_LOGI(TAG, "<< %s", (char *)data);
    } else {
        ESP_LOGW(TAG, "<< (sem resposta)");
    }
}

void app_main(void)
{


    am2302_config_t am2302_config = {
        .gpio_num = AM2302_GPIO,
    };
    am2302_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
    };
    am2302_handle_t sensor = NULL;
    ESP_ERROR_CHECK(am2302_new_sensor_rmt(&am2302_config, &rmt_config, &sensor));

    ESP_LOGI(TAG, "Start reading temperature and humidity from AM2302 sensor");
    float temperature = 0;
    float humidity = 0;
    
    gpio_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(RST_PIN, 1); // reset RAK3172
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configuração da UART
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Inicializa a UART
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(500));
    if (len > 0) {
        data[len] = '\0';
        ESP_LOGI(TAG, "<< %s", (char *)data);
    } else {
        ESP_LOGW(TAG, "<< (sem resposta)");
    }

    send_at("AT\r\n", 500);
    //send_at("AT+VER?\r\n", 500);
    send_at("AT+NWM=1\r\n", 500);
    send_at("AT+NJM=1\r\n", 500);    
    send_at("AT+CLASS=A\r\n", 500);  
    send_at("AT+BAND=6\r\n", 500);
    send_at("AT+TXP=6\r\n", 500);
    send_at("AT+DEVEUI=5E9D1E0857CF25F1\r\n", 500);  
    send_at("AT+APPEUI=5E9D1E0857CF25F1\r\n", 500);  
    send_at("AT+APPKEY=F921D50CD7D02EE3C5E6142154F274B2\r\n", 500);     
    send_at("AT+JOIN=1:0:10:8\r\n", 500);
    send_at("AT+SEND=2:12345678\r\n", 500);   

    while (true) {
        ESP_ERROR_CHECK(am2302_read_temp_humi(sensor, &temperature, &humidity));
        ESP_LOGI(TAG, "Temperature: %.1f °C, Humidity: %.1f %%", temperature, humidity);
        int16_t temp_int = (int16_t)(temperature * 10); // 25.6°C -> 256
        uint8_t hum_int = (uint8_t)(humidity * 2);      // 60% -> 120
        uint8_t payload[3];        
        payload[0] = (temp_int >> 8) & 0xFF;
        payload[1] = temp_int & 0xFF;
        payload[2] = hum_int;
        // Converter payload para string hexadecimal
        char hex_payload[7]; // 3 bytes -> 6 chars + '\0'
        snprintf(hex_payload, sizeof(hex_payload), "%02X%02X%02X", payload[0], payload[1], payload[2]);

        // Enviar para RAK3172 (porta 2, ex: AT+SEND=2:<hex>)
        char at_cmd[32];
        snprintf(at_cmd, sizeof(at_cmd), "AT+SEND=2:%s\r\n", hex_payload);
        send_at(at_cmd, 500);
        //send_at("AT+SEND=2:12345678\r\n", 500);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
}