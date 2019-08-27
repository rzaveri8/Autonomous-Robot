#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"

#define RECEIVER_PIN 21
int LOCATION = 0;

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, RECEIVER_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}

void read_location() {
  init_uart();
  uint8_t *data = (uint8_t *) malloc(1024);
  uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
  while(1) {
    int len = uart_read_bytes(UART_NUM_1, data, 1024, 20 / portTICK_RATE_MS);
    if (len > 0) {
      for (int i = 0; i < len; i++) {
        if ((int) data[i] == 10 && (int) data[i+1] < 4) {
          printf("%d\n", (int) data[i+1]);
          LOCATION = (int) data[i+1];
          break;
        }
      }
    }
  }
}
