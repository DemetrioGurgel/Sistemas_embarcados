#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "ssd1306.h"

void app_main(void)
{
    // Configuração para uso via I²C
    ssd1306_config_t config = {
        .bus = SSD1306_BUS_I2C,
        .iface.i2c = {
            .port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
            .clk_speed = 400000,  // 400 kHz
            .address = 0x3C,      
        },
    };

    // Inicializa o display SSD1306
    esp_err_t ret = ssd1306_init(&config);
    if (ret != ESP_OK) {
        printf("Erro ao inicializar o SSD1306\n");
        return;
    }

    // Limpa o buffer interno do display
    ssd1306_clear_buffer();
    
    // Define a posição do cursor (coluna 0, linha 0)
    ssd1306_set_cursor(0, 0);
    

    ssd1306_draw_text("Testandoo ABC 123!");
    
    // Atualiza o display com o conteúdo do buffer
    ssd1306_update();

    // Loop infinito para manter a task ativa
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
