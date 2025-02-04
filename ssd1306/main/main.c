/* #include <stdio.h>
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
*/

#include "ssd1306.h"
#include "esp_log.h"

void app_main(void) {
    ssd1306_config_t config = {
        .bus = SSD1306_BUS_I2C,
        .iface.i2c = {
            .port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
            .clk_speed = 400000,
            .address = 0x3C
        }
    };

    // Inicialização
    ESP_ERROR_CHECK(ssd1306_init(&config));

    // Desenho de elementos
    ssd1306_clear_buffer();
    
    // Texto
    ssd1306_set_cursor(10, 5);
    ssd1306_draw_text("OLED Test");
    
    // Formas geométricas
    ssd1306_draw_line(0, 15, 127, 15, 1);       // Linha horizontal
    ssd1306_draw_line(64, 16, 64, 63, 1);       // Linha vertical
    
    // Retângulo
    for(uint8_t i = 0; i < 5; i++) {
        ssd1306_draw_line(10+i, 20+i, 50-i, 20+i, 1);
        ssd1306_draw_line(10+i, 40-i, 50-i, 40-i, 1);
    }
    
    // Bitmap (exemplo: smile 16x16)
    const uint8_t smile[] = {
        0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 
        0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00
    };
    ssd1306_draw_bitmap(80, 20, smile, 16, 2);

    // Atualiza o display
    ESP_ERROR_CHECK(ssd1306_update());

    ESP_LOGI("MAIN", "Display atualizado com sucesso!");
}