#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"

// Definição dos pinos e configurações do I2C
#define I2C_SCL_PIN         GPIO_NUM_22  // Pino SCL do I2C
#define I2C_SDA_PIN         GPIO_NUM_21  // Pino SDA do I2C
#define I2C_NUM_PORT        I2C_NUM_0    // Número da porta I2C
#define SSD1306_I2C_ADDRESS 0x3C         // Endereço I2C do display SSD1306

static bool display_connected = false;  // Variável para verificar se o display está conectado

// Função para verificar a conexão I2C com o display
bool verify_i2c_connection() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();  // Cria um comando I2C
    i2c_master_start(cmd);  // Inicia a comunicação I2C
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);  // Envia o endereço do display no modo escrita
    i2c_master_stop(cmd);  // Para a comunicação I2C

    // Executa o comando I2C e verifica se houve sucesso na comunicação
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);  // Libera o comando I2C

    return (ret == ESP_OK);  // Retorna true se a comunicação foi bem-sucedida
}

// Função para verificar continuamente a conexão com o display
void check_display_connection() {
    while (!verify_i2c_connection()) {  // Verifica a conexão I2C
        if (display_connected) {
            ESP_LOGE("MAIN", "Erro: Display desconectado ou não respondendo! Aguardando reconexão...");
            display_connected = false;  // Atualiza o status do display para desconectado
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // Aguarda 5 segundos antes de verificar novamente
    }

    if (!display_connected) {
        ESP_LOGI("MAIN", "Display reconectado! Continuando execução...");
        display_connected = true;  // Atualiza o status do display para conectado
    }
}

// Função para desenhar um retângulo no display
void draw_rectangle() {
    ssd1306_draw_line(2, 2, 52, 2, 1);   // Linha superior
    ssd1306_draw_line(52, 2, 52, 27, 1); // Linha direita
    ssd1306_draw_line(52, 27, 2, 27, 1); // Linha inferior
    ssd1306_draw_line(2, 27, 2, 2, 1);   // Linha esquerda
}

// Função para desenhar um triângulo no display
void draw_triangle() {
    ssd1306_draw_line(70, 5, 120, 5, 1);  // Base do triângulo
    ssd1306_draw_line(120, 5, 95, 25, 1); // Lado direito do triângulo
    ssd1306_draw_line(95, 25, 70, 5, 1);  // Lado esquerdo do triângulo
}

// Função para desenhar um círculo no display
void draw_circle() {
    ssd1306_draw_circle(32, 48, 12, 1);  // Desenha um círculo com centro em (32, 48) e raio 12
}

// Função para desenhar uma linha diagonal no display
void draw_diagonal_line() {
    ssd1306_draw_line(70, 35, 120, 60, 1);  // Desenha uma linha diagonal de (70, 35) a (120, 60)
}

// Função principal do programa
void app_main(void)
{
    // Configuração do display SSD1306
    ssd1306_config_t config = {
        .bus = SSD1306_BUS_I2C,
        .iface.i2c = {
            .sda_pin   = I2C_SDA_PIN,    // Pino SDA
            .scl_pin   = I2C_SCL_PIN,    // Pino SCL
            .clk_speed = 400000,         // Velocidade do clock I2C (400 kHz)
            .port      = I2C_NUM_PORT,   // Porta I2C
            .address   = SSD1306_I2C_ADDRESS,  // Endereço I2C do display
        },
    };

    // Inicializa o display SSD1306
    if (ssd1306_init(&config) != ESP_OK) {
        ESP_LOGE("MAIN", "Erro ao iniciar o display SSD1306");
        return;  // Sai da função se a inicialização falhar
    }

    display_connected = true;  // Atualiza o status do display para conectado

    // Loop principal do programa
    while (1) {
        check_display_connection();  // Verifica a conexão com o display

        if (display_connected) {
            // Limpa o buffer do display e exibe um texto
            ssd1306_clear_buffer();
            ssd1306_set_cursor(0, 0);
            ssd1306_draw_text("Testandoo ABC 123!");
            ssd1306_update();
            vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos

            // Desenha um retângulo no display
            ssd1306_clear_buffer();
            draw_rectangle();
            ssd1306_update();
            vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos

            // Desenha um triângulo no display
            ssd1306_clear_buffer();
            draw_triangle();
            ssd1306_update();
            vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos

            // Desenha um círculo no display
            ssd1306_clear_buffer();
            draw_circle();
            ssd1306_update();
            vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos

            // Desenha uma linha diagonal no display
            ssd1306_clear_buffer();
            draw_diagonal_line();
            ssd1306_update();
            vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos
        }
    }
}
