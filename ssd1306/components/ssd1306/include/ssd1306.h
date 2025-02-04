#ifndef SSD1306_H
#define SSD1306_H

#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Parâmetros básicos do display SSD1306 */
#define SSD1306_WIDTH         128
#define SSD1306_HEIGHT        64
#define SSD1306_BUFFER_SIZE   (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/** Definição do tipo de interface */
typedef enum {
    SSD1306_BUS_I2C,
    SSD1306_BUS_SPI,
} ssd1306_bus_t;

/** Estrutura de configuração para o display */
typedef struct {
    ssd1306_bus_t bus;
    union {
        struct {
            i2c_port_t port;
            gpio_num_t sda_pin;
            gpio_num_t scl_pin;
            uint32_t clk_speed;  // por exemplo, 400000
            uint8_t address;     // geralmente 0x3C
        } i2c;
        struct {
            spi_host_device_t host;
            gpio_num_t cs_pin;    // Chip Select
            gpio_num_t dc_pin;    // Data/Command
            gpio_num_t mosi_pin;  // Master Out Slave In
            gpio_num_t sclk_pin;  // Clock
            gpio_num_t rst_pin;   // Reset (ou GPIO_NUM_NC se não utilizado)
            int dma_chan;         // canal DMA (por exemplo, 1 ou 2; use 0 se não utilizar DMA)
            int clock_speed_hz;   // velocidade do clock SPI (por exemplo, 10000000 para 10 MHz)
        } spi;
    } iface;
} ssd1306_config_t;

/**
 * @brief Inicializa o display SSD1306 utilizando a interface configurada (I²C ou SPI).
 *
 * @param config Ponteiro para a estrutura de configuração.
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro.
 */
esp_err_t ssd1306_init(const ssd1306_config_t *config);

/**
 * @brief Limpa (zera) o buffer de exibição.
 */
void ssd1306_clear_buffer(void);

/**
 * @brief Preenche o buffer com todos os pixels ligados.
 */
void ssd1306_fill_buffer(void);

/**
 * @brief Atualiza o display com o conteúdo do buffer.
 *
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro.
 */
esp_err_t ssd1306_update(void);

/**
 * @brief Desenha um pixel no buffer.
 *
 * @param x Coordenada X.
 * @param y Coordenada Y.
 * @param color 1 para pixel ligado, 0 para desligado.
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief Define a posição do cursor para escrita de texto.
 *
 * @param x Coordenada X.
 * @param y Coordenada Y.
 */
void ssd1306_set_cursor(uint8_t x, uint8_t y);

/**
 * @brief Desenha um caractere utilizando uma fonte 5×7.
 *
 * @param c Caractere a ser desenhado.
 */
void ssd1306_draw_char(char c);

/**
 * @brief Exibe uma string de texto no buffer.
 *
 * @param str Ponteiro para a string terminada em '\0'.
 */
void ssd1306_draw_text(const char* str);

/**
 * @brief Desenha uma linha utilizando o algoritmo de Bresenham.
 *
 * @param x0 Coordenada X inicial.
 * @param y0 Coordenada Y inicial.
 * @param x1 Coordenada X final.
 * @param y1 Coordenada Y final.
 * @param color 1 para linha ligada, 0 para desligada.
 */
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

/**
 * @brief Desenha um bitmap no buffer.
 *
 * @param x Posição X inicial.
 * @param y Posição Y inicial.
 * @param bitmap Ponteiro para os dados do bitmap (1 bit por pixel, em ordem row-major).
 * @param width Largura do bitmap.
 * @param height Altura do bitmap.
 */
void ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
