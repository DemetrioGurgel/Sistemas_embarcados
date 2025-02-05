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

/** 
 * @brief Dimensões básicas do display SSD1306.
 *
 * Definem a largura e altura do display, bem como o tamanho do buffer 
 * (em bytes) necessário para armazenar todos os pixels (1 bit por pixel).
 */
#define SSD1306_WIDTH         128
#define SSD1306_HEIGHT        64
#define SSD1306_BUFFER_SIZE   (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/**
 * @brief Tipo de interface de comunicação.
 *
 * Especifica se a comunicação com o display será via I²C ou SPI.
 */
typedef enum {
    SSD1306_BUS_I2C,  /**< Interface I²C */
    SSD1306_BUS_SPI,  /**< Interface SPI  */
} ssd1306_bus_t;

/**
 * @brief Estrutura de configuração do display SSD1306.
 *
 * Esta estrutura contém todos os parâmetros necessários para inicializar e 
 * configurar o display, tanto para a interface I²C quanto para a SPI.
 */
typedef struct {
    ssd1306_bus_t bus;  /**< Seleciona a interface de comunicação (I²C ou SPI). */
    union {
        struct {
            i2c_port_t port;      /**< Porta I²C utilizada (por exemplo, I2C_NUM_0 ou I2C_NUM_1). */
            gpio_num_t sda_pin;   /**< Pino de dados (SDA). */
            gpio_num_t scl_pin;   /**< Pino de clock (SCL). */
            uint32_t clk_speed;   /**< Velocidade do clock I²C (por exemplo, 400000 para 400 kHz). */
            uint8_t address;      /**< Endereço I²C do display (geralmente 0x3C). */
        } i2c;
        struct {
            spi_host_device_t host;  /**< Host SPI utilizado (por exemplo, HSPI_HOST ou VSPI_HOST). */
            gpio_num_t cs_pin;       /**< Pino de Chip Select (CS). */
            gpio_num_t dc_pin;       /**< Pino de Data/Command (DC). */
            gpio_num_t mosi_pin;     /**< Pino de dados (MOSI). */
            gpio_num_t sclk_pin;     /**< Pino de clock SPI (SCLK). */
            gpio_num_t rst_pin;      /**< Pino de reset do display (RST). Pode ser definido como GPIO_NUM_NC se não for utilizado. */
            int dma_chan;            /**< Canal DMA utilizado (por exemplo, 1 ou 2; use 0 se não utilizar DMA). */
            int clock_speed_hz;      /**< Velocidade do clock SPI em Hertz (por exemplo, 10000000 para 10 MHz). */
        } spi;
    } iface;
} ssd1306_config_t;

/**
 * @brief Inicializa o display SSD1306.
 *
 * Configura e inicializa o display de acordo com os parâmetros fornecidos na 
 * estrutura de configuração. A função realiza a configuração da interface (I²C ou SPI)
 * e prepara o display para receber comandos e dados.
 *
 * @param config Ponteiro para a estrutura de configuração ::ssd1306_config_t.
 * @return esp_err_t ESP_OK em caso de sucesso ou um código de erro em caso de falha.
 */
esp_err_t ssd1306_init(const ssd1306_config_t *config);

/**
 * @brief Limpa o buffer de exibição.
 *
 * Zera todos os bytes do buffer interno, fazendo com que todos os pixels sejam 
 * definidos como apagados (0). Essa função não atualiza o display; para visualizar 
 * a alteração, é necessário chamar ::ssd1306_update.
 */
void ssd1306_clear_buffer(void);

/**
 * @brief Preenche o buffer de exibição.
 *
 * Define todos os pixels do buffer como ligados (1). Assim como em ::ssd1306_clear_buffer,
 * essa função atua apenas no buffer interno, sendo necessária uma chamada a ::ssd1306_update
 * para atualizar o display.
 */
void ssd1306_fill_buffer(void);

/**
 * @brief Atualiza o display com o conteúdo do buffer interno.
 *
 * Envia os dados armazenados no buffer para o display, atualizando a imagem mostrada.
 *
 * @return esp_err_t ESP_OK em caso de sucesso ou um código de erro em caso de falha na comunicação.
 */
esp_err_t ssd1306_update(void);

/**
 * @brief Desenha um pixel no buffer interno.
 *
 * Define o estado (ligado ou desligado) de um pixel específico no buffer de exibição.
 * É importante que as coordenadas estejam dentro dos limites do display (0 ≤ x < SSD1306_WIDTH,
 * 0 ≤ y < SSD1306_HEIGHT).
 *
 * @param x Coordenada X do pixel.
 * @param y Coordenada Y do pixel.
 * @param color Cor do pixel: 1 para ligado, 0 para desligado.
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief Define a posição do cursor para a escrita de texto.
 *
 * Define a posição inicial (em pixels) onde os caracteres serão desenhados 
 * quando se utilizar as funções de texto, como ::ssd1306_draw_char ou ::ssd1306_draw_text.
 *
 * @param x Coordenada X do cursor.
 * @param y Coordenada Y do cursor.
 */
void ssd1306_set_cursor(uint8_t x, uint8_t y);

/**
 * @brief Desenha um caractere utilizando uma fonte 5×7.
 *
 * Desenha o caractere especificado na posição atual do cursor e avança a posição
 * para o próximo caractere. A fonte utilizada é de tamanho 5×7 pixels.
 *
 * @param c Caractere a ser desenhado.
 */
void ssd1306_draw_char(char c);

/**
 * @brief Exibe uma string de texto no buffer.
 *
 * Desenha uma sequência de caracteres (string terminada com '\0') a partir da posição
 * atual do cursor. Cada caractere é desenhado utilizando a fonte 5×7.
 *
 * @param str Ponteiro para a string a ser exibida.
 */
void ssd1306_draw_text(const char* str);

/**
 * @brief Desenha uma linha utilizando o algoritmo de Bresenham.
 *
 * Traça uma linha reta entre os pontos (x0, y0) e (x1, y1) no buffer, utilizando o 
 * algoritmo de Bresenham para determinar os pixels que compõem a linha.
 *
 * @param x0 Coordenada X do ponto inicial.
 * @param y0 Coordenada Y do ponto inicial.
 * @param x1 Coordenada X do ponto final.
 * @param y1 Coordenada Y do ponto final.
 * @param color Cor da linha: 1 para pixels ligados, 0 para pixels desligados.
 */
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

/**
 * @brief Desenha um bitmap no buffer.
 *
 * Desenha uma imagem (bitmap) monocromática no buffer do display. O bitmap deve estar
 * organizado em ordem row-major, com 1 bit representando cada pixel.
 *
 * @param x Posição X inicial onde o bitmap será desenhado.
 * @param y Posição Y inicial onde o bitmap será desenhado.
 * @param bitmap Ponteiro para os dados do bitmap.
 * @param width Largura do bitmap em pixels.
 * @param height Altura do bitmap em pixels.
 */
void ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);

/**
 * @brief Desenha um círculo no buffer.
 *
 * Desenha um círculo centrado em (x0, y0) com o raio especificado, utilizando um algoritmo
 * apropriado (por exemplo, o algoritmo de Bresenham para círculos). O parâmetro color define
 * se os pixels do círculo serão ligados ou desligados.
 *
 * @param x0 Coordenada X do centro do círculo.
 * @param y0 Coordenada Y do centro do círculo.
 * @param radius Raio do círculo.
 * @param color Cor do círculo: 1 para pixels ligados, 0 para pixels desligados.
 */
void ssd1306_draw_circle(int x0, int y0, int radius, int color);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
