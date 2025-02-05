#include "ssd1306.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

#define TAG "SSD1306"

/* Buffer interno de exibição e coordenadas do cursor */
static uint8_t ssd1306_buffer[SSD1306_BUFFER_SIZE];  /**< Buffer de 1 bit por pixel para o display SSD1306 */
static uint8_t cursor_x = 0;   /**< Coordenada X do cursor para escrita de texto */
static uint8_t cursor_y = 0;   /**< Coordenada Y do cursor para escrita de texto */

/* Variável global de configuração e handle SPI (se utilizado) */
static ssd1306_config_t ssd1306_dev;                 /**< Variável global que armazena a configuração do display */
static spi_device_handle_t ssd1306_spi_handle = NULL;  /**< Handle para o dispositivo SPI, válido se o bus configurado for SPI */

/* --- Funções auxiliares de comunicação --- */

/**
 * @brief Envia um comando para o display SSD1306.
 *
 * De acordo com a interface configurada (I²C ou SPI), esta função prepara e envia
 * um comando para o display.
 *
 * @param cmd Comando a ser enviado.
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro em falhas de comunicação.
 */
static esp_err_t ssd1306_send_command(uint8_t cmd)
{
    if (ssd1306_dev.bus == SSD1306_BUS_I2C) {
        i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
        esp_err_t ret;
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (ssd1306_dev.iface.i2c.address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_handle, 0x00, true);  // 0x00: Co = 0, D/C# = 0 (comando)
        i2c_master_write_byte(cmd_handle, cmd, true);
        i2c_master_stop(cmd_handle);
        ret = i2c_master_cmd_begin(ssd1306_dev.iface.i2c.port, cmd_handle, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd_handle);
        return ret;
    }
    else if (ssd1306_dev.bus == SSD1306_BUS_SPI) {
        /* Para SPI, define o pino D/C como 0 para indicar comando */
        gpio_set_level(ssd1306_dev.iface.spi.dc_pin, 0);
        spi_transaction_t t = {
            .length = 8,      // Tamanho em bits
            .tx_buffer = &cmd,
        };
        return spi_device_transmit(ssd1306_spi_handle, &t);
    }
    return ESP_ERR_INVALID_STATE;
}

/**
 * @brief Envia dados para o display SSD1306.
 *
 * Transmite um bloco de dados (buffer) para o display, utilizando a interface 
 * configurada (I²C ou SPI).
 *
 * @param data Ponteiro para os dados a serem enviados.
 * @param length Número de bytes a serem transmitidos.
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro em falhas de comunicação.
 */
static esp_err_t ssd1306_send_data(const uint8_t *data, size_t length)
{
    if (ssd1306_dev.bus == SSD1306_BUS_I2C) {
        i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
        esp_err_t ret;
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (ssd1306_dev.iface.i2c.address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_handle, 0x40, true);  // 0x40: Co = 0, D/C# = 1 (dados)
        i2c_master_write(cmd_handle, data, length, true);
        i2c_master_stop(cmd_handle);
        ret = i2c_master_cmd_begin(ssd1306_dev.iface.i2c.port, cmd_handle, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd_handle);
        return ret;
    }
    else if (ssd1306_dev.bus == SSD1306_BUS_SPI) {
        /* Para SPI, define o pino D/C como 1 para indicar dados */
        gpio_set_level(ssd1306_dev.iface.spi.dc_pin, 1);
        spi_transaction_t t = {
            .length = length * 8,   // Tamanho em bits
            .tx_buffer = data,
        };
        return spi_device_transmit(ssd1306_spi_handle, &t);
    }
    return ESP_ERR_INVALID_STATE;
}

/* --- Função de inicialização --- */

/**
 * @brief Inicializa o display SSD1306.
 *
 * Configura a interface de comunicação (I²C ou SPI), inicializa o barramento, 
 * realiza a sequência de inicialização do display e atualiza o buffer inicial.
 *
 * @param config Ponteiro para a estrutura de configuração ::ssd1306_config_t.
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro em falhas.
 */
esp_err_t ssd1306_init(const ssd1306_config_t *config)
{
    memcpy(&ssd1306_dev, config, sizeof(ssd1306_config_t));
    esp_err_t err = ESP_OK;

    if (ssd1306_dev.bus == SSD1306_BUS_I2C) {
        /* Configuração e instalação do driver I²C */
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = ssd1306_dev.iface.i2c.sda_pin,
            .scl_io_num = ssd1306_dev.iface.i2c.scl_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = ssd1306_dev.iface.i2c.clk_speed,
        };
        err = i2c_param_config(ssd1306_dev.iface.i2c.port, &i2c_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro na configuração do I²C");
            return err;
        }
        err = i2c_driver_install(ssd1306_dev.iface.i2c.port, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao instalar driver I²C");
            return err;
        }
    }
    else if (ssd1306_dev.bus == SSD1306_BUS_SPI) {
        /* Configuração do barramento SPI */
        spi_bus_config_t buscfg = {
            .mosi_io_num = ssd1306_dev.iface.spi.mosi_pin,
            .miso_io_num = -1,  // Não utilizado pelo SSD1306
            .sclk_io_num = ssd1306_dev.iface.spi.sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SSD1306_BUFFER_SIZE + 1,
        };
        err = spi_bus_initialize(ssd1306_dev.iface.spi.host, &buscfg, ssd1306_dev.iface.spi.dma_chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao inicializar barramento SPI");
            return err;
        }
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = ssd1306_dev.iface.spi.clock_speed_hz,
            .mode = 0,                        // SPI Mode 0
            .spics_io_num = ssd1306_dev.iface.spi.cs_pin,
            .queue_size = 7,
        };
        err = spi_bus_add_device(ssd1306_dev.iface.spi.host, &devcfg, &ssd1306_spi_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao adicionar dispositivo SPI");
            return err;
        }
        /* Configura o pino D/C como saída */
        gpio_set_direction(ssd1306_dev.iface.spi.dc_pin, GPIO_MODE_OUTPUT);
        /* Se houver pino de RESET, realiza a sequência de reset */
        if (ssd1306_dev.iface.spi.rst_pin != GPIO_NUM_NC) {
            gpio_set_direction(ssd1306_dev.iface.spi.rst_pin, GPIO_MODE_OUTPUT);
            gpio_set_level(ssd1306_dev.iface.spi.rst_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(ssd1306_dev.iface.spi.rst_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Aguarda estabilização do display

    /* Sequência de inicialização do SSD1306 */
    uint8_t init_cmds[] = {
        0xAE,                         // Display off
        0xD5, 0x80,                   // Set display clock divide ratio/oscillator frequency
        0xA8, SSD1306_HEIGHT - 1,      // Set multiplex ratio
        0xD3, 0x00,                   // Set display offset
        0x40,                         // Set start line address
        0x8D, 0x14,                   // Enable charge pump
        0x20, 0x00,                   // Memory addressing mode: Horizontal addressing
        0xA1,                         // Segment re-map (col 127 → SEG0)
        0xC8,                         // COM Output Scan Direction remapped
        0xDA, 0x12,                   // Set COM pins hardware configuration
        0x81, 0x7F,                   // Set contrast control
        0xD9, 0xF1,                   // Set pre-charge period
        0xDB, 0x40,                   // Set VCOMH deselect level
        0xA4,                         // Resume display (RAM content)
        0xA6,                         // Normal display (não invertido)
        0x2E,                         // Deactivate scroll
        0xAF                          // Display on
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = ssd1306_send_command(init_cmds[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha na inicialização, comando: 0x%02X", init_cmds[i]);
            return err;
        }
    }

    ssd1306_clear_buffer();
    ssd1306_update();

    ESP_LOGI(TAG, "SSD1306 inicializado com sucesso");
    return ESP_OK;
}

/* --- Manipulação do buffer --- */

/**
 * @brief Limpa o buffer interno do display.
 *
 * Zera todos os bytes do buffer, apagando todos os pixels, e reseta as posições
 * do cursor para (0,0).
 */
void ssd1306_clear_buffer(void)
{
    memset(ssd1306_buffer, 0x00, SSD1306_BUFFER_SIZE);
    cursor_x = 0;
    cursor_y = 0;
}

/**
 * @brief Preenche o buffer interno do display.
 *
 * Define todos os pixels do buffer como ligados (valor 0xFF em cada byte).
 */
void ssd1306_fill_buffer(void)
{
    memset(ssd1306_buffer, 0xFF, SSD1306_BUFFER_SIZE);
}

/**
 * @brief Atualiza o display com o conteúdo do buffer interno.
 *
 * Configura os endereços de coluna e página do display e envia o buffer completo
 * para atualização da imagem exibida.
 *
 * @return esp_err_t ESP_OK em caso de sucesso ou código de erro em falha de comunicação.
 */
esp_err_t ssd1306_update(void)
{
    esp_err_t err;
    /* Define os endereços de coluna e página antes de enviar o buffer */
    err = ssd1306_send_command(0x21);  // Set Column Address
    if (err != ESP_OK) return err;
    err = ssd1306_send_command(0x00);  // Start column 0
    if (err != ESP_OK) return err;
    err = ssd1306_send_command(SSD1306_WIDTH - 1);  // End column
    if (err != ESP_OK) return err;

    err = ssd1306_send_command(0x22);  // Set Page Address
    if (err != ESP_OK) return err;
    err = ssd1306_send_command(0x00);  // Start page 0
    if (err != ESP_OK) return err;
    err = ssd1306_send_command((SSD1306_HEIGHT / 8) - 1);  // End page
    if (err != ESP_OK) return err;

    /* Envia o buffer completo para o display */
    err = ssd1306_send_data(ssd1306_buffer, SSD1306_BUFFER_SIZE);
    return err;
}

/* --- Funções gráficas --- */

/**
 * @brief Desenha um pixel no buffer interno.
 *
 * Calcula o índice e a máscara de bit correspondentes à posição (x, y) e define o
 * pixel como ligado ou desligado, conforme o valor de color.
 *
 * @param x Coordenada X do pixel.
 * @param y Coordenada Y do pixel.
 * @param color 1 para ligar o pixel, 0 para apagá-lo.
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    uint16_t byte_index = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_mask = 1 << (y % 8);
    if (color)
        ssd1306_buffer[byte_index] |= bit_mask;
    else
        ssd1306_buffer[byte_index] &= ~bit_mask;
}

/**
 * @brief Define a posição do cursor para escrita de texto.
 *
 * Atualiza as variáveis globais que determinam a posição onde o próximo caractere
 * será desenhado.
 *
 * @param x Nova coordenada X do cursor.
 * @param y Nova coordenada Y do cursor.
 */
void ssd1306_set_cursor(uint8_t x, uint8_t y)
{
    cursor_x = x;
    cursor_y = y;
}

/* Fonte 5x7 para renderização de caracteres */
static const uint8_t font5x7[][5] = {
    /*  0 (32) ' ' */ {0x00, 0x00, 0x00, 0x00, 0x00},
    /*  1 (33) '!' */ {0x00, 0x00, 0x5F, 0x00, 0x00},
    /*  2 (34) '"' */ {0x00, 0x07, 0x00, 0x07, 0x00},
    /*  3 (35) '#' */ {0x14, 0x7F, 0x14, 0x7F, 0x14},
    /*  4 (36) '$' */ {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    /*  5 (37) '%' */ {0x23, 0x13, 0x08, 0x64, 0x62},
    /*  6 (38) '&' */ {0x36, 0x49, 0x55, 0x22, 0x50},
    /*  7 (39) ''' */ {0x00, 0x05, 0x03, 0x00, 0x00},
    /*  8 (40) '(' */ {0x00, 0x1C, 0x22, 0x41, 0x00},
    /*  9 (41) ')' */ {0x00, 0x41, 0x22, 0x1C, 0x00},
    /* 10 (42) '*' */ {0x14, 0x08, 0x3E, 0x08, 0x14},
    /* 11 (43) '+' */ {0x08, 0x08, 0x3E, 0x08, 0x08},
    /* 12 (44) ',' */ {0x00, 0x50, 0x30, 0x00, 0x00},
    /* 13 (45) '-' */ {0x08, 0x08, 0x08, 0x08, 0x08},
    /* 14 (46) '.' */ {0x00, 0x60, 0x60, 0x00, 0x00},
    /* 15 (47) '/' */ {0x20, 0x10, 0x08, 0x04, 0x02},

    /* 16 (48) '0' */ {0x3E, 0x51, 0x49, 0x45, 0x3E},
    /* 17 (49) '1' */ {0x00, 0x42, 0x7F, 0x40, 0x00},
    /* 18 (50) '2' */ {0x42, 0x61, 0x51, 0x49, 0x46},
    /* 19 (51) '3' */ {0x21, 0x41, 0x45, 0x4B, 0x31},
    /* 20 (52) '4' */ {0x18, 0x14, 0x12, 0x7F, 0x10},
    /* 21 (53) '5' */ {0x27, 0x45, 0x45, 0x45, 0x39},
    /* 22 (54) '6' */ {0x3C, 0x4A, 0x49, 0x49, 0x30},
    /* 23 (55) '7' */ {0x01, 0x71, 0x09, 0x05, 0x03},
    /* 24 (56) '8' */ {0x36, 0x49, 0x49, 0x49, 0x36},
    /* 25 (57) '9' */ {0x06, 0x49, 0x49, 0x29, 0x1E},
    /* 26 (58) ':' */ {0x00, 0x36, 0x36, 0x00, 0x00},
    /* 27 (59) ';' */ {0x00, 0x56, 0x36, 0x00, 0x00},
    /* 28 (60) '<' */ {0x08, 0x14, 0x22, 0x41, 0x00},
    /* 29 (61) '=' */ {0x14, 0x14, 0x14, 0x14, 0x14},
    /* 30 (62) '>' */ {0x00, 0x41, 0x22, 0x14, 0x08},
    /* 31 (63) '?' */ {0x02, 0x01, 0x51, 0x09, 0x06},
    /* 32 (64) '@' */ {0x3E, 0x41, 0x5D, 0x55, 0x1E},

    /* 33 (65) 'A' */ {0x7E, 0x11, 0x11, 0x11, 0x7E},
    /* 34 (66) 'B' */ {0x7F, 0x49, 0x49, 0x49, 0x36},
    /* 35 (67) 'C' */ {0x3E, 0x41, 0x41, 0x41, 0x22},
    /* 36 (68) 'D' */ {0x7F, 0x41, 0x41, 0x22, 0x1C},
    /* 37 (69) 'E' */ {0x7F, 0x49, 0x49, 0x49, 0x41},
    /* 38 (70) 'F' */ {0x7F, 0x09, 0x09, 0x09, 0x01},
    /* 39 (71) 'G' */ {0x3E, 0x41, 0x49, 0x49, 0x7A},
    /* 40 (72) 'H' */ {0x7F, 0x08, 0x08, 0x08, 0x7F},
    /* 41 (73) 'I' */ {0x00, 0x41, 0x7F, 0x41, 0x00},
    /* 42 (74) 'J' */ {0x20, 0x40, 0x41, 0x3F, 0x01},
    /* 43 (75) 'K' */ {0x7F, 0x08, 0x14, 0x22, 0x41},
    /* 44 (76) 'L' */ {0x7F, 0x40, 0x40, 0x40, 0x40},
    /* 45 (77) 'M' */ {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    /* 46 (78) 'N' */ {0x7F, 0x04, 0x08, 0x10, 0x7F},
    /* 47 (79) 'O' */ {0x3E, 0x41, 0x41, 0x41, 0x3E},
    /* 48 (80) 'P' */ {0x7F, 0x09, 0x09, 0x09, 0x06},
    /* 49 (81) 'Q' */ {0x3E, 0x41, 0x51, 0x21, 0x5E},
    /* 50 (82) 'R' */ {0x7F, 0x09, 0x19, 0x29, 0x46},
    /* 51 (83) 'S' */ {0x46, 0x49, 0x49, 0x49, 0x31},
    /* 52 (84) 'T' */ {0x01, 0x01, 0x7F, 0x01, 0x01},
    /* 53 (85) 'U' */ {0x3F, 0x40, 0x40, 0x40, 0x3F},
    /* 54 (86) 'V' */ {0x1F, 0x20, 0x40, 0x20, 0x1F},
    /* 55 (87) 'W' */ {0x3F, 0x40, 0x30, 0x40, 0x3F},
    /* 56 (88) 'X' */ {0x63, 0x14, 0x08, 0x14, 0x63},
    /* 57 (89) 'Y' */ {0x07, 0x08, 0x70, 0x08, 0x07},
    /* 58 (90) 'Z' */ {0x61, 0x51, 0x49, 0x45, 0x43},

    /* 59 (91) '[' */ {0x00, 0x7F, 0x41, 0x41, 0x00},
    /* 60 (92) '\' */ {0x02, 0x04, 0x08, 0x10, 0x20},
    /* 61 (93) ']' */ {0x00, 0x41, 0x41, 0x7F, 0x00},
    /* 62 (94) '^' */ {0x04, 0x02, 0x01, 0x02, 0x04},
    /* 63 (95) '_' */ {0x40, 0x40, 0x40, 0x40, 0x40},
    /* 64 (96) '`' */ {0x00, 0x01, 0x02, 0x04, 0x00},

    /* 65 (97) 'a' */ {0x20, 0x54, 0x54, 0x54, 0x78},
    /* 66 (98) 'b' */ {0x7F, 0x48, 0x44, 0x44, 0x38},
    /* 67 (99) 'c' */ {0x38, 0x44, 0x44, 0x44, 0x20},
    /* 68 (100) 'd' */ {0x38, 0x44, 0x44, 0x48, 0x7F},
    /* 69 (101) 'e' */ {0x38, 0x54, 0x54, 0x54, 0x18},
    /* 70 (102) 'f' */ {0x08, 0x7E, 0x09, 0x01, 0x02},
    /* 71 (103) 'g' */ {0x0C, 0x52, 0x52, 0x52, 0x3E},
    /* 72 (104) 'h' */ {0x7F, 0x08, 0x04, 0x04, 0x78},
    /* 73 (105) 'i' */ {0x00, 0x44, 0x7D, 0x40, 0x00},
    /* 74 (106) 'j' */ {0x20, 0x40, 0x44, 0x3D, 0x00},
    /* 75 (107) 'k' */ {0x7F, 0x10, 0x28, 0x44, 0x00},
    /* 76 (108) 'l' */ {0x00, 0x41, 0x7F, 0x40, 0x00},
    /* 77 (109) 'm' */ {0x7C, 0x04, 0x18, 0x04, 0x78},
    /* 78 (110) 'n' */ {0x7C, 0x08, 0x04, 0x04, 0x78},
    /* 79 (111) 'o' */ {0x38, 0x44, 0x44, 0x44, 0x38},
    /* 80 (112) 'p' */ {0x7C, 0x14, 0x14, 0x14, 0x08},
    /* 81 (113) 'q' */ {0x08, 0x14, 0x14, 0x18, 0x7C},
    /* 82 (114) 'r' */ {0x7C, 0x08, 0x04, 0x04, 0x08},
    /* 83 (115) 's' */ {0x48, 0x54, 0x54, 0x54, 0x20},
    /* 84 (116) 't' */ {0x04, 0x3F, 0x44, 0x40, 0x20},
    /* 85 (117) 'u' */ {0x3C, 0x40, 0x40, 0x20, 0x7C},
    /* 86 (118) 'v' */ {0x1C, 0x20, 0x40, 0x20, 0x1C},
    /* 87 (119) 'w' */ {0x3C, 0x40, 0x30, 0x40, 0x3C},
    /* 88 (120) 'x' */ {0x44, 0x28, 0x10, 0x28, 0x44},
    /* 89 (121) 'y' */ {0x0C, 0x50, 0x50, 0x50, 0x3C},
    /* 90 (122) 'z' */ {0x44, 0x64, 0x54, 0x4C, 0x44},

    /* 91 (123) '{' */ {0x00, 0x08, 0x36, 0x41, 0x00},
    /* 92 (124) '|' */ {0x00, 0x00, 0x7F, 0x00, 0x00},
    /* 93 (125) '}' */ {0x00, 0x41, 0x36, 0x08, 0x00},
    /* 94 (126) '~' */ {0x08, 0x04, 0x08, 0x10, 0x08},
    /* 95 (127) '⌂' */ {0x7F, 0x7F, 0x7F, 0x7F, 0x7F}  
};

#define FONT_WIDTH  5  /**< Largura da fonte em pixels */
#define FONT_HEIGHT 7  /**< Altura da fonte em pixels */

/**
 * @brief Desenha um caractere utilizando a fonte 5x7.
 *
 * Renderiza um caractere na posição atual do cursor utilizando a fonte definida.
 * Se o caractere for '\n', o cursor é movido para o início da próxima linha.
 * Caso o caractere esteja fora do intervalo imprimível, é substituído por '?'.
 *
 * @param c Caractere a ser desenhado.
 */
void ssd1306_draw_char(char c)
{
    if (c == '\n') {
        cursor_x = 0;
        cursor_y += FONT_HEIGHT + 1;
        return;
    }
    if (c < 32 || c > 127) {
        c = '?';
    }
    uint8_t index = c - 32;
    for (uint8_t col = 0; col < FONT_WIDTH; col++) {
        uint8_t line = font5x7[index][col];
        for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
            if (line & 0x01) {
                ssd1306_draw_pixel(cursor_x + col, cursor_y + row, 1);
            }
            else {
                ssd1306_draw_pixel(cursor_x + col, cursor_y + row, 0);
            }
            line >>= 1;
        }
    }
    /* Espaço entre caracteres */
    for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
        ssd1306_draw_pixel(cursor_x + FONT_WIDTH, cursor_y + row, 0);
    }
    cursor_x += FONT_WIDTH + 1;
    if (cursor_x + FONT_WIDTH >= SSD1306_WIDTH) {
        cursor_x = 0;
        cursor_y += FONT_HEIGHT + 1;
    }
}

/**
 * @brief Exibe uma string de texto no buffer.
 *
 * Percorre a string caractere a caractere e utiliza ::ssd1306_draw_char para
 * renderizar cada um deles na posição atual do cursor.
 *
 * @param str Ponteiro para a string terminada em '\0'.
 */
void ssd1306_draw_text(const char* str)
{
    while (*str) {
        ssd1306_draw_char(*str++);
    }
}

/**
 * @brief Desenha uma linha reta utilizando o algoritmo de Bresenham.
 *
 * Traça uma linha entre os pontos (x0, y0) e (x1, y1) definindo os pixels
 * correspondentes com a cor especificada.
 *
 * @param x0 Coordenada X do ponto inicial.
 * @param y0 Coordenada Y do ponto inicial.
 * @param x1 Coordenada X do ponto final.
 * @param y1 Coordenada Y do ponto final.
 * @param color Cor da linha (1 para pixel ligado, 0 para desligado).
 */
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
    /* Verifica se os pontos estão fora da área útil do display */
    if (x0 >= SSD1306_WIDTH || y0 >= SSD1306_HEIGHT ||
        x1 >= SSD1306_WIDTH || y1 >= SSD1306_HEIGHT) {
        return;
    }

    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        ssd1306_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;

        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief Desenha um bitmap no buffer.
 *
 * Desenha um bitmap monocromático a partir do ponto (x, y). O bitmap é organizado
 * em páginas, onde cada página possui 8 linhas (bits). Cada byte do bitmap representa
 * uma coluna de 8 pixels.
 *
 * @param x Posição X inicial onde o bitmap será desenhado.
 * @param y Posição Y inicial onde o bitmap será desenhado.
 * @param bitmap Ponteiro para os dados do bitmap (1 bit por pixel).
 * @param width Largura do bitmap em pixels.
 * @param height_pixels Altura do bitmap em pixels.
 */
void ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height_pixels) {
    uint8_t pages = height_pixels / 8;  // Cada page possui 8 linhas
    for (uint8_t j = 0; j < pages; j++) {
        for (uint8_t i = 0; i < width; i++) {
            uint8_t byte = bitmap[j * width + i];
            /* Percorre os 8 bits do byte (do bit 7 ao bit 0) */
            for (uint8_t b = 0; b < 8; b++) {
                uint8_t bit = (byte >> (7 - b)) & 0x01;
                ssd1306_draw_pixel(x + i, y + j * 8 + b, bit);
            }
        }
    }
}

/**
 * @brief Desenha um círculo utilizando o algoritmo de Bresenham.
 *
 * Desenha um círculo centrado em (x0, y0) com o raio especificado. São desenhados
 * os pontos correspondentes aos 8 octantes do círculo.
 *
 * @param x0 Coordenada X do centro do círculo.
 * @param y0 Coordenada Y do centro do círculo.
 * @param radius Raio do círculo.
 * @param color Cor do círculo (1 para ligar os pixels, 0 para apagá-los).
 */
void ssd1306_draw_circle(int x0, int y0, int radius, int color) {
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        ssd1306_draw_pixel(x0 + x, y0 + y, color);
        ssd1306_draw_pixel(x0 + y, y0 + x, color);
        ssd1306_draw_pixel(x0 - y, y0 + x, color);
        ssd1306_draw_pixel(x0 - x, y0 + y, color);
        ssd1306_draw_pixel(x0 - x, y0 - y, color);
        ssd1306_draw_pixel(x0 - y, y0 - x, color);
        ssd1306_draw_pixel(x0 + y, y0 - x, color);
        ssd1306_draw_pixel(x0 + x, y0 - y, color);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2 * x + 1;
        }
    }
}
