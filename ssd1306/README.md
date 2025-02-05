# Biblioteca SSD1306

Esta biblioteca fornece uma interface para comunicação com displays OLED SSD1306 via I2C ou SPI no ESP32. Com ela, é possível inicializar o display, exibir textos, desenhar formas geométricas e atualizar a tela de maneira eficiente.

## Funcionalidades
- Inicializar o display SSD1306 utilizando I2C ou SPI.
- Atualizar a tela com o conteúdo do buffer interno.
- Exibir texto com fonte 5x7.
- Desenhar pixels, linhas, círculos e bitmaps.
- Limpar ou preencher completamente a tela.

## Dependências
A biblioteca utiliza os seguintes componentes do ESP-IDF:
- `esp_err.h`: Para tratamento de erros.
- `driver/i2c.h`: Para comunicação via I2C.
- `driver/spi_master.h`: Para comunicação via SPI.
- `driver/gpio.h`: Para controle de pinos GPIO.

---

## Como Usar a Biblioteca

### Passo 1: Configurar o Display SSD1306

Antes de inicializar o display, configure a interface desejada (I2C ou SPI).

#### Configuração via I2C:

```c
#include "ssd1306.h"

ssd1306_config_t oled_config = {
    .bus = SSD1306_BUS_I2C,
    .iface.i2c = {
        .port = I2C_NUM_0,
        .sda_pin = 21,
        .scl_pin = 22,
        .clk_speed = 400000,
        .address = 0x3C
    }
};
```

#### Configuração via SPI:

```c
#include "ssd1306.h"

ssd1306_config_t oled_config = {
    .bus = SSD1306_BUS_SPI,
    .iface.spi = {
        .host = SPI2_HOST,
        .cs_pin = 5,
        .dc_pin = 4,
        .mosi_pin = 23,
        .sclk_pin = 18,
        .rst_pin = 2,
        .dma_chan = 1,
        .clock_speed_hz = 10000000
    }
};
```

### Passo 2: Inicializar o Display

Após configurar a estrutura, chame `ssd1306_init()` para inicializar o display:

```c
if (ssd1306_init(&oled_config) == ESP_OK) {
    printf("SSD1306 inicializado com sucesso.\n");
} else {
    printf("Erro ao inicializar o SSD1306.\n");
}
```

---

## Funções Disponíveis

### `ssd1306_init`
Inicializa o display utilizando a configuração fornecida.

```c
esp_err_t ssd1306_init(const ssd1306_config_t *config);
```

**Parâmetros:**
- `config`: Ponteiro para a estrutura de configuração do display.

**Retorno:**
- `ESP_OK` se a inicialização for bem-sucedida.
- Código de erro caso contrário.

---

### `ssd1306_set_cursor`
Define a posição do cursor para escrita de texto.

```c
void ssd1306_set_cursor(uint8_t x, uint8_t y);
```

**Parâmetros:**
- `x`: Posição horizontal do cursor (0 a 127).
- `y`: Posição vertical do cursor (0 a 63).

---

### `ssd1306_draw_text`
Exibe uma string de texto no display.

```c
void ssd1306_draw_text(const char* str);
```

**Parâmetros:**
- `str`: String terminada em `\0` a ser exibida no display.

---

### `ssd1306_draw_pixel`
Desenha um único pixel no buffer do display.

```c
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);
```

**Parâmetros:**
- `x`: Coordenada X do pixel.
- `y`: Coordenada Y do pixel.
- `color`: 1 para pixel ligado, 0 para pixel desligado.

---

### `ssd1306_draw_line`
Desenha uma linha utilizando o algoritmo de Bresenham.

```c
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
```

**Parâmetros:**
- `x0`, `y0`: Coordenadas do ponto inicial.
- `x1`, `y1`: Coordenadas do ponto final.
- `color`: 1 para linha visível, 0 para apagada.

---

### `ssd1306_draw_circle`
Desenha um círculo.

```c
void ssd1306_draw_circle(int x0, int y0, int radius, int color);
```

**Parâmetros:**
- `x0`, `y0`: Coordenadas do centro do círculo.
- `radius`: Raio do círculo.
- `color`: 1 para preenchido, 0 para apagado.

---

### `ssd1306_draw_bitmap`
Desenha um bitmap na tela.

```c
void ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);
```

**Parâmetros:**
- `x`, `y`: Posição inicial do bitmap.
- `bitmap`: Ponteiro para os dados do bitmap.
- `width`, `height`: Largura e altura do bitmap.

---

### `ssd1306_clear_buffer`
Limpa o buffer do display, apagando todos os pixels.

```c
void ssd1306_clear_buffer(void);
```

---

### `ssd1306_fill_buffer`
Preenche o buffer do display com todos os pixels ligados.

```c
void ssd1306_fill_buffer(void);
```

---

### `ssd1306_update`
Atualiza a tela do display com o conteúdo do buffer.

```c
esp_err_t ssd1306_update(void);
```

**Retorno:**
- `ESP_OK` se a atualização for bem-sucedida.
- Código de erro caso contrário.

---

## Definições Importantes

### Endereço I2C
O endereço padrão do SSD1306 é `0x3C`.

```c
#define SSD1306_I2C_ADDRESS 0x3C
```

### Dimensões do Display
A biblioteca foi projetada para displays de **128x64 pixels**.

```c
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
```

---

