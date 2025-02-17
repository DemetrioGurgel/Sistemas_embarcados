#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "ds18b20.h"
#include "rom/ets_sys.h"  // Para ets_delay_us

/* Define o pino utilizado para os sensores DS18B20 */
#define DS18B20_GPIO_PIN   GPIO_NUM_16

/* Configurações do I2C para o display SSD1306 */
#define I2C_SCL_PIN         GPIO_NUM_22  // Pino SCL do I2C
#define I2C_SDA_PIN         GPIO_NUM_21  // Pino SDA do I2C
#define I2C_NUM_PORT        I2C_NUM_0    // Porta I2C
#define SSD1306_I2C_ADDRESS 0x3C         // Endereço I2C do display

static bool display_connected = false;
static bool sensor_connected  = true;

/* Função para verificar a conexão I2C com o display */
bool verify_i2c_connection() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

/* Atualiza o display com uma mensagem de erro */
void display_error(const char *msg) {
    ssd1306_clear_buffer();
    ssd1306_set_cursor(0, 0);
    ssd1306_draw_text(msg);
    ssd1306_update();
}

/* Verifica continuamente a conexão com o display */
void check_display_connection() {
    while (!verify_i2c_connection()) {
        if (display_connected) {
            ESP_LOGE("MAIN", "Erro: Display desconectado ou não respondendo! Aguardando reconexão...");
            display_connected = false;
        }
        display_error("Display\n desconectado!");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (!display_connected) {
        ESP_LOGI("MAIN", "Display reconectado! Continuando execução...");
        display_connected = true;
    }
}

/* Pré-verificação do sensor DS18B20 */
bool precheck_sensor_connection(gpio_num_t gpio_pin) {
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_pin, 0);
    ets_delay_us(480);

    gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
    ets_delay_us(70);
    bool presence = !gpio_get_level(gpio_pin);
    ets_delay_us(480 - 70);

    return presence;
}

/*
 * Verifica se o sensor está conectado realizando uma conversão e leitura.
 * Se o sensor retornar aproximadamente 85°C ou 0°C, é considerado com problema.
 */
bool sensor_is_connected(ds18b20_t *sensor) {
    float temp;
    if (ds18b20_requestTemperatures(sensor) != ESP_OK)
        return false;
    if (ds18b20_getTempC(sensor, &temp) != ESP_OK)
        return false;
    if ((temp > 84.9f && temp < 85.1f) || (temp > -0.1f && temp < 0.1f))
        return false;
    return true;
}

#define MAX_SENSORS 10

void app_main(void)
{
    /* Inicializa o display SSD1306 */
    ssd1306_config_t disp_config = {
        .bus = SSD1306_BUS_I2C,
        .iface.i2c = {
            .sda_pin   = I2C_SDA_PIN,
            .scl_pin   = I2C_SCL_PIN,
            .clk_speed = 400000,
            .port      = I2C_NUM_PORT,
            .address   = SSD1306_I2C_ADDRESS,
        },
    };

    if (ssd1306_init(&disp_config) != ESP_OK) {
        ESP_LOGE("MAIN", "Erro ao iniciar o display SSD1306");
        return;
    }
    display_connected = true;

    /* Busca múltiplos sensores DS18B20 no pino definido */
    ds18b20_t sensors[MAX_SENSORS];
    int sensorCount = 0;
    while (ds18b20_searchSensors(DS18B20_GPIO_PIN, sensors, MAX_SENSORS, &sensorCount) != ESP_OK || sensorCount == 0) {
        ESP_LOGE("MAIN", "Nenhum sensor DS18B20 encontrado no pino definido!");
        display_error("Sensor DS18B20\nnão encontrado!");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI("MAIN", "Foram encontrados %d sensor(es) DS18B20", sensorCount);

    /* Inicializa cada sensor encontrado e marca como conectado */
    bool sensorConnected[MAX_SENSORS] = { false };
    for (int i = 0; i < sensorCount; i++) {
        bool init_success = false;
        bool error_printed = false;
        while (!init_success) {
            if (!precheck_sensor_connection(DS18B20_GPIO_PIN) ||
                ds18b20_begin(&sensors[i]) != ESP_OK ||
                ds18b20_setResolution(&sensors[i], 12) != ESP_OK) {
                if (!error_printed) {
                    ESP_LOGE("MAIN", "Falha na inicialização do sensor %d DS18B20! Aguardando...", i);
                    display_error("Falha DS18B20!\nTentando...");
                    error_printed = true;
                }
                vTaskDelay(pdMS_TO_TICKS(5000));
            } else {
                init_success = true;
            }
        }
        sensorConnected[i] = true;
    }
    ESP_LOGI("MAIN", "Todos os sensores DS18B20 inicializados com sucesso!");
    display_error("Sensores\nconectados!");

    /* Loop principal de operação */
    while (1) {
        check_display_connection();

        for (int i = 0; i < sensorCount; i++) {
            float temperature = 0.0f;

            if (sensorConnected[i]) {
                // Tenta ler a temperatura do sensor se ele estiver marcado como conectado
                if (sensor_is_connected(&sensors[i])) {
                    if (ds18b20_requestTemperatures(&sensors[i]) == ESP_OK &&
                        ds18b20_getTempC(&sensors[i], &temperature) == ESP_OK) {
                        char rom_str[17];
                        for (int j = 0; j < 8; j++) {
                            sprintf(&rom_str[j * 2], "%02X", sensors[i].rom_code[j]);
                        }
                        rom_str[16] = '\0';
                        ESP_LOGI("MAIN", "Sensor %d (ROM: %s) -> Temp: %.2f °C", i, rom_str, temperature);

                        // Atualiza o display com a temperatura do sensor 0 (ou outro escolhido)
                        if (i == 0) {
                            ssd1306_clear_buffer();
                            ssd1306_set_cursor(0, 0);
                            char temp_str[32];
                            snprintf(temp_str, sizeof(temp_str), "Temp: %.2f C", temperature);
                            ssd1306_draw_text(temp_str);
                            ssd1306_update();
                        }
                    } else {
                        // Falha na leitura, marca sensor como desconectado
                        sensorConnected[i] = false;
                        ESP_LOGE("MAIN", "Falha na leitura do sensor %d!", i);
                        if (i == 0) {
                            display_error("Sensor 0\ndesconectado!");
                        }
                    }
                } else {
                    // Se o sensor não responder, marca como desconectado
                    sensorConnected[i] = false;
                    ESP_LOGE("MAIN", "Sensor %d desconectado ou com problema!", i);
                    if (i == 0) {
                        display_error("Sensor 0\ndesconectado!");
                    }
                }
            } else {
                // Sensor marcado como desconectado: tenta reconectar sem bloquear os demais
                if (sensor_is_connected(&sensors[i])) {
                    sensorConnected[i] = true;
                    ESP_LOGI("MAIN", "Sensor %d reconectado!", i);
                } else {
                    ESP_LOGW("MAIN", "Sensor %d continua desconectado.", i);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
