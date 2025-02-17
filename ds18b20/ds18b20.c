#include "ds18b20.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <string.h>

#define TAG "DS18B20"

/* Definições de timing (em microssegundos) conforme o datasheet do DS18B20 */
#define DS18B20_RESET_PULSE_US    480
#define DS18B20_PRESENCE_WAIT_US   70
#define DS18B20_SLOT_US           60

/**
 * Funções auxiliares que operam diretamente sobre o pino, sem precisar da estrutura.
 */

static esp_err_t oneWireResetOnPin(gpio_num_t gpio_pin) {
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_pin, 0);
    ets_delay_us(DS18B20_RESET_PULSE_US);
    gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
    ets_delay_us(DS18B20_PRESENCE_WAIT_US);
    bool presence = !gpio_get_level(gpio_pin);
    ets_delay_us(DS18B20_RESET_PULSE_US - DS18B20_PRESENCE_WAIT_US);
    return presence ? ESP_OK : ESP_FAIL;
}

static void oneWireWriteBitOnPin(gpio_num_t gpio_pin, int bit) {
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_pin, 0);
    if (bit) {
        ets_delay_us(5);
        gpio_set_level(gpio_pin, 1);
        ets_delay_us(DS18B20_SLOT_US - 5);
    } else {
        ets_delay_us(60);
        gpio_set_level(gpio_pin, 1);
        ets_delay_us(DS18B20_SLOT_US - 60);
    }
}

static int oneWireReadBitOnPin(gpio_num_t gpio_pin) {
    int bit;
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_pin, 0);
    ets_delay_us(2);
    gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
    ets_delay_us(10);
    bit = gpio_get_level(gpio_pin);
    ets_delay_us(DS18B20_SLOT_US - 12);
    return bit;
}

static void oneWireWriteByteOnPin(gpio_num_t gpio_pin, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        oneWireWriteBitOnPin(gpio_pin, byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t oneWireReadByteOnPin(gpio_num_t gpio_pin) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (oneWireReadBitOnPin(gpio_pin) << i);
    }
    return byte;
}

/**
 * Wrappers para as funções OneWire que utilizam a estrutura ds18b20_t.
 */
static esp_err_t oneWireReset(ds18b20_t *sensor) {
    return oneWireResetOnPin(sensor->gpio_pin);
}

static void oneWireWriteBit(ds18b20_t *sensor, int bit) {
    oneWireWriteBitOnPin(sensor->gpio_pin, bit);
}

static void oneWireWriteByte(ds18b20_t *sensor, uint8_t byte) {
    oneWireWriteByteOnPin(sensor->gpio_pin, byte);
}

static uint8_t oneWireReadByte(ds18b20_t *sensor) {
    return oneWireReadByteOnPin(sensor->gpio_pin);
}

/**
 * Implementação simplificada do algoritmo OneWire Search.
 * Essa função preenche newAddr com o endereço encontrado.
 */
static bool oneWireSearch(uint8_t *newAddr, gpio_num_t gpio_pin, uint8_t *lastDiscrepancy, bool *lastDevice) {
    uint8_t id_bit, cmp_id_bit, search_direction;
    uint8_t last_zero = 0;
    int rom_byte_number = 0;
    uint8_t rom_byte_mask = 1;
    int id_bit_number = 1; // Contador de bits de 1 a 64

    // Se o último dispositivo já foi encontrado, não há mais nada a fazer
    if (*lastDevice) {
        *lastDiscrepancy = 0;
        return false;
    }

    // Inicia a comunicação com um reset
    if (oneWireResetOnPin(gpio_pin) != ESP_OK) {
        *lastDiscrepancy = 0;
        *lastDevice = true;
        return false;
    }

    // Envia o comando SEARCH_ROM
    oneWireWriteByteOnPin(gpio_pin, DS18B20_CMD_SEARCH_ROM);

    // Processa os 64 bits do endereço
    for (id_bit_number = 1; id_bit_number <= 64; id_bit_number++) {
        id_bit = oneWireReadBitOnPin(gpio_pin);
        cmp_id_bit = oneWireReadBitOnPin(gpio_pin);

        // Se ambos os bits forem 1, não há sensores ativos
        if (id_bit && cmp_id_bit) {
            break;
        }

        if (id_bit != cmp_id_bit) {
            // Sem discrepância, segue o valor lido
            search_direction = id_bit;
        } else {
            // Há discrepância: ambos os bits são 0
            if (id_bit_number < *lastDiscrepancy) {
                // Usa o mesmo caminho da busca anterior
                search_direction = ((newAddr[rom_byte_number] & rom_byte_mask) != 0) ? 1 : 0;
            } else if (id_bit_number == *lastDiscrepancy) {
                // No ponto da última divergência, forçamos a escolha para 1
                search_direction = 1;
            } else {
                // Para os bits após a última divergência, escolhe 0 por padrão
                search_direction = 0;
            }
            // Atualiza o último bit com discrepância, se for o caso
            if (search_direction == 0) {
                last_zero = id_bit_number;
            }
        }

        // Grava o bit escolhido na posição correta do endereço
        if (search_direction == 1)
            newAddr[rom_byte_number] |= rom_byte_mask;
        else
            newAddr[rom_byte_number] &= ~rom_byte_mask;

        // Escreve o bit escolhido no barramento
        oneWireWriteBitOnPin(gpio_pin, search_direction);

        // Avança para o próximo bit
        rom_byte_mask <<= 1;
        if (rom_byte_mask == 0) {
            rom_byte_number++;
            rom_byte_mask = 1;
        }
    }

    // Atualiza o estado da busca
    *lastDiscrepancy = last_zero;
    if (*lastDiscrepancy == 0)
        *lastDevice = true;

    // Valida se o endereço encontrado não é nulo
    bool valid = false;
    for (int i = 0; i < 8; i++) {
        if (newAddr[i] != 0) {
            valid = true;
            break;
        }
    }
    return valid;
}

/* --- Função pública para buscar sensores no barramento --- */
esp_err_t ds18b20_searchSensors(gpio_num_t gpio_pin, ds18b20_t *sensors, int maxSensors, int *foundSensors) {
    uint8_t lastDiscrepancy = 0;
    bool lastDevice = false;
    int sensorCount = 0;
    uint8_t rom[8] = {0};

    lastDiscrepancy = 0;
    lastDevice = false;

    while (!lastDevice && sensorCount < maxSensors) {
        // Zera o buffer de ROM antes de cada busca
        memset(rom, 0, sizeof(rom));
        if (!oneWireSearch(rom, gpio_pin, &lastDiscrepancy, &lastDevice)) {
            break;
        }
        // Armazena o endereço único encontrado
        memcpy(sensors[sensorCount].rom_code, rom, 8);
        sensors[sensorCount].gpio_pin = gpio_pin;
        sensors[sensorCount].resolution = 12; // resolução padrão
        ESP_LOGI(TAG, "Sensor %d encontrado -> ROM: %02X%02X%02X%02X%02X%02X%02X%02X", sensorCount,
                 rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
        sensorCount++;
    }
    *foundSensors = sensorCount;
    return sensorCount > 0 ? ESP_OK : ESP_FAIL;
}

/* --- Funções modificadas para usar MATCH ROM (0x55) --- */

esp_err_t ds18b20_begin(ds18b20_t *sensor) {
    gpio_reset_pin(sensor->gpio_pin);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sensor->gpio_pin),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao configurar o GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(sensor->gpio_pin, 1);
    return ESP_OK;
}

esp_err_t ds18b20_setResolution(ds18b20_t *sensor, uint8_t resolution) {
    uint8_t configValue;
    switch (resolution) {
        case 9:
            configValue = DS18B20_RESOLUTION_9BIT;
            break;
        case 10:
            configValue = DS18B20_RESOLUTION_10BIT;
            break;
        case 11:
            configValue = DS18B20_RESOLUTION_11BIT;
            break;
        case 12:
            configValue = DS18B20_RESOLUTION_12BIT;
            break;
        default:
            ESP_LOGE(TAG, "Resolução inválida: %d", resolution);
            return ESP_ERR_INVALID_ARG;
    }

    if (oneWireReset(sensor) != ESP_OK) return ESP_FAIL;
    oneWireWriteByte(sensor, DS18B20_CMD_MATCH_ROM);
    for (int i = 0; i < 8; i++) {
        oneWireWriteByte(sensor, sensor->rom_code[i]);
    }
    oneWireWriteByte(sensor, DS18B20_CMD_WRITE_SCRATCHPAD);
    oneWireWriteByte(sensor, 0);           // TH (não utilizado)
    oneWireWriteByte(sensor, 0);           // TL (não utilizado)
    oneWireWriteByte(sensor, configValue);

    sensor->resolution = resolution;
    return ESP_OK;
}

esp_err_t ds18b20_requestTemperatures(ds18b20_t *sensor) {
    if (oneWireReset(sensor) != ESP_OK) return ESP_FAIL;
    oneWireWriteByte(sensor, DS18B20_CMD_MATCH_ROM);
    for (int i = 0; i < 8; i++) {
        oneWireWriteByte(sensor, sensor->rom_code[i]);
    }
    oneWireWriteByte(sensor, DS18B20_CMD_CONVERT_T);

    uint32_t delay_ms;
    switch (sensor->resolution) {
        case 9:  delay_ms = 100; break;
        case 10: delay_ms = 200; break;
        case 11: delay_ms = 375; break;
        case 12: delay_ms = 750; break;
        default: delay_ms = 750; break;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    return ESP_OK;
}

esp_err_t ds18b20_getTempC(ds18b20_t *sensor, float *temperature) {
    uint8_t data[9];

    if (oneWireReset(sensor) != ESP_OK) return ESP_FAIL;
    oneWireWriteByte(sensor, DS18B20_CMD_MATCH_ROM);
    for (int i = 0; i < 8; i++) {
        oneWireWriteByte(sensor, sensor->rom_code[i]);
    }
    oneWireWriteByte(sensor, DS18B20_CMD_READ_SCRATCHPAD);

    for (int i = 0; i < 9; i++) {
        data[i] = oneWireReadByte(sensor);
    }

    int16_t raw = (data[1] << 8) | data[0];
    *temperature = raw / 16.0f;
    return ESP_OK;
}
