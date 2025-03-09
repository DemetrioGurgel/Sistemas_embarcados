#ifndef DS18B20_H
#define DS18B20_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Comandos do DS18B20 */
#define DS18B20_CMD_SKIP_ROM         0xCC  // Não usaremos para comunicação individual
#define DS18B20_CMD_MATCH_ROM        0x55  // Para selecionar sensor específico
#define DS18B20_CMD_SEARCH_ROM       0xF0  // Para buscar sensores no barramento
#define DS18B20_CMD_CONVERT_T        0x44  // Inicia a conversão de temperatura
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE  // Lê os dados do sensor
#define DS18B20_CMD_WRITE_SCRATCHPAD 0x4E  // Escreve configuração (por exemplo, resolução)

/* Valores para configuração de resolução */
#define DS18B20_RESOLUTION_9BIT  0x1F
#define DS18B20_RESOLUTION_10BIT 0x3F
#define DS18B20_RESOLUTION_11BIT 0x5F
#define DS18B20_RESOLUTION_12BIT 0x7F

/**
 * @brief Estrutura de configuração do DS18B20.
 *
 * - gpio_pin: pino GPIO utilizado para comunicação 1-Wire.
 * - resolution: resolução desejada (9, 10, 11 ou 12 bits).
 * - rom_code: endereço único (64 bits) do sensor.
 */
typedef struct {
    gpio_num_t gpio_pin;
    uint8_t resolution;  // em bits
    uint8_t rom_code[8]; // Endereço único do sensor
} ds18b20_t;

/**
 * @brief Inicializa o sensor DS18B20.
 *
 * Configura o pino como open-drain com pull-up e libera o barramento.
 *
 * @param sensor Ponteiro para a estrutura de configuração.
 * @return esp_err_t ESP_OK em caso de sucesso ou erro na configuração do GPIO.
 */
esp_err_t ds18b20_begin(ds18b20_t *sensor);

/**
 * @brief Define a resolução do DS18B20.
 *
 * Esta função envia o comando WRITE_SCRATCHPAD para configurar a resolução.
 *
 * @param sensor Ponteiro para a estrutura de configuração.
 * @param resolution Resolução desejada: 9, 10, 11 ou 12 bits.
 * @return esp_err_t ESP_OK em caso de sucesso ou erro se o argumento for inválido.
 */
esp_err_t ds18b20_setResolution(ds18b20_t *sensor, uint8_t resolution);

/**
 * @brief Solicita a conversão de temperatura.
 *
 * Envia os comandos necessários (MATCH ROM + CONVERT_T) e aguarda o tempo
 * de conversão adequado conforme a resolução.
 *
 * @param sensor Ponteiro para a estrutura de configuração.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ds18b20_requestTemperatures(ds18b20_t *sensor);

/**
 * @brief Lê a temperatura em graus Celsius.
 *
 * Realiza o reset do barramento, envia os comandos para leitura (MATCH ROM +
 * READ_SCRATCHPAD), lê os 9 bytes do scratchpad e converte o valor lido.
 *
 * @param sensor Ponteiro para a estrutura de configuração.
 * @param temperature Ponteiro onde será armazenada a temperatura.
 * @return esp_err_t ESP_OK se a leitura for realizada com sucesso.
 */
esp_err_t ds18b20_getTempC(ds18b20_t *sensor, float *temperature);

/**
 * @brief Busca por sensores DS18B20 no barramento 1-Wire.
 *
 * Varre o barramento conectado ao pino especificado e preenche o array de sensores
 * com os endereços únicos encontrados.
 *
 * @param gpio_pin Pino utilizado para a comunicação 1-Wire.
 * @param sensors Array de estruturas ds18b20_t para armazenar os sensores encontrados.
 * @param maxSensors Número máximo de sensores que podem ser armazenados.
 * @param foundSensors Ponteiro para variável que receberá o número de sensores encontrados.
 * @return esp_err_t ESP_OK se ao menos um sensor for encontrado, ESP_FAIL caso contrário.
 */
esp_err_t ds18b20_searchSensors(gpio_num_t gpio_pin, ds18b20_t *sensors, int maxSensors, int *foundSensors);

#ifdef __cplusplus
}
#endif

#endif // DS18B20_H
