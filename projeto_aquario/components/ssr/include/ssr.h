#ifndef SSR_H
#define SSR_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Modos de operação do SSR
typedef enum {
    SSR_MODE_ONOFF = 0,
    SSR_MODE_PWM
} ssr_mode_t;

// Estrutura de configuração do SSR
typedef struct {
    gpio_num_t gpio_pin;         // Pino utilizado para controle do SSR
    ssr_mode_t mode;             // Modo de operação: ON-OFF ou PWM
    uint32_t pwm_frequency;      // Frequência do PWM (Hz)
    uint8_t duty_cycle;          // Ciclo de trabalho do PWM (0-100 %)
    TaskHandle_t pwm_task_handle; // Handle da task de PWM (se estiver em uso)
    bool pwm_running;            // Flag que indica se o PWM está ativo
} ssr_t;

/**
 * @brief Inicializa o SSR configurando o pino como saída.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ssr_init(ssr_t *ssr);

/**
 * @brief Aciona o SSR no modo ON-OFF (liga o relé).
 *
 * Caso o PWM esteja ativo, ele será interrompido.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ssr_on(ssr_t *ssr);

/**
 * @brief Desliga o SSR no modo ON-OFF.
 *
 * Caso o PWM esteja ativo, ele será interrompido.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ssr_off(ssr_t *ssr);

/**
 * @brief Define o ciclo de trabalho (duty cycle) do PWM.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @param duty_cycle Valor de 0 a 100.
 * @return esp_err_t ESP_OK se bem-sucedido ou ESP_ERR_INVALID_ARG para valor inválido.
 */
esp_err_t ssr_setDutyCycle(ssr_t *ssr, uint8_t duty_cycle);

/**
 * @brief Define a frequência do PWM.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @param frequency Frequência em Hz (não pode ser zero).
 * @return esp_err_t ESP_OK se bem-sucedido ou ESP_ERR_INVALID_ARG para valor inválido.
 */
esp_err_t ssr_setFrequency(ssr_t *ssr, uint32_t frequency);

/**
 * @brief Inicia o modo PWM.
 *
 * Cria uma task dedicada para gerar o sinal PWM de acordo com os parâmetros.
 * Essa implementação utiliza um delay seguro (delay_us_safe) que evita bloqueios
 * longos e previne o disparo do watchdog.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ssr_startPWM(ssr_t *ssr);

/**
 * @brief Interrompe o modo PWM.
 *
 * @param ssr Ponteiro para a estrutura de configuração do SSR.
 * @return esp_err_t ESP_OK em caso de sucesso.
 */
esp_err_t ssr_stopPWM(ssr_t *ssr);

#ifdef __cplusplus
}
#endif

#endif // SSR_H
