#include "ssr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#ifdef CONFIG_ESP_TASK_WDT
#include "esp_task_wdt.h"
#else
// Caso o Task Watchdog não esteja habilitado, define macros vazias.
#define esp_task_wdt_reset() do {} while(0)
#define esp_task_wdt_add(task) do {} while(0)
#define esp_task_wdt_delete(task) do {} while(0)
#endif

#define TAG "SSR"

/* Função auxiliar para delay em microssegundos "seguro".
 *
 * Se o delay for >= 1000 µs, converte a parte inteira para milissegundos
 * e utiliza vTaskDelay() para ceder o processador (alimentando o watchdog),
 * e o restante (menos de 1 ms) é realizado com ets_delay_us().
 *
 * Após o vTaskDelay(), reinicia explicitamente o watchdog.
 */
static void delay_us_safe(uint32_t delay_us)
{
    if (delay_us >= 1000) {
        uint32_t delay_ms = delay_us / 1000;
        uint32_t remainder_us = delay_us % 1000;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
#ifdef CONFIG_ESP_TASK_WDT
        esp_task_wdt_reset();
#endif
        if (remainder_us > 0) {
            ets_delay_us(remainder_us);
        }
    } else {
        ets_delay_us(delay_us);
    }
}

/* Inicializa o SSR: configura o pino como saída */
esp_err_t ssr_init(ssr_t *ssr)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    gpio_reset_pin(ssr->gpio_pin);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ssr->gpio_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
        return ret;

    // Estado inicial: desligado
    gpio_set_level(ssr->gpio_pin, 0);

    // Configura os parâmetros padrão
    ssr->mode = SSR_MODE_ONOFF;
    ssr->pwm_frequency = 1;  // Frequência padrão de 1 Hz (pode ser alterada)
    ssr->duty_cycle = 0;
    ssr->pwm_running = false;
    ssr->pwm_task_handle = NULL;

    return ESP_OK;
}

/* Liga o SSR no modo ON-OFF */
esp_err_t ssr_on(ssr_t *ssr)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    if (ssr->pwm_running) {
        ssr_stopPWM(ssr);
    }

    ssr->mode = SSR_MODE_ONOFF;
    gpio_set_level(ssr->gpio_pin, 0);
    return ESP_OK;
}

/* Desliga o SSR no modo ON-OFF */
esp_err_t ssr_off(ssr_t *ssr)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    if (ssr->pwm_running) {
        ssr_stopPWM(ssr);
    }

    ssr->mode = SSR_MODE_ONOFF;
    gpio_set_level(ssr->gpio_pin, 1);
    return ESP_OK;
}

/* Atualiza o duty cycle do PWM */
esp_err_t ssr_setDutyCycle(ssr_t *ssr, uint8_t duty_cycle)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    if (duty_cycle > 100)
        return ESP_ERR_INVALID_ARG;

    ssr->duty_cycle = duty_cycle;
    return ESP_OK;
}

/* Atualiza a frequência do PWM */
esp_err_t ssr_setFrequency(ssr_t *ssr, uint32_t frequency)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    if (frequency == 0)
        return ESP_ERR_INVALID_ARG;

    ssr->pwm_frequency = frequency;
    return ESP_OK;
}

/* Tarefa responsável por gerar o sinal PWM */
static void ssr_pwm_task(void *arg)
{
    ssr_t *ssr = (ssr_t *) arg;
    if (ssr == NULL) {
        vTaskDelete(NULL);
        return;
    }

#ifdef CONFIG_ESP_TASK_WDT
    // Registra a task atual no watchdog
    esp_task_wdt_add(NULL);
#endif

    uint32_t period_us, on_time_us, off_time_us;

    while (ssr->pwm_running) {
#ifdef CONFIG_ESP_TASK_WDT
        // Reinicia o watchdog a cada iteração do loop
        esp_task_wdt_reset();
#endif
        period_us = 1000000 / ssr->pwm_frequency;

        if (ssr->duty_cycle >= 100) {
            // Sinal constantemente ativo
            gpio_set_level(ssr->gpio_pin, 1);
            delay_us_safe(period_us);
        } else if (ssr->duty_cycle == 0) {
            // Sinal constantemente desligado
            gpio_set_level(ssr->gpio_pin, 0);
            delay_us_safe(period_us);
        } else {
            on_time_us = (period_us * ssr->duty_cycle) / 100;
            off_time_us = period_us - on_time_us;

            gpio_set_level(ssr->gpio_pin, 1);
            delay_us_safe(on_time_us);
            gpio_set_level(ssr->gpio_pin, 0);
            delay_us_safe(off_time_us);
        }
    }

#ifdef CONFIG_ESP_TASK_WDT
    // Remove a task do watchdog antes de finalizar
    esp_task_wdt_delete(NULL);
#endif

    // Ao encerrar, zera o handle para indicar que a task foi finalizada
    ssr->pwm_task_handle = NULL;
    vTaskDelete(NULL);
}

/* Inicia o modo PWM criando a task de modulação */
esp_err_t ssr_startPWM(ssr_t *ssr)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    // Se o PWM já está ativo, não faz nada
    if (ssr->pwm_running)
        return ESP_OK;

    ssr->mode = SSR_MODE_PWM;
    ssr->pwm_running = true;

    BaseType_t ret = xTaskCreate(ssr_pwm_task, "ssr_pwm_task", 2048, (void *)ssr, 5, &ssr->pwm_task_handle);
    if (ret != pdPASS) {
        ssr->pwm_running = false;
        ssr->pwm_task_handle = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Interrompe o modo PWM */
esp_err_t ssr_stopPWM(ssr_t *ssr)
{
    if (ssr == NULL)
        return ESP_ERR_INVALID_ARG;

    ssr->pwm_running = false;

    // Aguarda a finalização da task de PWM
    while (ssr->pwm_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ssr->mode = SSR_MODE_ONOFF;
    return ESP_OK;
}
