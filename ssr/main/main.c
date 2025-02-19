#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssr.h"
#ifdef CONFIG_ESP_TASK_WDT
#include "esp_task_wdt.h"
#endif

#define TAG "MAIN"
#define SSR_GPIO GPIO_NUM_17  
// Hook da tarefa idle: reinicia o watchdog para as tarefas ociosas.
void vApplicationIdleHook(void)
{
#ifdef CONFIG_ESP_TASK_WDT
    esp_task_wdt_reset();
#endif
}

void app_main(void)
{
    ssr_t ssr;
    ssr.gpio_pin = SSR_GPIO;

    // Inicializa o SSR
    if (ssr_init(&ssr) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao inicializar o SSR");
        return;
    }

    // --- Teste 1: PWM com 2 Hz e 50% de duty cycle ---
    ESP_LOGI(TAG, "Iniciando PWM: 2 Hz, 50%% duty cycle");
    ssr_setFrequency(&ssr, 2);    // Define frequência de 2 Hz
    ssr_setDutyCycle(&ssr, 50);     // Define duty cycle de 50%

    if (ssr_startPWM(&ssr) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar PWM");
    }

    // Executa o PWM por 10 segundos
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Para o PWM
    ESP_LOGI(TAG, "Parando PWM");
    ssr_stopPWM(&ssr);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // --- Teste 2: Modo ON/OFF ---
    ESP_LOGI(TAG, "Ligando SSR (modo ON/OFF)");
    ssr_on(&ssr);
    vTaskDelay(pdMS_TO_TICKS(3000));  // Mantém ligado por 3 segundos

    ESP_LOGI(TAG, "Desligando SSR (modo ON/OFF)");
    ssr_off(&ssr);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // --- Teste 3: PWM com 5 Hz e 25% de duty cycle ---
    ESP_LOGI(TAG, "Iniciando PWM: 5 Hz, 25%% duty cycle");
    ssr_setFrequency(&ssr, 5);    // Define frequência de 5 Hz
    ssr_setDutyCycle(&ssr, 25);     // Define duty cycle de 25%

    if (ssr_startPWM(&ssr) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar PWM");
    }

    // Executa o PWM por 10 segundos
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Para o PWM novamente
    ESP_LOGI(TAG, "Parando PWM");
    ssr_stopPWM(&ssr);

    ESP_LOGI(TAG, "Teste finalizado");
}
