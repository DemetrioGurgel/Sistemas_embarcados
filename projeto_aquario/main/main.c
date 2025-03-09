#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "ds18b20.h"
#include "ssr.h"
#include "rom/ets_sys.h"  // Para ets_delay_us

#define I2C_MASTER_SCL_IO 22   // Pino SCL
#define I2C_MASTER_SDA_IO 21   // Pino SDA
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define DS18B20_GPIO_PIN GPIO_NUM_16  // Pino do sensor DS18B20
#define SSR_GPIO GPIO_NUM_17          // Pino do cooler
#define SSR_PUMP_GPIO GPIO_NUM_18     // Pino do rele da bomba
#define BUZZER_GPIO GPIO_NUM_14        // Pino do buzzer

static const char *TAG = "SISTEMA";    // Tag utilizada para identificação nas mensagens de log

// Declaração de variáveis estáticas para os dispositivos/sensores
static ds18b20_t temp_sensor;          // Estrutura que representa o sensor de temperatura DS18B20
static ssr_t ssr;                    // Estrutura para controle do SSR do cooler
static ssr_t pump_relay;             // Estrutura para controle do SSR da bomba

// Função para converter a aceleração no eixo Y em litros (usando um ajuste cúbico)
// Essa função utiliza uma equação do tipo ax^3 + bx^2 + cx + d para realizar a conversão.
float calcular_nivel_agua(float accel_y) {
    float a = -40.871;
    float b = 58.647;
    float c = -30.786;
    float d = 6.958;
    
    // Calcula o valor em litros a partir da aceleração usando a equação cúbica
    float litros = (a * pow(accel_y, 3)) + (b * pow(accel_y, 2)) + (c * accel_y) + d;
    
    if (litros < 0) litros = 0;  // Garante que o valor de litros não seja negativo
    return litros;
}

// Função para atualizar o display OLED com informações de nível de água e temperatura
void atualizar_display(float nivel_agua, float temperatura) {
    char buffer[50];
    // Se a temperatura estiver acima de 30°C, exibe uma mensagem de alerta no display
    if (temperatura >= 30.0) {
        snprintf(buffer, sizeof(buffer), "ALERTA!!!\nNivel: %.2fL\nTemp: %.2fC", nivel_agua, temperatura);
    } else {
        snprintf(buffer, sizeof(buffer), "Nivel: %.2fL\nTemp: %.2fC", nivel_agua, temperatura);
    }
    
    // Atualiza o conteúdo do display OLED
    ssd1306_clear_buffer();          // Limpa o buffer do display
    ssd1306_set_cursor(0, 0);          // Posiciona o cursor no início (linha 0, coluna 0)
    ssd1306_draw_text(buffer);         // Desenha o texto formatado no buffer
    ssd1306_update();                  // Atualiza o display com o conteúdo do buffer
}

// Função para emitir beeps com o buzzer
void beep_buzzer() {
    for (int i = 0; i < 3; i++) { // Executa três ciclos de beep
        gpio_set_level(BUZZER_GPIO, 1);                // Liga o buzzer (nível alto)
        vTaskDelay(pdMS_TO_TICKS(500));                // Aguarda 500 milissegundos com o buzzer ligado
        gpio_set_level(BUZZER_GPIO, 0);                // Desliga o buzzer (nível baixo)
        vTaskDelay(pdMS_TO_TICKS(500));                // Aguarda 500 milissegundos antes do próximo beep
    }
}

// Tarefa responsável por ler os sensores e controlar os dispositivos (cooler e bomba)
void sensor_task(void *pvParameter) {
    mpu6050_data_t sensor_data;        // Estrutura para armazenar os dados do MPU6050
    float temperatura = 0.0f;          // Variável para armazenar a temperatura lida pelo DS18B20
    int bomba_ligada = 0;              // Variável para controlar o estado da bomba: 0 = desligada, 1 = ligada

    while (1) {
        // Realiza a leitura completa do MPU6050 e armazena os dados em sensor_data
        if (mpu6050_read_all(I2C_MASTER_NUM, &sensor_data) == ESP_OK) {
            float pitch, roll;
            // Calcula os ângulos de Euler (pitch e roll) a partir dos dados do acelerômetro
            mpu6050_compute_euler_angles_from_accel(sensor_data.accel_x, 
                                                    sensor_data.accel_y, 
                                                    sensor_data.accel_z, 
                                                    &pitch, &roll);
            // Converte o valor de aceleração do eixo Y em nível de água (litros)
            float nivel_agua = calcular_nivel_agua(sensor_data.accel_y);
            
            // Solicita a leitura da temperatura do sensor DS18B20
            if (ds18b20_requestTemperatures(&temp_sensor) == ESP_OK &&
                ds18b20_getTempC(&temp_sensor, &temperatura) == ESP_OK) {
                ESP_LOGI(TAG, "Temperatura: %.2f C", temperatura);
            } else {
                ESP_LOGE(TAG, "Erro na leitura do DS18B20");
                temperatura = -1; // Define temperatura negativa para indicar erro na leitura
            }
            
            // Atualiza o display OLED com as informações de nível de água e temperatura
            atualizar_display(nivel_agua, temperatura);

            // Controle do cooler com base na temperatura lida
            if (temperatura >= 30.0) {
                ESP_LOGI(TAG, "Temperatura alta! Ligando cooler e ativando alarme...");
                ssr_off(&ssr);   // Liga o cooler (nível baixo ativa o dispositivo)
                beep_buzzer();   // Emite beeps para alarme
            } else {
                ESP_LOGI(TAG, "Temperatura normal. Desligando cooler...");
                ssr_on(&ssr);    // Desliga o cooler (nível alto desativa o dispositivo)
            }
            
            // Controle da bomba com base no nível de água
            if (nivel_agua < 0.50) {
                // Se o nível estiver abaixo de 0.50 litros e a bomba ainda estiver desligada
                if (!bomba_ligada) {
                    ESP_LOGI(TAG, "Nivel abaixo de 0.50L. Ligando bomba...");
                    ssr_off(&pump_relay); // Liga a bomba (nível baixo ativa o dispositivo)
                    bomba_ligada = 1;     // Atualiza o estado da bomba para ligada
                }
            } else if (nivel_agua >= 1.0) {
                // Se o nível atingir ou ultrapassar 1 litro e a bomba estiver ligada
                if (bomba_ligada) {
                    ESP_LOGI(TAG, "Nivel atingiu 1L. Desligando bomba...");
                    ssr_on(&pump_relay); // Desliga a bomba (nível alto desativa o dispositivo)
                    bomba_ligada = 0;    // Atualiza o estado da bomba para desligada
                }
            }
            
            // Loga as informações de nível de água e temperatura
            ESP_LOGI(TAG, "Nivel de Agua: %.2f L, Temperatura: %.2f C", nivel_agua, temperatura);
        } else {
            ESP_LOGE(TAG, "Erro ao ler o MPU6050");
        }
        
        // Aguarda 5 segundos antes de repetir a leitura dos sensores
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Função principal da aplicação
void app_main() {
    // Configuração e inicialização do display OLED SSD1306
    ssd1306_config_t config = {
        .bus = SSD1306_BUS_I2C,      // Define o barramento utilizado (I2C)
        .iface.i2c = {
            .port = I2C_MASTER_NUM,  // Número do controlador I2C
            .sda_pin = I2C_MASTER_SDA_IO,  // Pino SDA para comunicação I2C
            .scl_pin = I2C_MASTER_SCL_IO,  // Pino SCL para comunicação I2C
            .clk_speed = I2C_MASTER_FREQ_HZ,  // Velocidade do clock I2C
            .address = 0x3C,         // Endereço I2C do display OLED
        },
    };

    // Inicializa o display OLED; se houver erro, registra o erro e interrompe a execução
    if (ssd1306_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao inicializar o SSD1306");
        return;
    }
    
    // Exibe mensagem de inicialização no display
    ssd1306_clear_buffer();
    ssd1306_set_cursor(0, 0);
    ssd1306_draw_text("Sistema Iniciado!");
    ssd1306_update();

    // Inicializa o sensor MPU6050; se houver erro, registra e encerra
    if (mpu6050_init(I2C_MASTER_NUM) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o MPU6050");
        return;
    }
    ESP_LOGI(TAG, "MPU6050 inicializado com sucesso!");

    // Configuração e inicialização do SSR para controle do cooler
    ssr.gpio_pin = SSR_GPIO;
    if (ssr_init(&ssr) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao inicializar o SSR do cooler");
        return;
    }
    ssr_on(&ssr); // Inicializa com nível lógico alto (cooler desligado)

    // Configuração e inicialização do SSR para controle da bomba
    pump_relay.gpio_pin = SSR_PUMP_GPIO;
    if (ssr_init(&pump_relay) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao inicializar o SSR da bomba");
        return;
    }
    ssr_on(&pump_relay); // Inicializa com nível lógico alto (bomba desligada)

    // Configura o pino do buzzer como saída e inicia desligado
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0); // Garante que o buzzer comece desligado

    // Busca sensores DS18B20 conectados no pino definido
    int sensor_count = 0;
    ds18b20_t sensores[1]; // Espera encontrar apenas um sensor
    if (ds18b20_searchSensors(DS18B20_GPIO_PIN, sensores, 1, &sensor_count) != ESP_OK || sensor_count == 0) {
        ESP_LOGE(TAG, "Nenhum sensor DS18B20 encontrado");
        return;
    }
    // Seleciona o primeiro sensor encontrado
    temp_sensor = sensores[0];

    // Inicializa o sensor DS18B20 e configura sua resolução para 12 bits
    if (ds18b20_begin(&temp_sensor) != ESP_OK ||
        ds18b20_setResolution(&temp_sensor, 12) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o DS18B20");
        return;
    }
    ESP_LOGI(TAG, "DS18B20 inicializado com sucesso!");

    // Cria uma tarefa do FreeRTOS para monitorar os sensores e controlar os dispositivos
    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
