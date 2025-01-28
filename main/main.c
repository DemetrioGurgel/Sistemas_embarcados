#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

// Tag usado para identificação das mensagens no log
static const char *TAG = "MPU6050_DEMO";

// Definições dos pinos e configurações da I2C
// Ajuste de acordo com o hardware utilizado.
#define I2C_MASTER_SCL_IO    22    // GPIO para SCL
#define I2C_MASTER_SDA_IO    21    // GPIO para SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000  // Frequência da I2C (100kHz)

// Função (tarefa) que lê o MPU6050 e determina orientação e movimento
static void mpu6050_orientation_task(void *pvParameter)
{
    // Estrutura para armazenar as leituras do MPU6050
    mpu6050_data_t sensor_data;

    // Variáveis para auxiliar na detecção de movimento brusco
    // Estas variáveis guardarão a leitura anterior para comparar com a atual.
    float last_accel_x = 0, last_accel_y = 0, last_accel_z = 0;

    // Threshold para considerar uma diferença brusca de aceleração
    const float ACCEL_DIFF_THRESHOLD = 0.5f;

    while (1) {
        // Lê dados do MPU6050 através de nossa biblioteca
        esp_err_t ret = mpu6050_read(I2C_MASTER_NUM, &sensor_data);

        // Verifica se a leitura ocorreu sem erros
        if (ret == ESP_OK) {
            // Variáveis locais para ficar mais claro (sintético)
            // ax, ay, az -> Aceleração em g (gravidade)
            // gx, gy, gz -> Velocidade angular em deg/s (graus por segundo)
            float ax = sensor_data.accel_x;
            float ay = sensor_data.accel_y;
            float az = sensor_data.accel_z;
            float gx = sensor_data.gyro_x;
            float gy = sensor_data.gyro_y;
            float gz = sensor_data.gyro_z;

            //---------------------------------------------------------
            // 1) DETECÇÃO DE ORIENTAÇÃO (SIMPLIFICADA)
            //---------------------------------------------------------
            // Usamos thresholds simples:
            // - Se |az| > 0.8 e os outros eixos pequenos => Horizontal
            // - Se |ax| > 0.8 => Vertical em X
            // - Se |ay| > 0.8 => Vertical em Y
            // - Caso contrário => consideramos "Inclinado/Diagonal"
            //
            // Obs.: Estes são exemplos bem simplificados e podem não cobrir
            // todas as posições possíveis do sensor.

            // Calcula a magnitude total da aceleração para referência
            // (pode ser útil em outras verificações, ex. movimento brusco).
            float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

            // String para descrever a orientação detectada
            char orientation[32] = "Unknown";

            // Checagem: Horizontal se Z domina
            if (fabsf(az) > 0.8f && fabsf(ax) < 0.5f && fabsf(ay) < 0.5f) {
                // Se Z for positivo, está com a face voltada para cima
                // Se Z for negativo, face voltada para baixo
                if (az > 0) {
                    snprintf(orientation, sizeof(orientation), "Horizontal (UP)");
                } else {
                    snprintf(orientation, sizeof(orientation), "Horizontal (DOWN)");
                }
            }
            // Checagem: Vertical se X domina
            else if (fabsf(ax) > 0.8f && fabsf(ay) < 0.5f && fabsf(az) < 0.5f) {
                if (ax > 0) {
                    snprintf(orientation, sizeof(orientation), "Vertical (X+)");
                } else {
                    snprintf(orientation, sizeof(orientation), "Vertical (X-)");
                }
            }
            // Checagem: Vertical se Y domina
            else if (fabsf(ay) > 0.8f && fabsf(ax) < 0.5f && fabsf(az) < 0.5f) {
                if (ay > 0) {
                    snprintf(orientation, sizeof(orientation), "Vertical (Y+)");
                } else {
                    snprintf(orientation, sizeof(orientation), "Vertical (Y-)");
                }
            }
            // Se não se encaixa nos casos acima, podemos assumir que está inclinado.
            else {
                snprintf(orientation, sizeof(orientation), "Inclinado/Diagonal");
            }

            //---------------------------------------------------------
            // 2) DETECÇÃO DE MOVIMENTO BRUSCO
            //---------------------------------------------------------
            // Verificamos a diferença entre leituras consecutivas de aceleração
            // Se a diferença em algum eixo for maior que um threshold (ACCEL_DIFF_THRESHOLD),
            // consideramos um movimento brusco.
            // Também checamos se o giroscópio excede certo valor (ex. 150 deg/s),
            // o que indica uma rotação rápida.

            bool sudden_accel = false;

            // Diferença entre leituras atuais e anteriores (eixo a eixo)
            float diff_x = fabsf(ax - last_accel_x);
            float diff_y = fabsf(ay - last_accel_y);
            float diff_z = fabsf(az - last_accel_z);

            // Checa se alguma das diferenças excede o threshold
            if (diff_x > ACCEL_DIFF_THRESHOLD ||
                diff_y > ACCEL_DIFF_THRESHOLD ||
                diff_z > ACCEL_DIFF_THRESHOLD) {
                sudden_accel = true;
            }

            // Define critério para movimento brusco via giroscópio
            bool sudden_gyro = (fabsf(gx) > 150.0f ||
                                fabsf(gy) > 150.0f ||
                                fabsf(gz) > 150.0f);

            // Atualiza as variáveis de "última leitura" para a próxima iteração
            last_accel_x = ax;
            last_accel_y = ay;
            last_accel_z = az;

            //---------------------------------------------------------
            // 3) IMPRIME RESULTADOS NO LOG
            //---------------------------------------------------------
            // Mostramos as leituras de aceleração, giroscópio e temperatura.
            ESP_LOGI(TAG,
                     "Accel(g): X=%.2f Y=%.2f Z=%.2f | Gyro(deg/s): X=%.2f Y=%.2f Z=%.2f | Temp=%.2f C",
                     ax, ay, az, gx, gy, gz, sensor_data.temperature);

            // Mostramos a orientação detectada e a magnitude da aceleração
            ESP_LOGI(TAG, "Orientation: %s (accel=%.2f g)", orientation, accel_magnitude);

            // Se foi detectado movimento brusco, exibimos um aviso no log
            if (sudden_accel || sudden_gyro) {
                ESP_LOGW(TAG, ">>> MOVIMENTO BRUSCO DETECTADO!");
            }

        } else {
            // Caso a leitura do sensor falhe, registramos no log um erro
            ESP_LOGE(TAG, "Erro ao ler MPU6050: %s", esp_err_to_name(ret));
        }

        // Aguarda 500ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    esp_err_t ret;

    //===============================================
    // CONFIGURAÇÃO E INICIALIZAÇÃO DA I2C
    //===============================================
    // Definimos a configuração do driver I2C:
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,        // Modo Master (ESP32 como mestre)
        .sda_io_num = I2C_MASTER_SDA_IO,  // Pino SDA
        .scl_io_num = I2C_MASTER_SCL_IO,  // Pino SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Habilita pull-up
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Habilita pull-up
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Frequência de operação da I2C
    };

    // Aplica essa configuração na porta I2C escolhida
    i2c_param_config(I2C_MASTER_NUM, &conf);

    // Instala o driver I2C com os parâmetros acima
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao instalar driver I2C: %s", esp_err_to_name(ret));
        return; // Encerramos caso não consiga instalar o driver
    }

    //===============================================
    // INICIALIZA MPU6050
    //===============================================
    // Chama a função de inicialização da nossa biblioteca.
    // Essa função faz:
    //   - Verificação do WHO_AM_I
    //   - Configuração de registradores do MPU
    //   - Ajuste de ranges para acelerômetro e giroscópio
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar MPU6050: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "MPU6050 inicializado com sucesso.");

    //===============================================
    // CRIA TAREFA PARA DETECÇÃO DE POSIÇÃO E MOVIMENTO
    //===============================================
    // Criamos uma tarefa FreeRTOS que ficará em loop lendo o sensor
    // e determinando a orientação e movimentação.
    xTaskCreate(
        mpu6050_orientation_task,    // Ponteiro da função da tarefa
        "mpu6050_orientation_task",  // Nome da tarefa (para debug)
        4096,                        // Tamanho da stack em bytes
        NULL,                        // Parâmetro passado à função (não usado)
        5,                           // Prioridade da tarefa
        NULL                         // Handle da tarefa (não usado aqui)
    );
}
