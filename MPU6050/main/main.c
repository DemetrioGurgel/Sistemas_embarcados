#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

static const char *TAG = "MAIN";

// Função auxiliar para reinicializar o sensor após falha de leitura
static esp_err_t try_reinit_sensor(i2c_port_t i2c_port)
{
    ESP_LOGW(TAG, "Tentando re-inicializar o MPU6050...");
    esp_err_t ret = mpu6050_init(i2c_port);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 re-inicializado com sucesso!");
    } else {
        ESP_LOGE(TAG, "Falha ao re-inicializar o MPU6050 (erro=0x%x).", ret);
    }
    return ret;
}

void app_main(void)
{
    //----------------------------------------------------------------------------------
    // 1) Configura o driver I2C
    //----------------------------------------------------------------------------------
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    //----------------------------------------------------------------------------------
    // 2) Tenta inicializar o MPU6050 (loop para caso falhe de cara)
    //----------------------------------------------------------------------------------
    while (true) {
        ESP_LOGW(TAG, "Tentando inicializar o MPU6050...");
        esp_err_t init_err = mpu6050_init(I2C_MASTER_NUM);
        if (init_err == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 inicializado com sucesso!");
            break; // Sai do while e segue o programa
        } else {
            ESP_LOGW(TAG, "Falha ao inicializar o MPU6050 (erro=0x%x).", init_err);
            ESP_LOGW(TAG, "Aguardando 10s para tentar novamente...");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }

    //----------------------------------------------------------------------------------
    // 3) Loop principal de leituras
    //----------------------------------------------------------------------------------
    mpu6050_data_t sensor_data;

    while (1) {
        // Lê todos os dados (acel, giro, temp) de uma só vez
        esp_err_t ret_read = mpu6050_read_all(I2C_MASTER_NUM, &sensor_data);
        if (ret_read != ESP_OK) {
            // Se falhar (ex.: sensor desconectado), aguardamos 10s e tentamos re-inicializar
            ESP_LOGE(TAG, "Falha ao ler dados do sensor (0x%x). Aguardando 10s...", ret_read);
            vTaskDelay(pdMS_TO_TICKS(10000));

            // Tenta re-inicializar o sensor
            esp_err_t ret_reinit = try_reinit_sensor(I2C_MASTER_NUM);
            if (ret_reinit != ESP_OK) {
                // Se ainda falhar, continua no loop, tenta de novo em 10s.
                // Isso evita loop rápido e infinito de erro.
                continue;
            }
            // Se reinit OK, segue para próxima iteração para tentar ler novamente
            continue;
        }

        // Se a leitura foi bem-sucedida, pegamos aceleração e giroscópio
        float ax = sensor_data.accel_x;
        float ay = sensor_data.accel_y;
        float az = sensor_data.accel_z;
        float gx = sensor_data.gyro_x;
        float gy = sensor_data.gyro_y;
        float gz = sensor_data.gyro_z;
        float temp = sensor_data.temperature;

        // Calcula pitch & roll a partir da aceleração
        float pitch, roll;
        mpu6050_compute_euler_angles_from_accel(ax, ay, az, &pitch, &roll);

        // Exibe leituras atuais
        ESP_LOGI(TAG, "Accel: X=%.2f g, Y=%.2f g, Z=%.2f g", ax, ay, az);
        ESP_LOGI(TAG, "Gyro : X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s", gx, gy, gz);
        ESP_LOGI(TAG, "Temp : %.2f °C", temp);
        ESP_LOGI(TAG, "Angles: pitch=%.2f°, roll=%.2f°", pitch, roll);

        //----------------------------------------------------------------------------------
        // DETECÇÃO DE ORIENTAÇÃO (SIMPLIFICADA POR EIXOS)
        //----------------------------------------------------------------------------------
        // - Se |az| > 0.8 e (|ax|<0.5, |ay|<0.5) => Horizontal (UP ou DOWN)
        // - Se |ax| > 0.8 e (|ay|<0.5, |az|<0.5) => Vertical no eixo X
        // - Se |ay| > 0.8 e (|ax|<0.5, |az|<0.5) => Vertical no eixo Y
        // - Caso contrário => Inclinado/Diagonal

        char orientation[32] = "Unknown";

        if ((fabsf(az) > 0.8f) && (fabsf(ax) < 0.5f) && (fabsf(ay) < 0.5f)) {
            if (az > 0) {
                snprintf(orientation, sizeof(orientation), "Horizontal (UP)");
            } else {
                snprintf(orientation, sizeof(orientation), "Horizontal (DOWN)");
            }
        } else if ((fabsf(ax) > 0.8f) && (fabsf(ay) < 0.5f) && (fabsf(az) < 0.5f)) {
            if (ax > 0) {
                snprintf(orientation, sizeof(orientation), "Vertical (X+)");
            } else {
                snprintf(orientation, sizeof(orientation), "Vertical (X-)");
            }
        } else if ((fabsf(ay) > 0.8f) && (fabsf(ax) < 0.5f) && (fabsf(az) < 0.5f)) {
            if (ay > 0) {
                snprintf(orientation, sizeof(orientation), "Vertical (Y+)");
            } else {
                snprintf(orientation, sizeof(orientation), "Vertical (Y-)");
            }
        } else {
            snprintf(orientation, sizeof(orientation), "Inclinado/Diagonal");
        }

        ESP_LOGI(TAG, "Orientação: %s \n", orientation);

        // Aguarda 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
