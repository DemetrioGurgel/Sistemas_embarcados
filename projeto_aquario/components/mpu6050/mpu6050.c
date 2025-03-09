#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Inclusões de cabeçalhos matemáticos para atan2, sqrt etc.
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "MPU6050";

/**
 * @brief  Escreve 1 byte em um registrador do MPU6050
 *
 * @param  i2c_port   Porta I2C utilizada (ex.: I2C_NUM_0)
 * @param  reg_addr   Endereço do registrador no MPU6050 onde queremos escrever
 * @param  data       Byte que será escrito no registrador
 *
 * @return esp_err_t  Retorna ESP_OK caso sucesso ou código de erro
 */
static esp_err_t mpu6050_write_byte(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief  Lê 'length' bytes a partir de um registrador inicial (start_reg_addr) do MPU6050
 *
 * @param  i2c_port         Porta I2C utilizada (ex.: I2C_NUM_0)
 * @param  start_reg_addr   Endereço do registrador inicial no MPU6050 de onde começaremos a ler
 * @param  buffer           Ponteiro para onde os bytes lidos serão armazenados
 * @param  length           Quantidade de bytes a serem lidos
 *
 * @return esp_err_t        Retorna ESP_OK caso sucesso ou código de erro
 */
static esp_err_t mpu6050_read_bytes(i2c_port_t i2c_port,
                                    uint8_t start_reg_addr,
                                    uint8_t *buffer,
                                    size_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buffer[length - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief  Inicializa o sensor MPU6050
 *
 * @param  i2c_port   Porta I2C utilizada
 * @return esp_err_t  Retorna ESP_OK caso sucesso, ou erro caso falhe
 */
esp_err_t mpu6050_init(i2c_port_t i2c_port)
{
    esp_err_t ret;

    //-----------------------------------------------------------------------
    // 1. Verificar WHO_AM_I para confirmar se o dispositivo é realmente o MPU6050
    //-----------------------------------------------------------------------
    uint8_t who_am_i = 0;
    ret = mpu6050_read_bytes(i2c_port, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler WHO_AM_I (ret=0x%x)", ret);
        return ret;
    }

    // O valor esperado para o MPU6050 é 0x68
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 não encontrado (WHO_AM_I=0x%x)", who_am_i);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 detectado. WHO_AM_I=0x%x", who_am_i);

    //-----------------------------------------------------------------------
    // 2. Configurar registradores para inicialização
    //-----------------------------------------------------------------------

    // (a) Sair do modo SLEEP (PWR_MGMT_1 = 0x00)
    //     Desabilita o reset e o modo de suspensão.
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao escrever PWR_MGMT_1");
        return ret;
    }
    // Aguardamos um pouco para garantir que o sensor se estabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // (b) Sample Rate Divider
    //     Se o clock interno é 8kHz, dividir por (1 + 79) = 80 => 8kHz/80 = 100 Hz
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_SMPLRT_DIV, 79);
    if (ret != ESP_OK) {
        return ret;
    }

    // (c) CONFIG => DLPF=3 (~44 Hz)
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) {
        return ret;
    }

    // (d) GYRO_CONFIG => ±250 deg/s
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // (e) ACCEL_CONFIG => ±2g
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // (f) INT_ENABLE => habilita Data Ready (bit 0 = 1)
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_INT_ENABLE, 0x01);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 inicializado com sucesso.");
    return ESP_OK;
}

/**
 * @brief  Lê os dados de aceleração, giroscópio e temperatura do MPU6050 (14 bytes)
 *
 * @param  i2c_port    Porta I2C
 * @param  out_data    Ponteiro para a estrutura onde os dados lidos serão armazenados
 *
 * @return esp_err_t   ESP_OK em caso de sucesso; caso contrário, um código de erro
 */
esp_err_t mpu6050_read(i2c_port_t i2c_port, mpu6050_data_t *out_data)
{
    // O MPU6050 disponibiliza 14 bytes consecutivos para:
    //   ACCEL_XOUT_H/L (2 bytes)
    //   ACCEL_YOUT_H/L (2 bytes)
    //   ACCEL_ZOUT_H/L (2 bytes)
    //   TEMP_OUT_H/L   (2 bytes)
    //   GYRO_XOUT_H/L  (2 bytes)
    //   GYRO_YOUT_H/L  (2 bytes)
    //   GYRO_ZOUT_H/L  (2 bytes)
    uint8_t data_buf[MPU6050_DATA_LENGTH]; // Tamanho de 14 bytes
    esp_err_t ret = mpu6050_read_bytes(i2c_port,
                                       MPU6050_REG_ACCEL_XOUT_H,
                                       data_buf,
                                       MPU6050_DATA_LENGTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler dados do MPU6050");
        return ret;
    }

    // Cada valor (acel, temp, gyro) tem 2 bytes (16 bits).
    int16_t raw_accel_x = (data_buf[0] << 8) | data_buf[1];
    int16_t raw_accel_y = (data_buf[2] << 8) | data_buf[3];
    int16_t raw_accel_z = (data_buf[4] << 8) | data_buf[5];
    int16_t raw_temp    = (data_buf[6] << 8) | data_buf[7];
    int16_t raw_gyro_x  = (data_buf[8] << 8) | data_buf[9];
    int16_t raw_gyro_y  = (data_buf[10] << 8) | data_buf[11];
    int16_t raw_gyro_z  = (data_buf[12] << 8) | data_buf[13];

    // Conversão para unidades físicas (configurado para ±2g e ±250 deg/s):
    out_data->accel_x = (float)raw_accel_x / 16384.0f; // 1g = 16384 LSB
    out_data->accel_y = (float)raw_accel_y / 16384.0f;
    out_data->accel_z = (float)raw_accel_z / 16384.0f;

    out_data->temperature = ((float)raw_temp / 340.0f) + 36.53f;

    out_data->gyro_x = (float)raw_gyro_x / 131.0f;     // 1 deg/s = 131 LSB
    out_data->gyro_y = (float)raw_gyro_y / 131.0f;
    out_data->gyro_z = (float)raw_gyro_z / 131.0f;

    return ESP_OK;
}

/**
 * @brief  Lê somente os dados de aceleração (6 bytes)
 *
 * @param  i2c_port  Porta I2C utilizada
 * @param  accX      Saída: aceleração em X (g)
 * @param  accY      Saída: aceleração em Y (g)
 * @param  accZ      Saída: aceleração em Z (g)
 *
 * @return esp_err_t ESP_OK em caso de sucesso; caso contrário, erro
 */
esp_err_t mpu6050_read_accel(i2c_port_t i2c_port, float *accX, float *accY, float *accZ)
{
    uint8_t data_buf[6]; // 2 bytes para cada eixo (X, Y, Z)
    esp_err_t ret = mpu6050_read_bytes(i2c_port, MPU6050_REG_ACCEL_XOUT_H, data_buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler dados de aceleração do MPU6050");
        return ret;
    }

    int16_t raw_accel_x = (data_buf[0] << 8) | data_buf[1];
    int16_t raw_accel_y = (data_buf[2] << 8) | data_buf[3];
    int16_t raw_accel_z = (data_buf[4] << 8) | data_buf[5];

    // Converte para 'g' (±2g => 1g = 16384 LSB)
    *accX = (float)raw_accel_x / 16384.0f;
    *accY = (float)raw_accel_y / 16384.0f;
    *accZ = (float)raw_accel_z / 16384.0f;

    return ESP_OK;
}

/**
 * @brief  Lê somente os dados de giroscópio (6 bytes)
 *
 * @param  i2c_port  Porta I2C utilizada
 * @param  gyroX     Saída: rotação em X (deg/s)
 * @param  gyroY     Saída: rotação em Y (deg/s)
 * @param  gyroZ     Saída: rotação em Z (deg/s)
 *
 * @return esp_err_t ESP_OK em caso de sucesso; caso contrário, erro
 */
esp_err_t mpu6050_read_gyro(i2c_port_t i2c_port, float *gyroX, float *gyroY, float *gyroZ)
{
    uint8_t data_buf[6]; // 2 bytes para cada eixo (X, Y, Z)
    esp_err_t ret = mpu6050_read_bytes(i2c_port, MPU6050_REG_GYRO_XOUT_H, data_buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler dados de giroscópio do MPU6050");
        return ret;
    }

    int16_t raw_gyro_x = (data_buf[0] << 8) | data_buf[1];
    int16_t raw_gyro_y = (data_buf[2] << 8) | data_buf[3];
    int16_t raw_gyro_z = (data_buf[4] << 8) | data_buf[5];

    // Converte para deg/s (±250 deg/s => 1 deg/s = 131 LSB)
    *gyroX = (float)raw_gyro_x / 131.0f;
    *gyroY = (float)raw_gyro_y / 131.0f;
    *gyroZ = (float)raw_gyro_z / 131.0f;

    return ESP_OK;
}

/**
 * @brief  Lê todos os dados em uma única função (acel, giro e temperatura)
 *
 * @param  i2c_port  Porta I2C utilizada
 * @param  out_data  Estrutura que receberá os valores lidos
 * @return esp_err_t
 */
esp_err_t mpu6050_read_all(i2c_port_t i2c_port, mpu6050_data_t *out_data)
{
    // 14 bytes: 6 de acel + 2 de temp + 6 de giro
    uint8_t data_buf[14];
    esp_err_t ret = mpu6050_read_bytes(i2c_port, MPU6050_REG_ACCEL_XOUT_H, data_buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler dados do MPU6050");
        return ret;
    }

    int16_t raw_accel_x = (data_buf[0] << 8) | data_buf[1];
    int16_t raw_accel_y = (data_buf[2] << 8) | data_buf[3];
    int16_t raw_accel_z = (data_buf[4] << 8) | data_buf[5];
    int16_t raw_temp    = (data_buf[6] << 8) | data_buf[7];
    int16_t raw_gyro_x  = (data_buf[8] << 8) | data_buf[9];
    int16_t raw_gyro_y  = (data_buf[10] << 8) | data_buf[11];
    int16_t raw_gyro_z  = (data_buf[12] << 8) | data_buf[13];

    out_data->accel_x = (float)raw_accel_x / 16384.0f;
    out_data->accel_y = (float)raw_accel_y / 16384.0f;
    out_data->accel_z = (float)raw_accel_z / 16384.0f;

    out_data->temperature = ((float)raw_temp / 340.0f) + 36.53f;

    out_data->gyro_x = (float)raw_gyro_x / 131.0f;
    out_data->gyro_y = (float)raw_gyro_y / 131.0f;
    out_data->gyro_z = (float)raw_gyro_z / 131.0f;

    return ESP_OK;
}

/**
 * @brief  Calcula os ângulos de Euler (pitch e roll) a partir dos valores de aceleração
 *
 * @param  accX   Aceleração no eixo X (g)
 * @param  accY   Aceleração no eixo Y (g)
 * @param  accZ   Aceleração no eixo Z (g)
 * @param  pitch  Ponteiro para ângulo de pitch (graus)
 * @param  roll   Ponteiro para ângulo de roll (graus)
 */
void mpu6050_compute_euler_angles_from_accel(float accX, float accY, float accZ,
                                             float *pitch, float *roll)
{
    // Usa atan2f e sqrtf para melhor desempenho em float
    *pitch = atan2f(accX, sqrtf(accY * accY + accZ * accZ)) * (180.0f / M_PI);
    *roll  = atan2f(accY, accZ) * (180.0f / M_PI);
}
