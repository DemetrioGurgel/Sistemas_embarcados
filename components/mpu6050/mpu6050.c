#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Tag utilizado para identificação nos logs
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
    // Cria um "comando" para I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Envia condição de START na linha I2C
    i2c_master_start(cmd);

    // Escreve o endereço do dispositivo (MPU6050_I2C_ADDRESS) com bit de escrita (I2C_MASTER_WRITE)
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    // Escreve o endereço do registrador no qual queremos gravar
    i2c_master_write_byte(cmd, reg_addr, true);

    // Escreve o dado (1 byte) nesse registrador
    i2c_master_write_byte(cmd, data, true);

    // Finaliza a transmissão enviando STOP
    i2c_master_stop(cmd);

    // Envia todo o comando para o driver I2C (com timeout de 1000 ms)
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    
    // Libera o comando (cleanup)
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
static esp_err_t mpu6050_read_bytes(i2c_port_t i2c_port, uint8_t start_reg_addr, uint8_t *buffer, size_t length)
{
    // Cria um comando para I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Inicia condição de START
    i2c_master_start(cmd);

    // Envia endereço do dispositivo + modo de escrita
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    // Envia o endereço do registrador inicial
    i2c_master_write_byte(cmd, start_reg_addr, true);

    // Envia uma condição de RESTART para mudar para modo de leitura
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

    // Lê os bytes requisitados
    // Se houver mais de 1 byte a ler, lê (length - 1) com ACK
    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1, I2C_MASTER_ACK);
    }
    // Lê o último byte com NACK para encerrar a leitura
    i2c_master_read_byte(cmd, &buffer[length - 1], I2C_MASTER_NACK);

    // Envia STOP para finalizar
    i2c_master_stop(cmd);

    // Envia o comando para o driver I2C (com timeout de 1000 ms)
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    
    // Libera o comando (cleanup)
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
    //     Se o clock interno é 8kHz, dividir por (1 + 79) = 80 => 8kHz/80 = 100 Hz de amostragem
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_SMPLRT_DIV, 79); // 0x4F
    if (ret != ESP_OK) {
        return ret;
    }

    // (c) CONFIG => DLPF=3 (~44 Hz)
    //     Filtro passa-baixa digital para reduzir ruído
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) {
        return ret;
    }

    // (d) GYRO_CONFIG => ±250 deg/s
    //     Define a escala do giroscópio
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // (e) ACCEL_CONFIG => ±2g
    //     Define a escala do acelerômetro
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // (f) INT_ENABLE => habilita Data Ready (bit 0 = 1)
    //     Permite que o sensor gere interrupções quando houver novos dados disponíveis
    ret = mpu6050_write_byte(i2c_port, MPU6050_REG_INT_ENABLE, 0x01);
    if (ret != ESP_OK) {
        return ret;
    }

    // Log para indicar sucesso na inicialização
    ESP_LOGI(TAG, "MPU6050 inicializado com sucesso.");
    return ESP_OK;
}

/**
 * @brief  Lê os dados de aceleração, giroscópio e temperatura do MPU6050
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
    // Fazemos o shift e or para unir os bytes alto (H) e baixo (L).
    int16_t raw_accel_x = (data_buf[0] << 8) | data_buf[1];
    int16_t raw_accel_y = (data_buf[2] << 8) | data_buf[3];
    int16_t raw_accel_z = (data_buf[4] << 8) | data_buf[5];
    int16_t raw_temp    = (data_buf[6] << 8) | data_buf[7];
    int16_t raw_gyro_x  = (data_buf[8] << 8) | data_buf[9];
    int16_t raw_gyro_y  = (data_buf[10] << 8) | data_buf[11];
    int16_t raw_gyro_z  = (data_buf[12] << 8) | data_buf[13];

    // Conversão dos valores brutos (raw) para unidades físicas
    // De acordo com a configuração escolhida (±2g e ±250 deg/s):
    //   Aceleração: 1 g = 16384 LSB
    //   Giroscópio: 1 deg/s = 131 LSB
    //   Temperatura: T = (raw_temp / 340) + 36.53 (graus Celsius)

    out_data->accel_x = (float)raw_accel_x / 16384.0f;
    out_data->accel_y = (float)raw_accel_y / 16384.0f;
    out_data->accel_z = (float)raw_accel_z / 16384.0f;

    out_data->temperature = ((float)raw_temp / 340.0f) + 36.53f;

    out_data->gyro_x = (float)raw_gyro_x / 131.0f;
    out_data->gyro_y = (float)raw_gyro_y / 131.0f;
    out_data->gyro_z = (float)raw_gyro_z / 131.0f;

    return ESP_OK;
}
