#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Endereço I2C padrão do MPU6050
 *
 * Se AD0 (pino 9 do MPU6050) estiver conectado ao GND, o endereço é 0x68.
 * Se estiver em VCC, o endereço muda para 0x69.
 * Aqui, assumimos o mais comum (AD0=GND): 0x68.
 */
#define MPU6050_I2C_ADDRESS       0x68

//======================================================================================
// Definições dos registradores importantes no MPU6050
//======================================================================================

/**
 * @brief WHO_AM_I (0x75)
 *
 * Registrador usado para identificar o dispositivo.
 * O valor de leitura esperado para o MPU6050 é 0x68 se AD0 estiver conectado ao GND
 */
#define MPU6050_REG_WHO_AM_I      0x75

/**
 * @brief PWR_MGMT_1 (0x6B)
 *
 * Registrador de gerenciamento de energia (Power Management).
 * Controla o modo de suspensão (sleep), reset de dispositivos, fonte de clock, etc.
 */
#define MPU6050_REG_PWR_MGMT_1    0x6B

/**
 * @brief SMPLRT_DIV (0x19)
 *
 * Registrador que define o divisor de taxa de amostragem.
 * A taxa interna do MPU (por exemplo, 8 kHz) é dividida por (1 + SMPLRT_DIV).
 */
#define MPU6050_REG_SMPLRT_DIV    0x19

/**
 * @brief CONFIG (0x1A)
 *
 * Configura o filtro passa-baixa digital (DLPF) e a sincronização externa.
 */
#define MPU6050_REG_CONFIG        0x1A

/**
 * @brief GYRO_CONFIG (0x1B)
 *
 * Configura a faixa de escala do giroscópio (±250, ±500, ±1000, ±2000 deg/s).
 * Também pode habilitar o modo de self-test no giroscópio.
 */
#define MPU6050_REG_GYRO_CONFIG   0x1B

/**
 * @brief ACCEL_CONFIG (0x1C)
 *
 * Configura a faixa de escala do acelerômetro (±2g, ±4g, ±8g, ±16g).
 * Também pode habilitar o modo de self-test no acelerômetro.
 */
#define MPU6050_REG_ACCEL_CONFIG  0x1C

/**
 * @brief INT_ENABLE (0x38)
 *
 * Controla as interrupções disponíveis no MPU6050 (por exemplo, Data Ready).
 */
#define MPU6050_REG_INT_ENABLE    0x38

/**
 * @brief ACCEL_XOUT_H (0x3B)
 *
 * Registrador de início da leitura dos dados de aceleração.
 * A partir dele, em sequência, são lidos: ACCEL_XOUT_H/L, ACCEL_YOUT_H/L,
 * ACCEL_ZOUT_H/L, TEMP_OUT_H/L, GYRO_XOUT_H/L, GYRO_YOUT_H/L e GYRO_ZOUT_H/L.
 */
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

/**
 * @brief GYRO_XOUT_H (0x43)
 *
 * Registrador de início da leitura dos dados de giroscópio.
 * A partir dele, em sequência, são lidos: GYRO_XOUT_H/L, GYRO_YOUT_H/L e GYRO_ZOUT_H/L.
 */
#define MPU6050_REG_GYRO_XOUT_H   0x43

/**
 * @brief Quantidade total de bytes que abrangem Aceleração (XYZ), Temperatura e Giroscópio (XYZ).
 *
 * ACCEL_XOUT_H/L (2 bytes), ACCEL_YOUT_H/L (2 bytes), ACCEL_ZOUT_H/L (2 bytes),
 * TEMP_OUT_H/L (2 bytes), GYRO_XOUT_H/L (2 bytes), GYRO_YOUT_H/L (2 bytes),
 * GYRO_ZOUT_H/L (2 bytes).
 * Total: 14 bytes.
 */
#define MPU6050_DATA_LENGTH  14

//======================================================================================
// Estrutura de dados para armazenar leituras do MPU6050
//======================================================================================

/**
 * @brief Estrutura para armazenar leituras do sensor
 *
 * Os campos são:
 *  - accel_x, accel_y, accel_z: Aceleração em cada eixo (em g), dado configurado em ±2g, ±4g etc.
 *  - gyro_x, gyro_y, gyro_z: Velocidade angular em cada eixo (em deg/s), configurado em ±250, ±500 etc.
 *  - temperature: Temperatura interna do sensor em graus Celsius.
 */
typedef struct {
    float accel_x;      ///< Aceleração eixo X (g)
    float accel_y;      ///< Aceleração eixo Y (g)
    float accel_z;      ///< Aceleração eixo Z (g)
    float gyro_x;       ///< Velocidade angular eixo X (deg/s)
    float gyro_y;       ///< Velocidade angular eixo Y (deg/s)
    float gyro_z;       ///< Velocidade angular eixo Z (deg/s)
    float temperature;  ///< Temperatura interna do sensor (°C)
} mpu6050_data_t;

//======================================================================================
// Declaração das funções públicas para uso do MPU6050
//======================================================================================

/**
 * @brief Inicializa o MPU6050 (configura registradores básicos, etc.)
 *
 * Esta função verifica o identificador WHO_AM_I,
 * configura o clock, define faixas de leitura (±2g, ±250 deg/s),
 * habilita interrupção de Data Ready e sai do modo Sleep.
 *
 * @param i2c_port Porta I2C (por exemplo, I2C_NUM_0) já configurada em modo MASTER.
 * @return ESP_OK se sucesso; caso contrário, retorna um código de erro (ex.: ESP_FAIL).
 */
esp_err_t mpu6050_init(i2c_port_t i2c_port);

/**
 * @brief Lê dados de acelerômetro, giroscópio e temperatura do MPU6050 (14 bytes)
 *
 * Esta função lê 14 bytes sequenciais do MPU6050:
 *   - Acelerômetro (X, Y, Z)
 *   - Temperatura
 *   - Giroscópio (X, Y, Z)
 * Faz a conversão dos valores brutos (16 bits) para unidades físicas
 * e preenche a estrutura #mpu6050_data_t.
 *
 * @param i2c_port  Porta I2C (por exemplo, I2C_NUM_0)
 * @param out_data  Ponteiro para a estrutura que receberá os valores convertidos
 * @return ESP_OK se sucesso; caso contrário, retorna um código de erro
 */
esp_err_t mpu6050_read(i2c_port_t i2c_port, mpu6050_data_t *out_data);

/**
 * @brief Lê somente os dados de aceleração (X, Y, Z) do MPU6050
 *
 * Esta função lê 6 bytes (2 para cada eixo) a partir do registrador de aceleração.
 * Converte o valor bruto (16 bits) para unidades de gravidade (g).
 *
 * @param i2c_port  Porta I2C (por exemplo, I2C_NUM_0)
 * @param accX      Ponteiro onde será armazenado o valor de aceleração em X (g)
 * @param accY      Ponteiro onde será armazenado o valor de aceleração em Y (g)
 * @param accZ      Ponteiro onde será armazenado o valor de aceleração em Z (g)
 * @return ESP_OK se sucesso; caso contrário, retorna um código de erro
 */
esp_err_t mpu6050_read_accel(i2c_port_t i2c_port, float *accX, float *accY, float *accZ);

/**
 * @brief Lê somente os dados de giroscópio (X, Y, Z) do MPU6050
 *
 * Esta função lê 6 bytes (2 para cada eixo) a partir do registrador do giroscópio.
 * Converte o valor bruto (16 bits) para unidades de graus por segundo (deg/s).
 *
 * @param i2c_port  Porta I2C (por exemplo, I2C_NUM_0)
 * @param gyroX     Ponteiro onde será armazenado o valor de rotação em X (deg/s)
 * @param gyroY     Ponteiro onde será armazenado o valor de rotação em Y (deg/s)
 * @param gyroZ     Ponteiro onde será armazenado o valor de rotação em Z (deg/s)
 * @return ESP_OK se sucesso; caso contrário, retorna um código de erro
 */
esp_err_t mpu6050_read_gyro(i2c_port_t i2c_port, float *gyroX, float *gyroY, float *gyroZ);

/**
 * @brief Lê todos os dados em uma única função (acel, giro e temperatura)
 *
 * Semelhante a @ref mpu6050_read, porém mantém um nome que reflete mais claramente
 * que todos os dados são retornados. Lê 14 bytes (acel, temp, gyro).
 *
 * @param i2c_port  Porta I2C (por exemplo, I2C_NUM_0)
 * @param out_data  Estrutura #mpu6050_data_t para receber os valores convertidos
 * @return ESP_OK se sucesso; caso contrário, retorna um código de erro
 */
esp_err_t mpu6050_read_all(i2c_port_t i2c_port, mpu6050_data_t *out_data);

/**
 * @brief Calcula os ângulos de Euler (pitch e roll) a partir de valores de aceleração
 *
 * @note Este método considera que o sensor está sujeito apenas à gravidade.
 *       Para movimentos bruscos ou para cálculo de yaw, é necessário fusão de sensores
 *       (acel + giro + possivelmente magnetômetro) ou uso de filtros complementares/Kalman.
 *
 * @param accX     Valor de aceleração no eixo X (em g)
 * @param accY     Valor de aceleração no eixo Y (em g)
 * @param accZ     Valor de aceleração no eixo Z (em g)
 * @param pitch    Ponteiro para onde será armazenado o ângulo de pitch calculado (em graus)
 * @param roll     Ponteiro para onde será armazenado o ângulo de roll calculado (em graus)
 *
 * @return Esta função não retorna valor; os resultados são escritos em @p pitch e @p roll.
 */
void mpu6050_compute_euler_angles_from_accel(float accX, float accY, float accZ,
                                             float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
