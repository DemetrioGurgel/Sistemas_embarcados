 # Biblioteca MPU6050

Esta biblioteca fornece uma interface para comunicação com o sensor MPU6050 via I2C. Com ela, é possível inicializar o sensor, configurar os registradores necessários e ler os dados de aceleração, giroscópio e temperatura.

## Funcionalidades
- Inicializar o sensor MPU6050.
- Ler dados de aceleração, giroscópio e temperatura.

## Dependências
A biblioteca utiliza os seguintes componentes:
- `esp_err.h`: Para tratamento de erros.
- `driver/i2c.h`: Para comunicação I2C no ESP32.

---

## Como Usar a Biblioteca

### Passo 1: Configurar o I2C no ESP32
Antes de usar a biblioteca, certifique-se de configurar corretamente o driver I2C no ESP32. Aqui está um exemplo de inicialização:

c
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    22  // Pino SCL
#define I2C_MASTER_SDA_IO    21  // Pino SDA
#define I2C_MASTER_FREQ_HZ   100000

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}


### Passo 2: Inicializar o Sensor MPU6050
Use a função `mpu6050_init` para inicializar o sensor. Certifique-se de que a porta I2C configurada está sendo passada corretamente.

c
#include "mpu6050.h"

void app_main() {
    i2c_master_init();

    if (mpu6050_init(I2C_NUM_0) == ESP_OK) {
        printf("MPU6050 inicializado com sucesso.\n");
    } else {
        printf("Erro ao inicializar o MPU6050.\n");
    }
}


### Passo 3: Ler Dados do Sensor
Depois de inicializar o sensor, use a função `mpu6050_read` para obter os valores do acelerômetro, giroscópio e temperatura. Os dados serão retornados em uma estrutura `mpu6050_data_t`.

c
#include "mpu6050.h"

void app_main() {
    i2c_master_init();

    if (mpu6050_init(I2C_NUM_0) != ESP_OK) {
        printf("Erro ao inicializar o MPU6050.\n");
        return;
    }

    mpu6050_data_t sensor_data;

    if (mpu6050_read(I2C_NUM_0, &sensor_data) == ESP_OK) {
        printf("Aceleração: X=%.2f, Y=%.2f, Z=%.2f (g)\n", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        printf("Giroscópio: X=%.2f, Y=%.2f, Z=%.2f (°/s)\n", sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        printf("Temperatura: %.2f °C\n", sensor_data.temperature);
    } else {
        printf("Erro ao ler os dados do MPU6050.\n");
    }
}


---

## Definições
### Endereço I2C
O endereço padrão do MPU6050 é `0x68`.

c
#define MPU6050_I2C_ADDRESS 0x68


### Registradores Importantes
Abaixo estão os principais registradores utilizados na configuração e leitura de dados:

| Registrador             | Endereço | Descrição                              |
|-------------------------|----------|----------------------------------------|
| `MPU6050_REG_WHO_AM_I` | `0x75`   | Verifica a identidade do sensor.       |
| `MPU6050_REG_PWR_MGMT_1` | `0x6B`   | Gerenciamento de energia.              |
| `MPU6050_REG_SMPLRT_DIV` | `0x19`   | Configura a taxa de amostragem.        |
| `MPU6050_REG_CONFIG`    | `0x1A`   | Configura os filtros digitais.         |
| `MPU6050_REG_GYRO_CONFIG` | `0x1B`  | Configura a sensibilidade do giroscópio. |
| `MPU6050_REG_ACCEL_CONFIG` | `0x1C` | Configura a sensibilidade do acelerômetro. |
| `MPU6050_REG_INT_ENABLE` | `0x38`  | Habilita interrupções.                 |
| `MPU6050_REG_ACCEL_XOUT_H` | `0x3B` | Registro inicial para leitura dos dados. |

---

## Estrutura de Dados

c
typedef struct {
    float accel_x;   // Aceleração em X (g)
    float accel_y;   // Aceleração em Y (g)
    float accel_z;   // Aceleração em Z (g)
    float gyro_x;    // Velocidade angular em X (°/s)
    float gyro_y;    // Velocidade angular em Y (°/s)
    float gyro_z;    // Velocidade angular em Z (°/s)
    float temperature; // Temperatura (°C)
} mpu6050_data_t;


Essa estrutura é usada para armazenar os dados convertidos lidos do sensor.

---

## Funções

### `mpu6050_init`
Inicializa o MPU6050 e configura os registradores principais.

#### Protótipo
c
esp_err_t mpu6050_init(i2c_port_t i2c_port);


#### Parâmetros
- `i2c_port`: Porta I2C a ser utilizada (por exemplo, `I2C_NUM_0`).

#### Retorno
- `ESP_OK`: Sucesso.
- Outros valores: Erro durante a inicialização.

---

### `mpu6050_read`
Lê os dados de aceleração, giroscópio e temperatura do MPU6050.

#### Protótipo
c
esp_err_t mpu6050_read(i2c_port_t i2c_port, mpu6050_data_t *out_data);


#### Parâmetros
- `i2c_port`: Porta I2C a ser utilizada (por exemplo, `I2C_NUM_0`).
- `out_data`: Ponteiro para uma estrutura `mpu6050_data_t` que receberá os dados convertidos.

#### Retorno
- `ESP_OK`: Sucesso.
- Outros valores: Erro ao ler os dados.
