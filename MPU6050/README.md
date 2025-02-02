# Biblioteca MPU6050

Esta biblioteca fornece uma interface para comunicação com o sensor MPU6050 via I2C no ESP32. Com ela, é possível inicializar o sensor, configurar os registradores necessários e obter leituras de aceleração, giroscópio e temperatura.

## Funcionalidades
- Inicializar o sensor MPU6050.
- Ler dados completos do acelerômetro, giroscópio e temperatura.
- Ler apenas os dados do acelerômetro ou giroscópio separadamente.
- Calcular ângulos de Euler (pitch e roll) a partir da aceleração.

## Dependências
A biblioteca utiliza os seguintes componentes do ESP-IDF:
- `esp_err.h`: Para tratamento de erros.
- `driver/i2c.h`: Para comunicação I2C.
- `math.h`: Para cálculos de ângulos.

---

## Como Usar a Biblioteca

### Passo 1: Configurar o I2C no ESP32
Antes de usar a biblioteca, configure corretamente o driver I2C:

```c
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
```

### Passo 2: Inicializar o Sensor MPU6050
Utilize `mpu6050_init` para configurar o sensor:

```c
#include "mpu6050.h"

void app_main() {
    i2c_master_init();

    if (mpu6050_init(I2C_NUM_0) == ESP_OK) {
        printf("MPU6050 inicializado com sucesso.\n");
    } else {
        printf("Erro ao inicializar o MPU6050.\n");
    }
}
```

### Passo 3: Ler Dados do Sensor
Após inicializar o sensor, você pode obter leituras completas:

```c
#include "mpu6050.h"

void app_main() {
    i2c_master_init();

    if (mpu6050_init(I2C_NUM_0) != ESP_OK) {
        printf("Erro ao inicializar o MPU6050.\n");
        return;
    }

    mpu6050_data_t sensor_data;
    if (mpu6050_read_all(I2C_NUM_0, &sensor_data) == ESP_OK) {
        printf("Aceleração: X=%.2f, Y=%.2f, Z=%.2f (g)\n", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        printf("Giroscópio: X=%.2f, Y=%.2f, Z=%.2f (°/s)\n", sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        printf("Temperatura: %.2f °C\n", sensor_data.temperature);
    } else {
        printf("Erro ao ler os dados do MPU6050.\n");
    }
}
```

### Passo 4: Ler Apenas Aceleração ou Giroscópio
Se precisar de leituras específicas:

```c
float ax, ay, az, gx, gy, gz;

// Ler aceleração
mpu6050_read_accel(I2C_NUM_0, &ax, &ay, &az);
printf("Aceleração: X=%.2f, Y=%.2f, Z=%.2f (g)\n", ax, ay, az);

// Ler giroscópio
mpu6050_read_gyro(I2C_NUM_0, &gx, &gy, &gz);
printf("Giroscópio: X=%.2f, Y=%.2f, Z=%.2f (°/s)\n", gx, gy, gz);
```

---

## Definições Importantes

### Endereço I2C
O endereço padrão do MPU6050 é `0x68`, podendo ser `0x69` se o pino AD0 estiver conectado ao VCC.

```c
#define MPU6050_I2C_ADDRESS 0x68
```

### Funções Principais

- `mpu6050_init(i2c_port_t i2c_port)`: Inicializa o sensor.
- `mpu6050_read_all(i2c_port_t i2c_port, mpu6050_data_t *out_data)`: Lê aceleração, giroscópio e temperatura.
- `mpu6050_read_accel(i2c_port_t i2c_port, float *accX, float *accY, float *accZ)`: Lê apenas aceleração.
- `mpu6050_read_gyro(i2c_port_t i2c_port, float *gyroX, float *gyroY, float *gyroZ)`: Lê apenas giroscópio.
- `mpu6050_compute_euler_angles_from_accel(float accX, float accY, float accZ, float *pitch, float *roll)`: Calcula ângulos de pitch e roll.

---

