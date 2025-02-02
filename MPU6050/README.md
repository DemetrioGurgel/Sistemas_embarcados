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

### Passo 5: Calcular Ângulos de Euler (Pitch e Roll)
A partir dos dados do acelerômetro, podemos estimar os ângulos de inclinação:

```c
#include "mpu6050.h"

void app_main() {
    i2c_master_init();

    if (mpu6050_init(I2C_NUM_0) != ESP_OK) {
        printf("Erro ao inicializar o MPU6050.\n");
        return;
    }

    float accX, accY, accZ, pitch, roll;
    
    // Ler aceleração
    if (mpu6050_read_accel(I2C_NUM_0, &accX, &accY, &accZ) == ESP_OK) {
        // Calcular ângulos de pitch e roll
        mpu6050_compute_euler_angles_from_accel(accX, accY, accZ, &pitch, &roll);
        printf("Pitch: %.2f°, Roll: %.2f°\n", pitch, roll);
    } else {
        printf("Erro ao ler aceleração do MPU6050.\n");
    }
}
```

---

## Definições Importantes

### `mpu6050_compute_euler_angles_from_accel`
Calcula os ângulos de Euler (pitch e roll) a partir da aceleração.

##### Protótipo
```c
void mpu6050_compute_euler_angles_from_accel(float accX, float accY, float accZ,
                                             float *pitch, float *roll);
```

##### Parâmetros
- `accX`: Aceleração no eixo X (g).
- `accY`: Aceleração no eixo Y (g).
- `accZ`: Aceleração no eixo Z (g).
- `pitch`: Ponteiro onde será armazenado o ângulo de inclinação em torno do eixo Y (graus).
- `roll`: Ponteiro onde será armazenado o ângulo de inclinação em torno do eixo X (graus).

##### Retorno
Esta função não retorna um valor. Os resultados são armazenados nos ponteiros `pitch` e `roll`.

##### Fórmula
Os ângulos são calculados com base nos valores de aceleração usando as fórmulas:

- **Pitch (inclinação para frente/trás):**  
  \[ 	heta = rctan \left( rac{	ext{accX}}{\sqrt{	ext{accY}^2 + 	ext{accZ}^2}} 
ight) 	imes rac{180}{\pi} \]

- **Roll (inclinação lateral):**  
  \[ \phi = rctan \left( rac{	ext{accY}}{	ext{accZ}} 
ight) 	imes rac{180}{\pi} \]

Essas equações assumem que o sensor está parado e sujeito apenas à gravidade.

---

