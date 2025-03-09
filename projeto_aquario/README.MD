# Sistema de Monitoramento e Controle com ESP32

Este projeto utiliza um ESP32 para monitorar e controlar um sistema de resfriamento e bombeamento, baseando-se em leituras de sensores de aceleração, temperatura e outros. O sistema realiza as seguintes funções:

- **Monitoramento do nível de água:**  
  Utiliza um sensor MPU6050 para medir aceleração e, através de uma função de conversão, estima o nível de água em litros.

- **Monitoramento da temperatura:**  
  Utiliza um sensor DS18B20 para medir a temperatura ambiente.

- **Controle do Cooler:**  
  Aciona um relé SSR para ligar um cooler quando a temperatura ultrapassa 30°C e gera um alerta sonoro com um buzzer.

- **Controle da Bomba:**  
  Aciona um relé SSR para ligar ou desligar a bomba com base no nível de água (liga quando o nível está abaixo de 0.50L e desliga quando atinge ou ultrapassa 1L).

- **Display OLED:**  
  Exibe informações de nível de água e temperatura, e mostra alertas caso a temperatura esteja alta.

---

## Índice

- [Recursos](#recursos)
- [Dependências](#dependências)
- [Configuração de Hardware](#configuração-de-hardware)
- [Configuração do Ambiente](#configuração-do-ambiente)
- [Como Compilar e Flashar](#como-compilar-e-flashar)
- [Estrutura do Código](#estrutura-do-código)
- [Funcionamento](#funcionamento)

---

## Recursos

- **ESP32**
- **Sensor MPU6050** (acelerômetro e giroscópio)
- **Sensor DS18B20** (sensor de temperatura)
- **Display OLED SSD1306**
- **Relés SSR** para o controle do cooler e da bomba
- **Buzzer** para alertas sonoros

---

## Dependências

Este projeto utiliza as seguintes bibliotecas do ESP-IDF:

- **FreeRTOS:** para gerenciamento de tarefas.
- **Driver GPIO e I2C:** para comunicação com os periféricos.
- **Bibliotecas específicas para os dispositivos:**  
  - `mpu6050.h` para o sensor MPU6050.  
  - `ssd1306.h` para o display OLED.  
  - `ds18b20.h` para o sensor de temperatura DS18B20.  
  - `ssr.h` para controle dos relés SSR.

Certifique-se de que todas as bibliotecas estejam instaladas e configuradas corretamente no seu ambiente de desenvolvimento ESP-IDF.

---

## Configuração de Hardware

- **I2C (para MPU6050 e SSD1306):**  
  - **SCL:** GPIO 22  
  - **SDA:** GPIO 21  
  - **Frequência:** 400kHz  

- **Sensor DS18B20:**  
  - Conectado ao GPIO 16

- **Relés SSR:**  
  - **Cooler:** GPIO 17  
  - **Bomba:** GPIO 18

- **Buzzer:**  
  - Conectado ao GPIO 14

Verifique as conexões físicas e certifique-se de que os dispositivos estão conectados aos pinos corretos conforme definido no código.

---

## Configuração do Ambiente

1. **Instale o ESP-IDF:**  
   Siga as instruções oficiais de instalação e configuração do [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

2. **Clone o repositório:**  
   ```bash
   git clone <https://github.com/DemetrioGurgel/Sistemas_embarcados.git>
   cd <Sistemas_embarcados/projeto_aquario>

3. **Configure o projeto:**
    idf.py menuconfig

4. **Compile o projeto:**
    idf.py build

5. **Flash no dispositivo:**
    idf.py -p <PORTA_SERIAL> flash monitor

---

## Estrutura do Código

- **`app_main`:**  
  Função principal responsável por inicializar todos os dispositivos, incluindo o display OLED, sensores (MPU6050, DS18B20), relés SSR (para o cooler e a bomba) e o buzzer. Também é responsável por configurar o hardware e criar a tarefa de monitoramento.

- **`sensor_task`:**  
  Tarefa que realiza a leitura periódica dos sensores. Ela lê os dados do MPU6050 para obter a aceleração e converte essa leitura em nível de água (litros) por meio da função `calcular_nivel_agua`. Em seguida, lê a temperatura do sensor DS18B20 e, com base nessas informações, atualiza o display OLED e controla os dispositivos (cooler e bomba).

- **`calcular_nivel_agua`:**  
  Função que utiliza uma equação cúbica para converter o valor da aceleração do eixo Y (obtido pelo MPU6050) em litros, garantindo que o valor não seja negativo.

- **`atualizar_display`:**  
  Função responsável por limpar e atualizar o display OLED com as informações do nível de água e temperatura. Se a temperatura estiver alta (≥ 30°C), exibe uma mensagem de alerta.

- **`beep_buzzer`:**  
  Função que gera três sinais sonoros curtos utilizando o buzzer para alertar sobre condições críticas, como alta temperatura.

---

## Funcionamento

1. **Inicialização:**  
   Ao iniciar, o ESP32 configura todos os periféricos:
   - O display OLED é inicializado e exibe a mensagem "Sistema Iniciado!".
   - Os sensores MPU6050 e DS18B20 são configurados e iniciados.
   - Os relés SSR responsáveis pelo controle do cooler e da bomba são inicializados em estado desligado (nível lógico alto).
   - O buzzer é configurado para começar desligado.

2. **Monitoramento Periódico:**  
   A tarefa `sensor_task` é criada e executada periodicamente (a cada 5 segundos):
   - **Leitura dos Sensores:**  
     Os dados do MPU6050 são lidos e os valores de aceleração são convertidos para determinar o nível de água. Em paralelo, a temperatura é lida pelo DS18B20.
     
   - **Atualização do Display:**  
     As leituras de nível de água e temperatura são enviadas para o display OLED. Se a temperatura for igual ou superior a 30°C, o display também mostra um alerta.

   - **Controle do Cooler:**  
     Caso a temperatura seja alta (≥ 30°C), o cooler é ligado (ativado pelo SSR em nível baixo) e o buzzer emite três beeps para alertar. Se a temperatura for inferior, o cooler permanece desligado.

   - **Controle da Bomba:**  
     A bomba é ligada se o nível de água estiver abaixo de 0.50L e desligada quando o nível atingir ou ultrapassar 1L. Esse controle evita o funcionamento desnecessário da bomba.

3. **Registro de Logs:**  
   Durante todo o processo, mensagens de log são geradas para monitorar o estado dos sensores e dispositivos, auxiliando no debug e na manutenção do sistema.




