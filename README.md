# Projeto de Desenvolvimento de Bibliotecas para Dispositivos Embarcados

## Objetivo
Esta atividade tem como objetivo principal o desenvolvimento de bibliotecas para dispositivos embarcados e a aplicação prática dessas bibliotecas em um projeto final. Deverão ser implementadas bibliotecas para o módulo MPU6050 (I2C), o display SSD1306 (SPI e I2C), o sensor de temperatura DS18B20 e o módulo SSR (Relé de Estado Sólido).

## Etapa 1: Desenvolvimento de Bibliotecas
Criar bibliotecas/componentes para os microcontroladores ESP32 em C/C++ utilizando o framework ESP-IDF para os seguintes dispositivos:

### MPU6050
A biblioteca deverá permitir a leitura dos dados do acelerômetro e giroscópio via protocolo I2C.

### SSD1306
A biblioteca deverá permitir a comunicação via SPI e I2C, com funções para exibição de texto e figuras no display OLED.

### DS18B20
A biblioteca deverá permitir a leitura de temperatura via comunicação 1-Wire.

### SSR (Relé de Estado Sólido)
A biblioteca para o SSR deverá permitir o acionamento do relé de forma precisa, oferecendo funções para controle tanto no modo ON-OFF (liga/desliga) quanto por Modulação por Largura de Pulso (PWM).


## Como Usar
1. Clone este repositório.
2. Siga as instruções de cada biblioteca para configurar e utilizar os dispositivos.
3. Integre as bibliotecas no seu projeto final conforme necessário.

# Descrição projeto de disciplina:
## Aquário Inteligente

## Introdução

Este projeto consiste no desenvolvimento de um **aquário inteligente**, um sistema automatizado para monitoramento e controle da temperatura e do nível da água. O objetivo principal é garantir um ambiente estável e seguro para os organismos aquáticos, minimizando riscos associados a variações térmicas e possíveis perdas de água.

O sistema é baseado no **ESP32**, que gerencia a comunicação entre sensores, atuadores e módulos de exibição de dados. Para o monitoramento da temperatura da água, é utilizado o sensor **DS18B20**, cuja precisão permite um controle eficiente do ambiente térmico. Caso a temperatura ultrapasse os limites predefinidos, um **relé mecânico** é acionado para controlar um cooler, além de emitir um sinal sonoro por meio de um **buzzer**, permitindo uma resposta rápida do usuário.

Além disso, a segurança é aprimorada pelo uso do sensor **MPU6050**, que combina acelerômetro e giroscópio para monitorar o nível da água. Caso ocorra um vazamento ou redução anormal no nível, o sensor detecta e aciona um **relé de estado sólido** para ligar uma bomba de água e completar o nível de água até um o nível de água seguro pré-definido.

Este projeto apresenta uma aplicação prática de sistemas embarcados e automação, sendo uma solução inovadora para a manutenção de aquários domésticos e profissionais, promovendo maior segurança e estabilidade para os organismos aquáticos.

---

## Componentes Utilizados

- **ESP32**: Microcontrolador principal responsável por gerenciar todos os sensores e atuadores.
- **DS18B20**: Sensor de temperatura que mede a temperatura da água.
- **MPU6050**: Acelerômetro e giroscópio, sensor utilizado para detectar variações no nível da água.
- **SSD1306**: Display OLED para exibição da temperatura, nível de água em litros, e alertas.
- **Relé Mecânico**: Controla o cooler para ajustar a temperatura da água.
- **SSR (Relé de Estado Sólido)**: Controla a bomba de água para manutenção do nível
- **Cooler**: Atuador que resfria a água quando a temperatura está acima do limite desejado.
- **Buzzer**: Emite alertas sonoros em caso de temperaturas críticas ou detecção de vazamento de água.

---

## Funcionalidades do Sistema

### 1. Monitoramento da Temperatura
- O sistema realiza a medição contínua da temperatura da água por meio do sensor **DS18B20**, exibindo os valores no **display OLED SSD1306**.

### 2. Controle Automático de Temperatura
- Se a temperatura ultrapassar um limite predefinido, o **SSR** aciona o **cooler** para reduzir a temperatura da água até retornar a um nível seguro.

### 3. Detecção do Nível da Água
- O **MPU6050** monitora o nível da água
 
### 4. Controle Automático de nível
- Se o nível estiver abaixo do nível pré-definido, o **SSR** aciona a bomba de água até atingir o nível adequado.

### 5. Sistema de Alertas Sonoros
  - O **buzzer** emite sinais sonoros quando:
  - A temperatura da água está fora da faixa ideal.

### 6. Interface de Usuário Intuitiva
- O **display OLED SSD1306** exibe informações em tempo real sobre a temperatura da água, nível da água em litros, e alertas sobre a temperatura acima do ideal.

---

## Integração das Bibliotecas

- **Biblioteca MPU6050**: Responsável pela leitura dos dados do acelerômetro e giroscópio via I2C.
- **Biblioteca SSD1306**: Permite a comunicação com o display OLED via I2C.
- **Biblioteca DS18B20**: Realiza a leitura da temperatura via comunicação 1-Wire.
- **Biblioteca SSR**: Controla o relé para acionamento do cooler e bomba de água.

---

## Fluxo de Funcionamento

1. **Inicialização**
   - Configura sensores, display e buzzer. Exibe mensagem de boas-vindas no **OLED**.

2. **Leitura e Exibição da Temperatura**
   - O **DS18B20** realiza medições periódicas e exibe os valores no **display OLED**.

3. **Controle Automático de Temperatura**
   - Se a temperatura ultrapassar o limite configurado, o **SSR** aciona o **cooler** e gera um alerta no display Oled.

4. **Monitoramento do Nível da Água**
   - O **MPU6050** detecta o nível da água, é feito um calculo em função cúbica de forma manual para determinar a quantidade de litros de acordo com a inclinação.

5 **Controle Automático de Nível**
  - Se o nível estiver abaixo do nível pré-definido, o **SSR** aciona a bomba de água até atingir o nível adequado que deve ser definido.

6. **Execução Contínua**
   - O sistema opera em **loop contínuo**, garantindo monitoramento constante.

---

## Documentação das Bibliotecas

Cada biblioteca desenvolvida conta com uma documentação detalhada:
- **Descrição das Funções**: Explicação das funcionalidades, incluindo parâmetros e aplicações.
- **Configuração**: Orientações para instalação e inicialização dos sensores e atuadores.

---

## Conclusão

Este projeto exemplifica a integração eficiente de sensores e atuadores no desenvolvimento de um **sistema embarcado inteligente**.
O código-fonte e a documentação técnica estão disponíveis neste repositório GitHub, permitindo fácil acesso e contribuições futuras.

