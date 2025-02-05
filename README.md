# Projeto de Desenvolvimento de Bibliotecas para Dispositivos Embarcados

## Objetivo
Esta atividade tem como objetivo principal o desenvolvimento de bibliotecas para dispositivos embarcados e a aplicação prática dessas bibliotecas em um projeto final. Os alunos deverão implementar bibliotecas para o módulo MPU6050 (I2C), o display SSD1306 (SPI e I2C), o sensor de temperatura DS18B20 e o módulo SSR (Relé de Estado Sólido). Na segunda etapa, os alunos irão propor e desenvolver um dispositivo completo que utilize as bibliotecas desenvolvidas.

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
