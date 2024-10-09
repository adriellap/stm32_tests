#include "stm32f1xx.h"
#include <Arduino.h>  // Inclui para habilitar a interface USB Serial

// void delay_ms(uint32_t ms);
// void LED_Init(void);
// void LED_Toggle(void);

// void setup() {
//     // Inicializa o USB Serial para comunicação com o PC
//     Serial.begin(115200);

//     // Inicializa o LED no pino PC13
//     LED_Init();

//     // Mensagem inicial tanto no Serial (USB) quanto na UART
//     Serial.println("hello world via USB Serial");
// }

// void loop() {
//     // Envia uma mensagem pela USB Serial e pela UART1
//     Serial.println("hello world via USB Serial");

//     // Pisca o LED (para debug)
//     LED_Toggle();

//     // Atraso de 2 segundos
//     delay_ms(2000);
// }

// void delay_ms(uint32_t ms) {
//     // Função de delay simples (não precisa de alta precisão)
//     for (uint32_t i = 0; i < ms * 8000; i++) {
//         __NOP();  // Instrução de no-operation (NOP)
//     }
// }

// // Inicializa o LED no pino PC13
// void LED_Init(void) {
//     // Habilita o clock para o GPIOC
//     RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

//     // Configura PC13 como saída push-pull
//     GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Limpa bits
//     GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Configura PC13 como saída de baixa velocidade (2 MHz)
// }

// // Alterna o estado do LED (PC13)
// void LED_Toggle(void) {
//     GPIOC->ODR ^= GPIO_ODR_ODR13;  // Inverte o valor do pino PC13
// }
               // Soma das temperaturas no buffer


#define BUFFER_SIZE 10  // Tamanho do buffer para a média móvel

int32_t tempBuffer[BUFFER_SIZE];  // Buffer para armazenar as leituras de temperatura
uint8_t bufferIndex = 0;          // Índice do buffer
int32_t sum = 0;   
int32_t minTemperature = 2000;  // Exemplo: -40°C (em décimos de grau)
int32_t maxTemperature = 3000;  // Exemplo: 125°C (em décimos de grau)

uint16_t analogReadRegister() {
    ADC1->CR2 |= ADC_CR2_ADON;      // Iniciar conversão
    while (!(ADC1->SR & ADC_SR_EOC));  // Aguardar o fim da conversão
    return ADC1->DR;                // Retornar o valor lido
}

// Função otimizada para converter a leitura ADC diretamente em temperatura
int32_t adcToTemperature(uint16_t adcValue) {
    // Fórmula simplificada: (V_adc - 0.76) / 0.0025 + 25
    // Vamos trabalhar diretamente em mV e evitar ponto flutuante:
    // Vadc (mV) = (adcValue * 3300) / 4095
    int32_t Vadc = (adcValue * 3300) / 4095;

    // Cálculo direto da temperatura sem ponto flutuante
    int32_t temperature = ((Vadc - 760) * 100) / 25 + 2500; // A temperatura está 100x maior (2500 = 25°C)
    
    return temperature;
}

// Função para calcular a média móvel
int32_t calculateMovingAverage(int32_t newTemperature) {
    // Subtrair o valor mais antigo do somatório
    sum -= tempBuffer[bufferIndex];
    
    // Atualizar o buffer com a nova leitura
    tempBuffer[bufferIndex] = newTemperature;
    
    // Adicionar a nova leitura ao somatório
    sum += newTemperature;

    // Atualizar o índice do buffer (circular)
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Retornar a média
    return sum / BUFFER_SIZE;
}

// Função para mapear temperatura para o duty cycle
uint8_t mapTemperatureToDutyCycle(int32_t temperature) {
    if (temperature <= minTemperature) return 0;
    if (temperature >= maxTemperature) return 100;

    return (temperature - minTemperature) * 100 / (maxTemperature - minTemperature);
}

void setup() {
    Serial.begin(9600);

    // Habilitar o clock do ADC1 e o sensor de temperatura
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;

    // Habilitar o sensor de temperatura e configurar o ADC
    ADC1->CR2 |= ADC_CR2_TSVREFE;   // Habilita o sensor de temperatura interno
    ADC1->CR2 |= ADC_CR2_ADON;      // Ligar o ADC1
    ADC1->CR2 |= ADC_CR2_CONT;      // Modo contínuo

    // Configurar o tempo de amostragem do canal 16 (sensor de temperatura)
    ADC1->SMPR1 |= ADC_SMPR1_SMP16_2 | ADC_SMPR1_SMP16_1 | ADC_SMPR1_SMP16_0; // Amostragem de 239.5 ciclos

    // Selecionar o canal 16 (sensor de temperatura interno)
    ADC1->SQR3 = 29;                // Seleciona o canal 16 para a primeira conversão
    
    // Calibração do ADC
    ADC1->CR2 |= ADC_CR2_CAL;       // Iniciar calibração
    while (ADC1->CR2 & ADC_CR2_CAL); // Esperar até a calibração terminar

    // Inicializar o buffer de temperatura com zero
    for (int i = 0; i < BUFFER_SIZE; i++) {
        tempBuffer[i] = 0;
    }
    
    // Configurar o PWM
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Habilitar o clock do Timer1
    TIM1->PSC = 799;  // Prescaler para reduzir a frequência
    TIM1->ARR = 100;  // Definir o ARR para controle de duty cycle de 0-100%
    TIM1->CCR1 = 0;   // Duty cycle inicial 0%
    TIM1->CCER |= TIM_CCER_CC1E;  // Habilitar canal 1
    TIM1->CR1 |= TIM_CR1_CEN;     // Ligar o timer
}

void loop() {
    uint16_t adcValue = analogReadRegister();

    // Calcular a temperatura (em décimos de grau para economizar memória)
    int32_t currentTemperature = adcToTemperature(adcValue);

    // Calcular a média móvel
    int32_t averageTemperature = calculateMovingAverage(currentTemperature);

    // Mapear a temperatura para o duty cycle
    uint8_t dutyCycle = mapTemperatureToDutyCycle(averageTemperature);

    // Ajustar o duty cycle do PWM
    TIM1->CCR1 = dutyCycle;  // Ajustar duty cycle do PWM com base na temperatura

    // Exibir os resultados no monitor serial
    Serial.print("ADC Value: ");
    Serial.print(adcValue);
    Serial.print(" | Current Temperature: ");
    Serial.print(averageTemperature / 100);
    Serial.print(".");
    Serial.print(averageTemperature % 100);
    Serial.print(" °C | Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.println(" %");


    delay(1000);
}