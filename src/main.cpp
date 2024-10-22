/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 UNIDADE 2
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #include "stm32f1xx.h"
// #include <Arduino.h>  // Inclui para habilitar a interface USB Serial

// #define QTD_AMOSTRAS 10  // Tamanho do buffer para a média móvel

// // Variáveis globais
// int32_t Buffer[QTD_AMOSTRAS];  // Buffer para armazenar as leituras de temperatura
// uint8_t Contador = 0;          // Índice do buffer
// int32_t Somador = 0;   
// int32_t minTemperatura = -40000;  // Exemplo: -40°C (em décimos de grau)
// int32_t maxTemperatura = 125000;  // Exemplo: 125°C (em décimos de grau)
// volatile bool flagAtualizar = false; // Flag para indicar quando atualizar

// // Função de configuração geral
// void setup_perifericos() {
//     Serial.begin(9600);

//     // Habilitar clocks do GPIOA e TIM1
//     RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;

//     // Configurar PA8 como saída alternativa push-pull
//     GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
//     GPIOA->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;  // 50 MHz, saída alternativa push-pull
//     GPIOA->CRH |= GPIO_CRH_CNF8_1;  // Modo de saída alternativa push-pull
// }

// // Função para configurar o PWM
// void configurar_pwm() {
//     // Configurar o Timer 1 para gerar PWM
//     TIM1->PSC = 72 - 1;  // Prescaler para 1 MHz
//     TIM1->ARR = 1000 - 1;  // Período do PWM (frequência de 1 kHz)
//     TIM1->CCR1 = 0;   // Duty cycle inicial de 0%
//     TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // PWM mode 1
//     TIM1->CCER |= TIM_CCER_CC1E;  // Habilitar o canal 1
//     TIM1->BDTR |= TIM_BDTR_MOE;  // Habilitar a saída principal
//     TIM1->CR1 |= TIM_CR1_CEN;     // Ligar o temporizador
// }

// // Função para configurar o ADC
// void configurar_adc() {
//     ADC1->CR2 |= ADC_CR2_TSVREFE | ADC_CR2_ADON | ADC_CR2_CONT;
//     ADC1->SMPR1 |= ADC_SMPR1_SMP16;  // Configurar o tempo de amostragem
//     ADC1->SQR3 = 16;  // Canal 16 (sensor de temperatura interno)
// }

// // Função para inicializar o buffer de temperatura
// void inicializar_buffer() {
//     for (int i = 0; i < QTD_AMOSTRAS; i++) {
//         Buffer[i] = 0;
//     }
// }

// // Função para ler a entrada analógica e retornar o valor do registrador ADC
// uint16_t leitor_analogico_resgistradores() {
//     ADC1->CR2 |= ADC_CR2_ADON;      // Iniciar conversão
//     while (!(ADC1->SR & ADC_SR_EOC));  // Aguardar o fim da conversão
//     return ADC1->DR;                // Retornar o valor lido
// }

// // Função para converter o valor ADC em temperatura
// int32_t senso_interno(uint16_t tensao) {
//     int32_t Vadc = (tensao * 3300) / 4095;
//     int32_t temperature = ((Vadc - 760) * 100) / 25 + 2500;
//     return temperature;
// }

// // Função para calcular a média móvel
// int32_t media_movel(int32_t novaTemperatura) {
//     Somador -= Buffer[Contador];
//     Buffer[Contador] = novaTemperatura;
//     Somador += novaTemperatura;
//     Contador = (Contador + 1) % QTD_AMOSTRAS;
//     return Somador / QTD_AMOSTRAS;
// }

// // Função para mapear temperatura para o duty cycle (0-100%)
// uint8_t calcula_dutycicle(int32_t temperatura) {
//     if (temperatura <= minTemperatura) return 0;
//     if (temperatura >= maxTemperatura) return 100;
//     return (temperatura - minTemperatura) * 100 / (maxTemperatura - minTemperatura);
// }

// // Função para configurar o Timer com base em um intervalo em milissegundos
// void configurarTimer(int intervalo_ms) {
//     RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Habilitar clock do Timer 3
//     TIM3->PSC = 7200 - 1;  // Prescaler para 10 kHz (72 MHz / 7200 = 10 kHz)
//     TIM3->ARR = (intervalo_ms * 10) - 1;  // Calcular o valor de ARR para o intervalo em ms
//     TIM3->DIER |= TIM_DIER_UIE;  // Habilitar interrupção de atualização
//     TIM3->CR1 |= TIM_CR1_CEN;  // Ligar Timer 3
//     NVIC_EnableIRQ(TIM3_IRQn);  // Habilitar interrupção no NVIC
// }

// // Função para ser chamada pela interrupção do Timer 3
// extern "C" void TIM3_IRQHandler(void) {
//     if (TIM3->SR & TIM_SR_UIF) {  // Se a interrupção foi gerada por atualização
//         TIM3->SR &= ~TIM_SR_UIF;  // Limpar flag de atualização
//         flagAtualizar = true;  // Sinalizar que é hora de atualizar os dados
//     }
// }

// // Função para processar os dados
// void processar_dados() {
//     uint16_t tensao = leitor_analogico_resgistradores();
//     int32_t temperatura = senso_interno(tensao);
//     int32_t temperatura_media = media_movel(temperatura);
//     uint8_t dutyCycle = calcula_dutycicle(temperatura_media);

//     // Ajustar o duty cycle do PWM
//     TIM1->CCR1 = dutyCycle * 10;  // Ajustar duty cycle do PWM com base na temperatura

//     // Exibir os resultados no monitor serial
//     Serial.print(tensao);
//     Serial.print(" mV | ");
//     Serial.print(temperatura_media / 100);
//     Serial.print(".");
//     Serial.print(temperatura_media % 100);
//     Serial.print(" °C | ");
//     Serial.print(dutyCycle);
//     Serial.println("%");
// }

// void setup() {
//     setup_perifericos();
//     configurar_pwm();
//     configurar_adc();
//     inicializar_buffer();

//     // Configurar o Timer 3 com intervalo desejado (por exemplo, 1000 ms = 1 segundo)
//     configurarTimer(1000);  // Intervalo de 1 segundo
// }

// void loop() {
//     if (flagAtualizar) {
//         flagAtualizar = false;  // Resetar a flag
//         processar_dados();
//     }
// }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 UNIDADE 3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stm32f1xx.h"
#include <Arduino.h>  // Inclui para habilitar a interface USB Serial

#define QTD_AMOSTRAS 10  // Tamanho do buffer para a média móvel
#define TEMP_ALVO 2000   // Temperatura alvo em décimos de grau (30,00°C)

// Variáveis globais
int32_t Buffer[QTD_AMOSTRAS];  // Buffer para armazenar as leituras de temperatura
uint8_t Contador = 0;          // Índice do buffer
int32_t Somador = 0;   
volatile bool flagAtualizar = false; // Flag para indicar quando atualizar

// Função de configuração geral
void setup_perifericos() {
    Serial.begin(9600);

    // Habilitar clocks do GPIOA e TIM1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;

    // Configurar PA8 como saída alternativa push-pull
    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;  // 50 MHz, saída alternativa push-pull
    GPIOA->CRH |= GPIO_CRH_CNF8_1;  // Modo de saída alternativa push-pull
}

// Função para configurar o PWM no pino PA8
void configurar_pwm() {
    // Configurar o Timer 1 para gerar PWM
    TIM1->PSC = 72 - 1;  // Prescaler para 1 MHz
    TIM1->ARR = 1000 - 1;  // Período do PWM (frequência de 1 kHz)
    TIM1->CCR1 = 0;   // Duty cycle inicial de 0%
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // PWM mode 1
    TIM1->CCER |= TIM_CCER_CC1E;  // Habilitar o canal 1
    TIM1->BDTR |= TIM_BDTR_MOE;  // Habilitar a saída principal
    TIM1->CR1 |= TIM_CR1_CEN;     // Ligar o temporizador
}

// Função para configurar o ADC
void configurar_adc() {
    // Desabilitar o sensor de temperatura interno e configurar para o canal 3 (PA3)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);  // PA3 como entrada analógica

    ADC1->SQR3 = 3;  // Configurar o ADC para o canal 3 (PA3)
    ADC1->SMPR2 |= ADC_SMPR2_SMP3;  // Definir o tempo de amostragem para o canal 3
    ADC1->CR2 |= ADC_CR2_ADON;  // Ligar o ADC
    delay(1);  // Pequeno delay para garantir a ativação
    ADC1->CR2 |= ADC_CR2_CAL;  // Iniciar a calibração do ADC
    while (ADC1->CR2 & ADC_CR2_CAL);  // Esperar até a calibração terminar
}

// Função para inicializar o buffer de temperatura
void inicializar_buffer() {
    for (int i = 0; i < QTD_AMOSTRAS; i++) {
        Buffer[i] = 0;
    }
}

// Função para ler a entrada analógica e retornar o valor do registrador ADC
uint16_t leitor_analogico_resgistradores() {
    ADC1->CR2 |= ADC_CR2_ADON;      // Iniciar conversão
    while (!(ADC1->SR & ADC_SR_EOC));  // Aguardar o fim da conversão
    return ADC1->DR;                // Retornar o valor lido
}

// Função para converter o valor ADC em temperatura com base no LM35
int32_t calcularTemperaturaLM35(uint16_t tensaoADC) {
    // A tensão de saída do LM35 é 10mV/°C, e o ADC é de 12 bits (0-4096) com referência de 3,18V
    // Portanto, para cada valor do ADC temos (3180 mV / 4096) por ponto do ADC.
    // Multiplicamos por 100 para termos o valor em décimos de grau Celsius
    return (tensaoADC * 318 * 100) / 4096;  // Ajustado para Vref = 3,18V
}

// Função para calcular a média móvel
int32_t media_movel(int32_t novaTemperatura) {
    Somador -= Buffer[Contador];
    Buffer[Contador] = novaTemperatura;
    Somador += novaTemperatura;
    Contador = (Contador + 1) % QTD_AMOSTRAS;
    return Somador / QTD_AMOSTRAS;
}

// Função para mapear a diferença de temperatura para o duty cycle
uint8_t calcula_dutyCycle(int32_t temperatura_media) {
    // Se a temperatura estiver abaixo da temperatura alvo, o cooler deve estar desligado (duty cycle = 0)
    if (temperatura_media <= TEMP_ALVO) {
        return 0;
    }
    
    // Caso contrário, o duty cycle aumenta proporcionalmente à diferença de temperatura
    int32_t diferenca = temperatura_media - TEMP_ALVO;
    int32_t dutyCycle = (diferenca * 100) / TEMP_ALVO;  // Mapeia proporcionalmente a diferença de temperatura para o cooler
    if (dutyCycle > 100) dutyCycle = 100;  // Limitar duty cycle a 100%
    return dutyCycle;
}

// Função para configurar o Timer com base em um intervalo em milissegundos
void configurarTimer(int intervalo_ms) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Habilitar clock do Timer 3
    TIM3->PSC = 7200 - 1;  // Prescaler para 10 kHz (72 MHz / 7200 = 10 kHz)
    TIM3->ARR = (intervalo_ms * 10) - 1;  // Calcular o valor de ARR para o intervalo em ms
    TIM3->DIER |= TIM_DIER_UIE;  // Habilitar interrupção de atualização
    TIM3->CR1 |= TIM_CR1_CEN;  // Ligar Timer 3
    NVIC_EnableIRQ(TIM3_IRQn);  // Habilitar interrupção no NVIC
}

// Função para ser chamada pela interrupção do Timer 3
extern "C" void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {  // Se a interrupção foi gerada por atualização
        TIM3->SR &= ~TIM_SR_UIF;  // Limpar flag de atualização
        flagAtualizar = true;  // Sinalizar que é hora de atualizar os dados
    }
}

// Função para processar os dados
void processar_dados() {
    uint16_t tensao = leitor_analogico_resgistradores();
    int32_t temperatura = calcularTemperaturaLM35(tensao);
    int32_t temperatura_media = media_movel(temperatura);
    uint8_t dutyCycle = calcula_dutyCycle(temperatura_media);

    // Ajustar o duty cycle do PWM para controlar o cooler
    TIM1->CCR1 = dutyCycle * 10;  // Ajustar duty cycle do PWM com base na temperatura

    // Exibir os resultados no monitor serial
    Serial.print(tensao);
    Serial.print(" mV | ");
    Serial.print(temperatura_media / 100);
    Serial.print(".");
    Serial.print(temperatura_media % 100);
    Serial.print(" °C | Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.println("%");
}

void setup() {
    setup_perifericos();
    configurar_pwm();
    configurar_adc();
    inicializar_buffer();

    // Configurar o Timer 3 com intervalo desejado (por exemplo, 1000 ms = 1 segundo)
    configurarTimer(1000);  // Intervalo de 1 segundo
}

void loop() {
    if (flagAtualizar) {
        flagAtualizar = false;  // Resetar a flag
        processar_dados();
    }
}