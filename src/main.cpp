#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void usart_setup(void) {
    // Habilitar o clock para GPIOA e USART1
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // Configurar os pinos PA9 (TX) e PA10 (RX)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO10);

    // Configurar a USART1
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    // Habilitar a USART1
    usart_enable(USART1);
}

void usart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str++);
    }
}

int main(void) {
    // Configurar a USART
    usart_setup();

    // Enviar a mensagem "hello world"
    while (1) {
        // Enviar "vitor é foda" a cada 2 segundos
        usart_send_string("hello world\n");
        for (int i = 0; i < 8000000; i++) { // Aproximadamente 2 segundos de delay
            __asm__("nop");
        }
    }

    return 0;
}