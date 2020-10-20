#define F_CPU 1000000UL
//#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*** serial stuff ***/
// define some macros
#define BAUD 9600                                   // define baud
#define UBRR ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR

#define VOLTREF 3.3

// pin defines
#define PORT_LED PORTD
#define DDR_LED DDRD
#define PORT_PWR PORTB
#define DDR_PWR DDRB

// define Seven segment output ports for Vcc to Seven Segment
#define LED0    2
#define LED1    3
#define LED2    4

//binary representations of binary required to light seven segment 0-9
unsigned int pins[] = {
    0b00111111, //0
    0b00000110, //1
    0b01011011, //2
    0b01001111, //3
    0b01100110, //4
    0b01101101, //5
    0b01111101, //6
    0b00000111, //7
    0b01111111, //8
    0b01101111, //9
};
volatile int i_active_display = LED0;
float display_value = 0;

void write_digit(int i_digit, bool dot) {
    // write to entire LED port - invert due to the fact that in a common anode display LOW = on.
    // to use in a common cathode display, remove inversion.
    PORT_LED = pins[i_digit%10];
    if (dot){
        PORT_LED |= (1<<7);
    }   
}

void write_number(float n){
    cli();
    display_value = n;
    sei();
}

// function to initialize UART
void uart_init (void)
{
    UBRR0H = (UBRR>>8);                      // shift the register right by 8 bits
    UBRR0L = UBRR;                           // set baud rate
    //UBRR0 = UBRR;
    UCSR0B|= (1<<TXEN0)|(1<<RXEN0);                // enable receiver and transmitter
    UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

void usart_transmit(uint8_t data)
{
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

void usart_transmit_str(const char* str)
{
    int len = strlen(str);
    int i;
    for (i = 0; i < len; i++) {
        usart_transmit(str[i]);
    }
}

/*** ADC stuff ***/

void adc_init()
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);
 
    // ADC Enable and prescaler
    // channel ADC0
    // 16000000/128 = 125000
    // 1e6/8=125k
    //ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
    // select the corresponding channel 0~7
    // ANDing with ’7′ will always keep the value
    // of ‘ch’ between 0 and 7
    ch &= 0b00000111;  // AND operation with 7
    ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
    _delay_us(10);

    // start single convertion
    // write ’1′ to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes ’0′ again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADC);
}

ISR(TIMER0_OVF_vect) {
    PORT_PWR &= 0;
        //toggle active display
    if (i_active_display == LED2 )
        i_active_display = LED0;
    else
        i_active_display++;
    
    switch (i_active_display) {
        case LED0:
            write_digit((int)display_value / 10 % 10, false);
            //write_digit((int)99/10%10);
            break;
        case LED1:
            write_digit((int)display_value % 10, true);
            break;
        default:
            write_digit((int)(display_value * 10) % 10, false);
            break;
    }
    PORT_PWR = (1 << i_active_display);
}

int main(){
    uint16_t adcVal = 0;
    float volt = 0;
    float temp = 0;
    //char buf[512];

    cli();
    //uart_init();
    DDR_LED = 0xFF;
    DDR_PWR = 0b00011100;

    TIMSK0 |= (1 << TOIE0);  // timer 0 overflow interupt
    
    sei();
    
    TCCR0B |= (1 << CS01); //timer 0 at clk/8
    //TCCR0B |= (1 << CS01) | (1 << CS00); //timer at clock/64
    //TCCR1B |= (1 << CS12);

    adc_init();
    //PORT_PWR = 0b00011100;

    while(1){
        adcVal = adc_read(0);
        volt = adcVal * (3.3 / 1023);
        temp = (volt - 0.5) * 100;
        write_number(temp);
        /*
        usart_transmit_str("-------------\r\n");
        memset(buf, 0, 512);
        sprintf(buf, "ADC: %hu, mV: %f, temp: %fC\r\n", adcVal, volt, temp);
        usart_transmit_str(buf);
        */
        _delay_ms(1000);
    }
    return 0;
}
