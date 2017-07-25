/*
 * Winder.cpp
 *
 * Created: 7/25/2017 12:47:12 PM
 * Author : CNTS04
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

char Receive_buf[256], Transmite_buf[256];								// Buferul de receptie UART software si transmitere
uint8_t Receive_W = 0, Receive_R = 0, Receive_C = 0;						// variabilele contoare pentru masivul de receptie

/************************* Nu utilizez, voi folosi UART_hardware receptie prin intrerupere *************************/
unsigned char USART_receive_hardware(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}


/************************* Transmiterea unui byte UART_hardware *************************/
void USART_send_hardware(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}


/************************* Transmiterea unui sir de caractere UART_hardware *************************/
void USART_putstring_hardware(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		USART_send_hardware(*StringPtr);
		StringPtr++;
	}
}

int main(void)
{
    /* Replace with your application code */
	
	// Input/Output Ports initialization
	// Port A initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTA=(0<<PA7) | (0<<PA6) | (0<<PA5) | (0<<PA4) | (0<<PA3) | (0<<PA2) | (0<<PA1) | (0<<PA0);

	// Port B initialization
	// Function: Bit7=Out Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRB=(1<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
	// State: Bit7=0 Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTB=(0<<PB7) | (0<<PB6) | (0<<PB5) | (0<<PB4) | (0<<PB3) | (0<<PB2) | (0<<PB1) | (0<<PB0);

	// Port C initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTC=(0<<PC7) | (0<<PC6) | (0<<PC5) | (0<<PC4) | (0<<PC3) | (0<<PC2) | (0<<PC1) | (0<<PC0);

	// Port D initialization
	// Function: Bit7=Out Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRD=(1<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
	// State: Bit7=0 Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTD=(0<<PD7) | (0<<PD6) | (0<<PD5) | (0<<PD4) | (0<<PD3) | (0<<PD2) | (0<<PD1) | (0<<PD0);

	// Port E initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In
	DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (1<<DDE1) | (0<<DDE0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=0 Bit0=T
	PORTE=(0<<PE7) | (0<<PE6) | (0<<PE5) | (0<<PE4) | (0<<PE3) | (0<<PE2) | (0<<PE1) | (0<<PE0);

	// Port F initialization
	// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=Out Bit0=Out
	DDRF=(1<<DDF7) | (1<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (1<<DDF2) | (1<<DDF1) | (1<<DDF0);
	// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=0 Bit1=0 Bit0=0
	PORTF=(0<<PF7) | (0<<PF6) | (0<<PF5) | (0<<PF4) | (0<<PF3) | (0<<PF2) | (0<<PF1) | (0<<PF0);

	// Port G initialization
	// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
	// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTG=(0<<PG5) | (0<<PG4) | (0<<PG3) | (0<<PG2) | (0<<PG1) | (0<<PG0);

	// Port H initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRH=(0<<DDH7) | (0<<DDH6) | (0<<DDH5) | (0<<DDH4) | (0<<DDH3) | (0<<DDH2) | (0<<DDH1) | (0<<DDH0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTH=(0<<PH7) | (0<<PH6) | (0<<PH5) | (0<<PH4) | (0<<PH3) | (0<<PH2) | (0<<PH1) | (0<<PH0);

	// Port J initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (0<<DDJ1) | (0<<DDJ0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTJ=(0<<PJ7) | (0<<PJ6) | (0<<PJ5) | (0<<PJ4) | (0<<PJ3) | (0<<PJ2) | (0<<PJ1) | (0<<PJ0);

	// Port K initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTK=(0<<PK7) | (0<<PK6) | (0<<PK5) | (0<<PK4) | (0<<PK3) | (0<<PK2) | (0<<PK1) | (0<<PK0);

	// Port L initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTL=(0<<PL7) | (0<<PL6) | (0<<PL5) | (0<<PL4) | (0<<PL3) | (0<<PL2) | (0<<PL1) | (0<<PL0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// OC1C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	OCR1CH=0x00;
	OCR1CL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
	TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;

	// Timer/Counter 3 initialization
	// Clock source: System Clock
	// Clock value: Timer3 Stopped
	// Mode: Normal top=0xFFFF
	// OC3A output: Disconnected
	// OC3B output: Disconnected
	// OC3C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer3 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
	TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (0<<CS30);
	TCNT3H=0x00;
	TCNT3L=0x00;
	ICR3H=0x00;
	ICR3L=0x00;
	OCR3AH=0x00;
	OCR3AL=0x00;
	OCR3BH=0x00;
	OCR3BL=0x00;
	OCR3CH=0x00;
	OCR3CL=0x00;

	// Timer/Counter 4 initialization
	// Clock source: System Clock
	// Clock value: Timer4 Stopped
	// Mode: Normal top=0xFFFF
	// OC4A output: Disconnected
	// OC4B output: Disconnected
	// OC4C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer4 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR4A=(0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) | (0<<COM4C1) | (0<<COM4C0) | (0<<WGM41) | (0<<WGM40);
	TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (0<<CS40);
	TCNT4H=0x00;
	TCNT4L=0x00;
	ICR4H=0x00;
	ICR4L=0x00;
	OCR4AH=0x00;
	OCR4AL=0x00;
	OCR4BH=0x00;
	OCR4BL=0x00;
	OCR4CH=0x00;
	OCR4CL=0x00;

	// Timer/Counter 5 initialization
	// Clock source: System Clock
	// Clock value: Timer5 Stopped
	// Mode: Normal top=0xFFFF
	// OC5A output: Disconnected
	// OC5B output: Disconnected
	// OC5C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer5 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR5A=(0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<COM5C1) | (0<<COM5C0) | (0<<WGM51) | (0<<WGM50);
	TCCR5B=(0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (0<<WGM52) | (0<<CS52) | (0<<CS51) | (0<<CS50);
	TCNT5H=0x00;
	TCNT5L=0x00;
	ICR5H=0x00;
	ICR5L=0x00;
	OCR5AH=0x00;
	OCR5AL=0x00;
	OCR5BH=0x00;
	OCR5BL=0x00;
	OCR5CH=0x00;
	OCR5CL=0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

	// Timer/Counter 3 Interrupt(s) initialization
	TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (0<<OCIE3A) | (0<<TOIE3);

	// Timer/Counter 4 Interrupt(s) initialization
	TIMSK4=(0<<ICIE4) | (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (0<<TOIE4);

	// Timer/Counter 5 Interrupt(s) initialization
	TIMSK5=(0<<ICIE5) | (0<<OCIE5C) | (0<<OCIE5B) | (0<<OCIE5A) | (0<<TOIE5);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// INT2: Off
	// INT3: Off
	// INT4: Off
	// INT5: Off
	// INT6: Off
	// INT7: Off
	EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
	EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
	// PCINT0 interrupt: Off
	// PCINT1 interrupt: Off
	// PCINT2 interrupt: Off
	// PCINT3 interrupt: Off
	// PCINT4 interrupt: Off
	// PCINT5 interrupt: Off
	// PCINT6 interrupt: Off
	// PCINT7 interrupt: Off
	// PCINT8 interrupt: Off
	// PCINT9 interrupt: Off
	// PCINT10 interrupt: Off
	// PCINT11 interrupt: Off
	// PCINT12 interrupt: Off
	// PCINT13 interrupt: Off
	// PCINT14 interrupt: Off
	// PCINT15 interrupt: Off
	// PCINT16 interrupt: Off
	// PCINT17 interrupt: Off
	// PCINT18 interrupt: Off
	// PCINT19 interrupt: Off
	// PCINT20 interrupt: Off
	// PCINT21 interrupt: Off
	// PCINT22 interrupt: Off
	// PCINT23 interrupt: Off
	PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
	PCMSK1=(0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (0<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (0<<PCINT9) | (0<<PCINT8);
	PCMSK2=(0<<PCINT23) | (0<<PCINT22) | (0<<PCINT21) | (0<<PCINT20) | (0<<PCINT19) | (0<<PCINT18) | (0<<PCINT17) | (0<<PCINT16);
	PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

	// Setarea UART la 16MHZ
	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: On
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// USART0 Baud Rate: 9600
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H=0x00;
	UBRR0L=0x67;

	// USART1 initialization
	// USART1 disabled
	UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

	// USART2 initialization
	// USART2 disabled
	UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (0<<RXEN2) | (0<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);

	// USART3 initialization
	// USART3 disabled
	UCSR3B=(0<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (0<<RXEN3) | (0<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83);

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	ADCSRB=(0<<ACME);
	// Digital input buffer on AIN0: On
	// Digital input buffer on AIN1: On
	DIDR1=(0<<AIN0D) | (0<<AIN1D);

	// ADC initialization
	// ADC disabled
	ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

	// SPI initialization
	// SPI disabled
	SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

	// TWI initialization
	// TWI disabled
	TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
	
	sei();
	
    while (1) 
		{
			_delay_ms(1000);
			sprintf(Transmite_buf, "Hello\n");
			USART_putstring_hardware(Transmite_buf);
			PORTB^=(1<<PB7);
		}
}

ISR(USART0_RX_vect)
{
USART_send_hardware(UDR0);
}