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
#include <avr/eeprom.h>
#include <math.h>

#define LED PB7

#define Step_orotatieX 200
//#define Step_second_X 100
#define X_EN   PD7
#define X_STEP PF0
#define X_DIR  PF1
#define MS1_X 0
#define MS2_X 0
#define MS3_X 0
#if (MS1_X == 1 && MS2_X == 1 && MS3_X == 1)
	#define Microstepping_X 16
#elif (MS1_X == 1 && MS2_X == 1 && MS3_X == 0)
	#define Microstepping_X 8
#elif (MS1_X == 0 && MS2_X == 1 && MS3_X == 0)
	#define Microstepping_X 4
#elif (MS1_X == 1 && MS2_X == 0 && MS3_X == 0)
	#define Microstepping_X 2
#else
	#define Microstepping_X 1
#endif
//#define Steps_X Step_second_X*Microstepping_X

#define Step_orotatieY 200
//#define Step_second_Y 100
#define Y_EN   PF2
#define Y_STEP PF6
#define Y_DIR  PF7
#define MS1_Y 0
#define MS2_Y 0
#define MS3_Y 0
#if (MS1_Y == 1 && MS2_Y == 1 && MS3_Y == 1)
	#define Microstepping_Y 16
#elif (MS1_Y == 1 && MS2_Y == 1 && MS3_Y == 0)
	#define Microstepping_Y 8
#elif (MS1_Y == 0 && MS2_Y == 1 && MS3_Y == 0)
	#define Microstepping_Y 4
#elif (MS1_Y == 1 && MS2_Y == 0 && MS3_Y == 0)
	#define Microstepping_Y 2
#else
	#define Microstepping_Y 1
#endif
//#define Steps_Y Step_second_Y*Microstepping_Y

#define RightX 0
#define LeftX 1
unsigned char directia=0;
#define RightY 0
#define LeftY 1

uint8_t V1 EEMEM;
uint8_t V2 EEMEM;
uint8_t V3 EEMEM;
uint8_t V4 EEMEM;
uint8_t V5 EEMEM;
uint8_t V6 EEMEM;
uint8_t V7 EEMEM;
uint8_t V8 EEMEM;
uint8_t V9 EEMEM;
uint8_t V10 EEMEM;
uint8_t V11 EEMEM;
uint8_t V12 EEMEM;
uint8_t V13 EEMEM;
uint8_t V14 EEMEM;
uint8_t V15 EEMEM;
uint8_t date=0;

volatile char Receive_buf[256];
char Transmite_buf[256];								
volatile uint8_t Receive_W = 0, Receive_C = 0;						
uint8_t Receive_R = 0, Transmite_T = 0;
volatile unsigned char fanion_asteapta_raspuns=0;

volatile uint8_t fanion_x=0, fanion_y=0;
volatile unsigned char Overflow_timer11=0, Overflow_timer12=0, Overflow_timer31=0, Overflow_timer32=0;
long int Valoarea_de_numarareX=0, Valoarea_de_numarareY=0;

volatile unsigned long int Pasi_decrement_X=0, Pasi_decrement_Y=0;
volatile unsigned char Fanion_RunStep_X=0, Fanion_RunStep_Y=0;

unsigned long int Nr_impulsuriX=0, Nr_impulsuriY=0;

unsigned long int Lungimea_bobinei=0, Nr_spire=0;
double Diametrul_sarmei=0, Coef_deplasare=0, Viteza=0;
unsigned long int Nr_fire_pestrat=0, Nr_straturi=0;

unsigned long int Steps_X=0, Steps_Y=0;	//nr de pasi pe secunda, viteza

void Start_timer0()
{
	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 15.625 kHz
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	// Timer Period: 16.384 ms
	TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	TCNT0=0x00;
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
}

void Stop_timer0()
{
	// Timer/Counter 0 stop
	TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
}

void Off_timer1()
{
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 2000.000 kHz
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// OC1C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 32.768 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
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
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
}

void Set_timer1()
{
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 2000.000 kHz
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// OC1C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 32.768 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	OCR1CH=0x00;
	OCR1CL=0x00;
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
}

void Off_timer3()
{
	// Timer/Counter 3 initialization
	// Clock source: System Clock
	// Clock value: 2000.000 kHz
	// Mode: Normal top=0xFFFF
	// OC3A output: Disconnected
	// OC3B output: Disconnected
	// OC3C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 32.768 ms
	// Timer3 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
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
	
	// Timer/Counter 3 Interrupt(s) initialization
	TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);	
}

void Set_timer3()
{
	// Timer/Counter 3 initialization
	// Clock source: System Clock
	// Clock value: 2000.000 kHz
	// Mode: Normal top=0xFFFF
	// OC3A output: Disconnected
	// OC3B output: Disconnected
	// OC3C output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 32.768 ms
	// Timer3 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
	TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (1<<CS31) | (0<<CS30);
	TCNT3H=0x00;
	TCNT3L=0x00;
	ICR3H=0x00;
	ICR3L=0x00;
	OCR3BH=0x00;
	OCR3BL=0x00;
	OCR3CH=0x00;
	OCR3CL=0x00;
	
	// Timer/Counter 3 Interrupt(s) initialization
	TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);
}

void Stepper_setup_X(unsigned long int nr_impulsuri)
{
Valoarea_de_numarareX = 2000000/(nr_impulsuri*2);	//2000000 - frecventa de tactare a timerului, nu am facuto prin defined, este setata la clk/8=16000000/8=2000000;
while((Valoarea_de_numarareX-65536)>0)
	{
		Overflow_timer11++;
		Valoarea_de_numarareX -= 65536;
	}
if(Overflow_timer11==0)
	OCR1A = Valoarea_de_numarareX;
else
	OCR1A = 0xFFFF;	
Overflow_timer12=Overflow_timer11;	
Set_timer1();	
}

void Stepper_setup_Y(unsigned long int nr_impulsuri)
{
Valoarea_de_numarareY = 2000000/(nr_impulsuri*2);	//2000000 - frecventa de tactare a timerului, nu am facuto prin defined, este setata la clk/8=16000000/8=2000000;
while((Valoarea_de_numarareY-65536)>0)
	{
		Overflow_timer31++;
		Valoarea_de_numarareY -= 65536;
	}
if(Overflow_timer31==0)
	OCR3A = Valoarea_de_numarareY;
else
	OCR3A = 0xFFFF;
Overflow_timer32=Overflow_timer31;
Set_timer3();	
}

void Enable_stepper(unsigned char Motor_Enable)
{
	Motor_Enable == PD7 ? PORTD &= ~(1 << Motor_Enable) : PORTF &= ~(1 << Motor_Enable);
	//_delay_us(5);
}
void Disable_stepper(unsigned char Motor_Disable)
{
	Motor_Disable == PD7 ? PORTD |= (1 << Motor_Disable) : PORTF |= (1 << Motor_Disable);
	//_delay_us(5);
}
void Setdir_stepper(unsigned char Motor, unsigned char Direction)
{
	Direction > 0 ? PORTF |= (1 << Motor) : PORTF &= ~(1 << Motor);			// scriem pe axa X dar poate fi utilizata si pe axa y deoarece directia pinilor la ambele se afla pe un port portul F
	//_delay_us(5);
}
void RunSpeed(unsigned char Motor, unsigned char Direction)
{
	Motor == PF0 ? Setdir_stepper(X_DIR, Direction) : Setdir_stepper(Y_DIR, Direction); 
	Motor == PF0 ? Stepper_setup_X(Steps_X) : Stepper_setup_Y(Steps_Y);
}
void RunStep(unsigned char Motor, unsigned char Direction, unsigned long int Steps)
{
	RunSpeed(Motor, Direction);
	Motor == PF0 ? Pasi_decrement_X=Steps : Pasi_decrement_Y=Steps;
	Motor == PF0 ? Fanion_RunStep_X=1 : Fanion_RunStep_Y=1;
}


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

void Parametri_de_intrare()
{
		sprintf(Transmite_buf, "Helllo\n\n");
		USART_putstring_hardware(Transmite_buf);
		PORTB ^= 1<<LED;
		
		unsigned char flag=0, contor_cifre=0;
		unsigned long int Diametrul_sarmei_parteaintreaga=0;
		sprintf(Transmite_buf, "Diametrul sarmei in mm? \n");
		USART_putstring_hardware(Transmite_buf);
		while(fanion_asteapta_raspuns==0)
			{
				while(Receive_C>0)
					{
						if(Receive_buf[Receive_R] == '.')
							flag=1;
					
						if(flag==0)
							Diametrul_sarmei_parteaintreaga = ((Diametrul_sarmei_parteaintreaga*10) + (Receive_buf[Receive_R]-0x30));
					
						if(Receive_buf[Receive_R] != '.' && flag==1)
							{
								Diametrul_sarmei = ((Diametrul_sarmei*10) + (Receive_buf[Receive_R]-0x30));
								contor_cifre++;
							}
						Receive_R++;
						Receive_C--;
					}
			}
		while(contor_cifre>0)
			{
				Diametrul_sarmei = Diametrul_sarmei/10;
				contor_cifre--;	
			}
		Diametrul_sarmei = Diametrul_sarmei_parteaintreaga + Diametrul_sarmei;
		fanion_asteapta_raspuns=0;
		sprintf(Transmite_buf, "Diametrul sarmei este %.6f mm \n", Diametrul_sarmei);
		USART_putstring_hardware(Transmite_buf);
		
		sprintf(Transmite_buf, "Lungimea bobinei mm?\n");
		USART_putstring_hardware(Transmite_buf);
		while(fanion_asteapta_raspuns==0)
			{
				while(Receive_C>0)
				{
					Lungimea_bobinei = ((Lungimea_bobinei*10) + (Receive_buf[Receive_R]-0x30));
					Receive_R++;
					Receive_C--;
				}
			}
		fanion_asteapta_raspuns=0;
		sprintf(Transmite_buf, "Lungimea bobinei este %lu mm \n", Lungimea_bobinei);
		USART_putstring_hardware(Transmite_buf);
		
		sprintf(Transmite_buf, "Numarul de spire?\n");
		USART_putstring_hardware(Transmite_buf);
		while(fanion_asteapta_raspuns==0)
			{
				while(Receive_C>0)
					{
						Nr_spire = ((Nr_spire*10) + (Receive_buf[Receive_R]-0x30));
						Receive_R++;
						Receive_C--;
					}
			}
		fanion_asteapta_raspuns=0;
		sprintf(Transmite_buf, "Numarul de spire este %lu \n", Nr_spire);
		USART_putstring_hardware(Transmite_buf);
		
		flag=0;
		contor_cifre=0;
		unsigned long int Coef_deplasare_parteaintreaga=0;
		sprintf(Transmite_buf, "Coeficientul de deplasare?\n");
		USART_putstring_hardware(Transmite_buf);
		while(fanion_asteapta_raspuns==0)
				{
					while(Receive_C>0)
						{
							if(Receive_buf[Receive_R] == '.')
								flag=1;
						
							if(flag==0)
								Coef_deplasare_parteaintreaga = ((Coef_deplasare_parteaintreaga*10) + (Receive_buf[Receive_R]-0x30));
						
							if(Receive_buf[Receive_R] != '.' && flag==1)
								{
									Coef_deplasare = ((Coef_deplasare*10) + (Receive_buf[Receive_R]-0x30));
									contor_cifre++;
								}
							Receive_R++;
							Receive_C--;
						}
				}
		while(contor_cifre>0)
			{
				Coef_deplasare = Coef_deplasare/10;
				contor_cifre--;
			}
		Coef_deplasare = Coef_deplasare_parteaintreaga + Coef_deplasare;
		fanion_asteapta_raspuns=0;
		sprintf(Transmite_buf, "Coeficientul de deplasare este %.6f \n", Coef_deplasare);
		USART_putstring_hardware(Transmite_buf);
		
		
		flag=0;
		contor_cifre=0;
		unsigned long int Viteza_parteaintreaga=0;
		sprintf(Transmite_buf, "Viteza, nr de bobinari pe secunda?\n");
		USART_putstring_hardware(Transmite_buf);
		while(fanion_asteapta_raspuns==0)
		{
			while(Receive_C>0)
				{
					if(Receive_buf[Receive_R] == '.')
						flag=1;
				
					if(flag==0)
						Viteza_parteaintreaga = ((Viteza_parteaintreaga*10) + (Receive_buf[Receive_R]-0x30));
				
					if(Receive_buf[Receive_R] != '.' && flag==1)
						{
							Viteza = ((Viteza*10) + (Receive_buf[Receive_R]-0x30));
							contor_cifre++;
						}
					Receive_R++;
					Receive_C--;
				}
		}
		while(contor_cifre>0)
		{
			Viteza = Viteza/10;
			contor_cifre--;
		}
		Viteza = Viteza_parteaintreaga + Viteza;
		fanion_asteapta_raspuns=0;
		sprintf(Transmite_buf, "Nr de bobinari pe secunda este %.6f mm\n", Viteza);
		USART_putstring_hardware(Transmite_buf);
}

int main(void)
{
    /* Replace with your application code */
	date=eeprom_read_byte(&V1);
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
	Disable_stepper(X_EN);	//pentru ca motoarele sa nu consume curent
	Disable_stepper(Y_EN);
	
	Parametri_de_intrare();
	
	Nr_fire_pestrat = ceil(Lungimea_bobinei/Diametrul_sarmei);
	unsigned long int Nr_fire_pestrat2=0;
	Nr_fire_pestrat2=Nr_fire_pestrat;
	
	Nr_straturi = ceil((Nr_spire*Diametrul_sarmei)/Lungimea_bobinei);
	Nr_impulsuriX=Step_orotatieX*Microstepping_X*2;		//inmultirea la 2 deoarece o perioada de timp impulsul este in 1 si alta perioada in 0, in taimer decrementez si la 0 si la 1, de aceea trebuie numarul de pasi si inmultim la 2 
	Nr_impulsuriY=((Step_orotatieY/Coef_deplasare))*Diametrul_sarmei*Microstepping_Y*2;
	

	Steps_X = Step_orotatieX * Microstepping_X * Viteza * 2; // inmultirea cu 2 deoarece sunt 2 motoare si se necesita timp pentru a bobina un coil apoi a misca, timpul la ambele motoare este egal, pe viitor sar putea face ca motorul Y sa se deplaseze mai rapid deoarece are putine impulsuri iar X mai lent.
	Steps_Y = Step_orotatieY * Microstepping_Y * Viteza * 2;
	
	sprintf(Transmite_buf, "Nr de fire pe strat %lu \n", Nr_fire_pestrat);
	USART_putstring_hardware(Transmite_buf);
	sprintf(Transmite_buf, "Nr de straturi %lu \n", Nr_straturi);
	USART_putstring_hardware(Transmite_buf);
	sprintf(Transmite_buf, "Nr de impulsuri pe X %lu \n", Nr_impulsuriX);
	USART_putstring_hardware(Transmite_buf);
	sprintf(Transmite_buf, "Nr de impulsuri pe Y %lu \n", Nr_impulsuriY);
	USART_putstring_hardware(Transmite_buf);
	sprintf(Transmite_buf, "Nr de impulsuri pe secunda X %lu \n", Steps_X);
	USART_putstring_hardware(Transmite_buf);
	sprintf(Transmite_buf, "Nr de impulsuri pe secunda Y %lu \n", Steps_Y);
	USART_putstring_hardware(Transmite_buf);
	
	unsigned char fanion_start=0;
	
    while (1) 
		{
			//_delay_ms(1000);
			/*while(Receive_C>0)
				{
					Transmite_buf[Transmite_T]=Receive_buf[Receive_R];
					Receive_R++;
					Receive_C--;
					Transmite_T++;
				}
			Transmite_buf[Transmite_T]=0x00;
			USART_putstring_hardware(Transmite_buf);
			Transmite_T=0;
			_delay_us(1);*/
			
					//Enable_stepper(X_EN);
					//RunSpeed(X_STEP, Left);
					//RunStep(X_STEP, RightX, Nr_impulsuriX);	//inmultirea la 2 deoarece o perioada de timp impulsul este in 1 si alta perioada in 0, in taimer decrementez si la 0 si la 1, de aceea trebuie numarul de pasi si inmultim la 2
					
					//Enable_stepper(Y_EN);
					///RunSpeed(X_STEP, Left);
				//	RunStep(Y_STEP, RightX, Nr_impulsuriY);
			
			if(fanion_start==0)	
			{sprintf(Transmite_buf, "Start\n\n");
			USART_putstring_hardware(Transmite_buf);
			Enable_stepper(X_EN);
			Enable_stepper(Y_EN);
			directia=RightY;
			while(Nr_straturi>0)
				{
					sprintf(Transmite_buf, "Nr_straturi = %lu!\n", Nr_straturi);
					USART_putstring_hardware(Transmite_buf);
					while(Nr_fire_pestrat2>0)
						{
							sprintf(Transmite_buf, "Nr_fire = %lu!\n", Nr_fire_pestrat2);
							USART_putstring_hardware(Transmite_buf);
							RunStep(X_STEP, RightX, Nr_impulsuriX);
							while(Fanion_RunStep_X !=0)
								{
									//_delay_us(1);
								}
							_delay_ms(1);
								
							RunStep(Y_STEP, directia, Nr_impulsuriY);
							while(Fanion_RunStep_Y !=0)
								{
									//_delay_us(1);
								}
							_delay_ms(1);
							
							Nr_fire_pestrat2--;
							//_delay_ms(10);
						}
					if(directia==RightY)
						directia=LeftY;
					else
						directia=RightY;	
					Nr_fire_pestrat2=Nr_fire_pestrat;
					Nr_straturi--;

				}
			Disable_stepper(X_EN);
			Disable_stepper(Y_EN);
			sprintf(Transmite_buf, "END !\n");
			USART_putstring_hardware(Transmite_buf);
			fanion_start=1;
			}
			_delay_ms(100);
		}
}

ISR(USART0_RX_vect)
{
Receive_buf[Receive_W] = UDR0;	
Receive_W++;							// pregatirea adresei pentru urmatorul byte ce va veni in port
Receive_C++;
Start_timer0();
}

ISR(TIMER1_COMPA_vect)
{
	if(Overflow_timer11 != 0)
		{
			Overflow_timer11--;	
		}
	if(Overflow_timer11==0)
		{	
			if(Overflow_timer12 !=0)
				{
					OCR1A = Valoarea_de_numarareX;
					fanion_x++;
					if(fanion_x==2)
						{
							fanion_x=0;
							OCR1A=0xFFFF;
							Overflow_timer11=Overflow_timer12;
							if(Fanion_RunStep_X==1)
								{
									Pasi_decrement_X--;
									if(Pasi_decrement_X==0)
										{
											Fanion_RunStep_X=0;
											Off_timer1();
											//Disable_stepper(X_EN);
										}
								}
							PORTF ^= (1 << X_STEP);
						}
				}
			else
				{
					if(Fanion_RunStep_X==1)
						{
							Pasi_decrement_X--;
							if(Pasi_decrement_X==0)
								{
									Fanion_RunStep_X=0;
									Off_timer1();
									//Disable_stepper(X_EN);
								}
						}
					PORTF ^= (1 << X_STEP);
				}
		}
	TCNT1H=0x00;
	TCNT1L=0x00;	
}

ISR(TIMER3_COMPA_vect)
{
	if(Overflow_timer31 != 0)
	{
		Overflow_timer31--;
	}
	if(Overflow_timer31==0)
	{
		if(Overflow_timer32 !=0)
		{
			OCR3A = Valoarea_de_numarareY;
			fanion_y++;
			if(fanion_y==2)
			{
				fanion_y=0;
				OCR3A=0xFFFF;
				Overflow_timer31=Overflow_timer32;
				if(Fanion_RunStep_Y==1)
				{
					Pasi_decrement_Y--;
					if(Pasi_decrement_Y==0)
					{
						Fanion_RunStep_Y=0;
						Off_timer3();
						//Disable_stepper(Y_EN);
					}
				}
				PORTF ^= (1 << Y_STEP);
			}
		}
		else
		{
			if(Fanion_RunStep_Y==1)
			{
				Pasi_decrement_Y--;
				if(Pasi_decrement_Y==0)
				{
					Fanion_RunStep_Y=0;
					Off_timer3();
					//Disable_stepper(Y_EN);
				}
			}
			PORTF ^= (1 << Y_STEP);
		}
	}
	TCNT3H=0x00;
	TCNT3L=0x00;
}

ISR(TIMER0_OVF_vect)
{
fanion_asteapta_raspuns=1;
Stop_timer0();	
}