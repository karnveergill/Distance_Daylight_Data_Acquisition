#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib/timer.h" 
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "tm4c123gh6pm.h"

//Tiva -------- HC05 Bluetooth Module
//PD6   ------  TXD
//PC7   ------  RXD
//3.3v  ------  Vc
//gnd   ------  gnd
//Tiva -------- UltraSonic Sensor
//5v		------  Vcc
//PB4   ------  TRIG
//PB0   ------  ECHO
//gnd		------  gnd  
#define TRIG 0x10  //PB4
#define ECHO 0x01  //PB0
#define RED 0x02
#define BLUE 0x04 

#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

char rxChar[10];		
char TXcolor, TXdist[2], TX_day_night[2];
char* unit = "cm";    
uint8_t  timeron=60, upperThresh = 15, lowerThresh = 2, Rlow = 2, Rhigh = 50, Yhigh = 100;
uint32_t start, stop, time, distance, echoStat;
bool dataFlag=0, unitFlag=0; 
char countstr[10];
char distancestr[10]; 
volatile unsigned int index = 0;

uint16_t adcReadValue;
float voltage,resVar; 
float Threshold=0.35;


void setDelay(uint32_t time)
{
		int i; 
		for(i=0;i<time;i++);
}


void writeCharToUart3(char c)
{
	while (UART3_FR_R & UART_FR_TXFF); //wait till Transmitter is not full
	UART3_DR_R = c; //write to UART3
}


void writeStringToUart3(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
    	writeCharToUart3(str[i]);
}


uint16_t adcOutput()
{   
		uint16_t  readValue;
		ADC0_PSSI_R |= ADC_PSSI_SS3;
		while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);
		readValue=ADC0_SSFIFO3_R&0xFFF;
		return readValue;
}

 
void Day_Night_Calc(void)
{
	adcReadValue= adcOutput();
	resVar= (4096000/(adcReadValue+1))-1000;//in case adcReadValue=0, this step i.e adding 1 in denominator makes sure that resVar is still defined
	voltage=3300/(1000+resVar);


	if (voltage>Threshold) //If bright out 
	{	
		TX_day_night[0] = 'D';
		TX_day_night[1] = 'Z'; 
		UARTCharPut(UART1_BASE, TX_day_night[0]);
		UARTCharPut(UART1_BASE, TX_day_night[1]);
		GPIO_PORTF_DATA_R |=0x08;
	}
	else 
	{
		TX_day_night[0] = 'N';
		TX_day_night[1] = 'Z';
		UARTCharPut(UART1_BASE, TX_day_night[0]);
		UARTCharPut(UART1_BASE, TX_day_night[1]);
		GPIO_PORTF_DATA_R &= ~0x08;
  }
}


void Day_Night_Print(void)
{
		adcReadValue= adcOutput();
		resVar= (4096000/(adcReadValue+1))-1000;//in case adcReadValue=0, this step i.e adding 1 in denominator makes sure that resVar is still defined
		voltage=3300/(1000+resVar);


		if (voltage>Threshold) 
		{	
			writeStringToUart3("  Day"); 	
			GPIO_PORTF_DATA_R |=0x08;
		}
		else 
		{
			writeStringToUart3("  Night"); 
			GPIO_PORTF_DATA_R &= ~0x08;   
		}
}


void Uart2InterruptIsr()//this interrupt routine is for receiving data from bluetooth
{
  rxChar[index] = UARTCharGetNonBlocking(UART2_BASE);
	index++;
	if(rxChar[index-1] ==13)  			//13 = ASCII Carriage Return (i.e. Enter) 
	{
			rxChar[index-1]='\0';		 			//remove "Carriage Return" value from array for comparison 
			if(strcmp(rxChar,"unit")==0)
			{
				redLED^=1;
				unitFlag^=1; 
			}
			if(strcmp(rxChar,"Data")==0)
			{
				blueLED^=1; 	
				dataFlag ^= 1; 
			}	
		index=0;
	}
	if(unitFlag == 0) {
		//cm
		upperThresh = 15;
		lowerThresh = 2;
		Rlow = 2, Rhigh = 50, Yhigh = 100;
	}
	else {
		//inches
		upperThresh = 6;
		lowerThresh = 0; 
		Rlow = 0, Rhigh = 20, Yhigh = 40; 
	}

    UARTIntClear(UART2_BASE, UARTIntStatus(UART2_BASE, true));//clear interrupt
}


void Sonic_Trigger(void)
{
	GPIO_PORTB_DATA_R &= ~TRIG; 
	SysCtlDelay(70); 
	GPIO_PORTB_DATA_R |= TRIG; 
	SysCtlDelay(62); 
	GPIO_PORTB_DATA_R &= ~TRIG; 
}


void Dist_Calc(void)
{
		while((GPIO_PORTB_DATA_R&ECHO) == 0) {}
		start = TimerValueGet(TIMER0_BASE, TIMER_A);
		while((GPIO_PORTB_DATA_R&ECHO) == 1) {}
		stop = TimerValueGet(TIMER0_BASE, TIMER_A);
		time = start - stop; 
		if(unitFlag == 0)	{
			distance = time /1166;
		}
		else {
			distance = time /2962;
		}
		
		if(distance >=Rlow && distance <= Rhigh) TXcolor = 'R'; 
		if(distance > Rhigh && distance <= Yhigh) TXcolor = 'Y'; 
		if(distance > Yhigh) TXcolor = 'G'; 
			
		TXdist[0] = TXcolor; 
	  	TXdist[1] = 'S'; 
		UARTCharPut(UART1_BASE, TXdist[0]); 
		UARTCharPut(UART1_BASE, TXdist[1]); 
		 	
		if(distance >= lowerThresh && distance <= upperThresh) {
			GPIO_PORTF_DATA_R |= RED;
			GPIO_PORTF_DATA_R &= ~BLUE;
		}
		else {
			GPIO_PORTF_DATA_R |= BLUE;
			GPIO_PORTF_DATA_R &= ~RED; 
		}
}


void Timer0InterruptIsr(void)
{ 
	Sonic_Trigger();    //Send 10us pulse to trigger pin and trigger pulse
	Dist_Calc();  			//Calculate distance based off pulse timing 
	Day_Night_Calc(); 	//
	if(timeron>0) 
		timeron--;
	
	if(timeron==0 && dataFlag == 1)
	{
		timeron=100; //i.e every 3 sec this block will execute
		writeCharToUart3('\n'); 
		sprintf(distancestr, "%u", distance); 
		writeStringToUart3(countstr);
		writeStringToUart3("\t");
		writeStringToUart3(distancestr); 
		if(unitFlag == 0) {
			writeStringToUart3("cm"); 
		}
		else {
			writeStringToUart3("in"); 
		}
		Day_Night_Print(); 
		writeStringToUart3("\n\r"); 
	}
	
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Clear Timer Interrupt
}

void setUp(int period)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//Enabling UART1 which operates on GPIO Port B
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); 
	//Configuring GPIO Port B Pin 1  for UART1 Transmission 
	GPIOPinConfigure(GPIO_PB1_U1TX); 
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1); 
	//Configure UART1 to receive 8-data bits, 1-stop bit, No Parity bit, at 9600 baud 
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, 
											(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));	
	
	
	
	//Configure Port B Pin 0 for ECHOL (PB0)
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
	//Configure Port B Pin 4 for TRIGL (PB4)
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4); 
	GPIO_PORTB_PDR_R |= ECHO; 																//Pull down resistor to guarantee echo default is low
	
	
	
	//Configure GPIO Pins 1,2,3 on Port F 
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); 
	
	
	
	//Configure UART2 on GPIO Port D Pin 6 (PD6) 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2); 							//Enable clock for UART2 
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6); 						//Configure PD6 as UART Pin 
	GPIOPinConfigure(GPIO_PD6_U2RX);  												//Configure PD6 Rx <-- Tx on Bluetooth
	//Configure UART2 to receive 8-data bits, 1-stop bit, No Parity bit, at 9600 baud 
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600, 
												(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
	IntEnable(INT_UART2); 																		//Enable UART2 Interrupts 
	UARTIntEnable(UART2_BASE, UART_INT_RX|UART_INT_RT); 			//Enable Interrupts for Recieve and **Recieve Time-Out**
	
	
 
	//Configure UART3 on GPIO Port C Pin 7 (PC7) 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);  						//Enable clock for UART3	
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7);  						//Configure PD7 as UART Pin
	GPIOPinConfigure(GPIO_PC7_U3TX); 													//Configure PD7 as Tx --> Rx on Bluetooth
	//Configure UART3 to transmit 8-data bits, 1-stop bit, No Parity bit, at 9600 baud 
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600, 
												(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE)); 											
	IntEnable(INT_UART3);																			//Enable UART3 Interrupts 
	UARTIntEnable(UART3_BASE, UART_INT_TX); 									//Enable Transmit Interrupts 


	
	//Configure Timer0A 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 						//Enable clock for TIMER0
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 					//Configure timer as periodic
	TimerLoadSet(TIMER0_BASE, TIMER_A, period-1); 						//Set reset value, i.e the period of timer 
	IntPrioritySet(INT_TIMER0A, 0x02); 												//Timer Interrupt priority set
	IntEnable(INT_TIMER0A); 																	//Timer0A Interrupt enable 
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 					//Arm the Timer0A interrupt 
	TimerEnable(TIMER0_BASE, TIMER_A); 												//Enable the Timer0A 

		
	
			//AN0/PE3 as an analog input
		SYSCTL_RCGCADC_R |= 1;				//Sets the sequence number
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;   //For setting up PE3
		setDelay(1000);//give time for clock to settle
		
		//Enabling GPIO Pin as ADC 
		GPIO_PORTE_DIR_R &=0x08;		
		GPIO_PORTE_AFSEL_R |= 0x08;
		GPIO_PORTE_DEN_R &= ~0x08;
		GPIO_PORTE_AMSEL_R |= 0x08;


		ADC0_CC_R = 0;							//0 because ADC0  
		ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;		//Sequence configure
		ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;	//
		ADC0_SSMUX3_R = 0;
		ADC0_SSCTL3_R = ADC_SSCTL3_END0;
		ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
}


int main(void)
{
	//Setting up a 20 MHz clock 
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); 
	int period = 500000;  //Set up timer0A to interrupt every 25ms  
	IntPriorityGroupingSet(3); 
  setUp(period);
	
	while(1)
	{		
	}
}
