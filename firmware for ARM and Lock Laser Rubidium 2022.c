//*****************************************************************************
//                   Laboratoire de Physique des Lasers
//                            Wiotte Fabrice 
//                        Ingénieur d'étude CNRS
//                    Service Electronique D002 Rdc
//                CNRS – URM7538 – Université Paris 13
//                       99 Avenue J.-B. Clément
//                         93430 Villetaneuse
//                    http://www-lpl.univ-paris13.fr/
//DDS board AD9959 on TM4C123 for 1GHz clock distribution and DDS chanel CH1 = 100KHz to 190MHz
//                       date : 05/04/2022 revision 2
//                        Lock Laser Rubidium 2022
// This is part of revision 2.1.0.12573 of the EK-TM4C123 Firmware Package.
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "grlib/grlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "driverlib/flash.h"
//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
#define SSI0_CR0_R              (*((volatile uint32_t *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32_t *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32_t *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32_t *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32_t *)0x40008010))
#define SSI0_IM_R               (*((volatile uint32_t *)0x40008014))
#define SSI0_RIS_R              (*((volatile uint32_t *)0x40008018))
#define SSI0_MIS_R              (*((volatile uint32_t *)0x4000801C))
#define SSI0_ICR_R              (*((volatile uint32_t *)0x40008020))
#define SSI0_DMACTL_R           (*((volatile uint32_t *)0x40008024))
#define SSI0_CC_R               (*((volatile uint32_t *)0x40008FC8))
#define SSI_SR_TNF    0x00000002  // SSI Transmit FIFO Not Full

//UART0 PORTA
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control

// GPIO for port F
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))

// GPIO for port A
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_PUR_R        (*((volatile unsigned long *)0x40004510))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_CR_R         (*((volatile unsigned long *)0x40004524))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))

// GPIO for port D
#define GPIO_PORTD_DATA_BITS_R  ((volatile unsigned long *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_IS_R         (*((volatile unsigned long *)0x40007404))
#define GPIO_PORTD_IBE_R        (*((volatile unsigned long *)0x40007408))
#define GPIO_PORTD_IEV_R        (*((volatile unsigned long *)0x4000740C))
#define GPIO_PORTD_IM_R         (*((volatile unsigned long *)0x40007410))
#define GPIO_PORTD_RIS_R        (*((volatile unsigned long *)0x40007414))
#define GPIO_PORTD_MIS_R        (*((volatile unsigned long *)0x40007418))
#define GPIO_PORTD_ICR_R        (*((volatile unsigned long *)0x4000741C))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DR2R_R       (*((volatile unsigned long *)0x40007500))
#define GPIO_PORTD_DR4R_R       (*((volatile unsigned long *)0x40007504))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
#define GPIO_PORTD_ODR_R        (*((volatile unsigned long *)0x4000750C))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile unsigned long *)0x40007514))
#define GPIO_PORTD_SLR_R        (*((volatile unsigned long *)0x40007518))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile unsigned long *)0x40007530))
#define GPIO_PORTD_DMACTL_R     (*((volatile unsigned long *)0x40007534))

// GPIO for port E
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

#define PF0 (*((volatile unsigned long *)0x40025004))
#define PF1 (*((volatile unsigned long *)0x40025008))    
#define PF2 (*((volatile unsigned long *)0x40025010))     
#define PF3 (*((volatile unsigned long *)0x40025020))
#define PF4 (*((volatile unsigned long *)0x40025040))

#define PD0 (*((volatile unsigned long *)0x40007004))
#define PD1 (*((volatile unsigned long *)0x40007008))
#define PD2 (*((volatile unsigned long *)0x40007010))
#define PD3 (*((volatile unsigned long *)0x40007020))
#define PD4 (*((volatile unsigned long *)0x40007040))
#define PD5 (*((volatile unsigned long *)0x40007080))

#define PA6 (*((volatile unsigned long *)0x40004100))
#define PA7 (*((volatile unsigned long *)0x40004200))

// Flash ROM addresses must be 1k byte aligned, e.g., 0x8000, 0x8400, 0x8800...
#define FLASH                   0x10000  // location in flash to write; make sure no program code is in this block
#define FLASH1                  0x10400

                    /* DDS AD9959 */ 
short CSR_ADRESS = 0x00;                  // AD9959 CSR adresss Byte
int CSR_NUM_BYTE = 0x01;                  // AD9959 byte number by CSR 
long CSR0 = 0x10;                         // AD9959 CH0  MSB first
long CSR1 = 0x20;                         // AD9959 CH1  MSB first
long CSR2 = 0x40;                         // AD9959 CH2  MSB first
long CSR3 = 0x80;                         // AD9959 CH3  MSB first

                  
short FTW_ADRESS = 0x04;                  // AD9959 FTW adresss Byte 
int FTW_NUM_BYTE = 0x04;
long FTW0 = 0x51EB851;      // AD9959 Frequency Tuning Word_0 10 MHz @ Clk=500MHz 1GHz /2  
long FTW1;
long FTW2;
long FTW3;
    
short ACR_ADRESS = 0x06;                 // AD9959 ACR (Amp Ctrl Register)addresss Byte
int ACR_NUM_BYTE = 0x03;                  // AD9959 byte number by ACR 
long ACR0 = 0x1400;       // full DAC = 0x1400 10bits OV output  Manual mode is selected by programming ACR <12:11> = 10.
long ACR1; 
long ACR2;       
long ACR3;

int i = 0;
int j = 0;
int t = 0;
char data;
char temp[6];
unsigned long mot_32bits;
int amplitude_CH1;
uint32_t FTW1_memory[1];
uint32_t ACR1_memory[1];
long read_byte1;
long read_flash1;
int read_byte2;
int read_flash2;

//fonctions//
void write_immediate(void);
void ssi0PutData( int instruction,long data,int num_byte);
void UARTIntHandler(void);
void GPIOIntHandler(void);
void interrupt_portA(void);
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
//send data to uC via USB/Uart for change one output frequency ( 4 octets)
void UARTIntHandler(void)
{
  
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true);
 
    UARTIntClear(UART0_BASE, ui32Status);

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
     {
       //send data to uC via USB/Uart for change one output frequency ( 4 octets) 
       temp[t++] = UART0_DR_R; //Read from buffer
           if(t>5)
             {
              write_immediate();
              t = 0; //Reset read length
             }
     }
}
//------------UART_InChar------------
// Wait for new input, then return ASCII code
unsigned char UART_InChar(void)
{
  while((UART0_FR_R&0x0010) != 0);      // wait until RXFE is 0
  return((unsigned char)(UART0_DR_R&0xFF));
}
//------------UART_OutChar------------
// Wait for new output, then return ASCII code
void UART_OutChar(unsigned char data)
{
  while((UART0_FR_R&0x0020) != 0);      // wait until TXFF is 0
  UART0_DR_R = data;
}
void PortA_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001;     // 1) activate clock for Port A
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTA_AMSEL_R &= ~0x80;      // 3) disable analog on PA7
  GPIO_PORTA_PCTL_R &= ~0xF0000000; // 4) PCTL GPIO on PA7
  GPIO_PORTA_DIR_R |= 0x00;        // 5) PORT A IN
  GPIO_PORTA_AFSEL_R &= ~0x80;      // 6) disable alt funct on PA7
  GPIO_PORTA_DEN_R |= 0xFF;         // 7) enable digital I/O 
  
}
void PortD_Init(void)
{
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000008;     // 1) activate clock for Port D
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) regular port function 
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO
  GPIO_PORTD_DIR_R |= 0xFF;           //PD output
  GPIO_PORTD_DEN_R |= 0xFF;         // 7) enable digital I/O on PD7-0
}
//// PF4 is input SW1 and PF2 is output Blue LED
void PortF_Init(void)
{ 
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  //GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-PF1 out 
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF4,PF2,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  //GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}
void PeripheralEnableInit(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}
/*void init_UART0(void)
{
  // Enable Peripheral UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,                      
  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
*/
// 1Mbps BAUD RATE USB to Sérial FTDI adaptator
void init_UART0(void)
{
  //Enable Peripheral UART0  1Mbps
  // Initialize the UART for 8Mbps baud rate (assuming 80 MHz UART clock),
  // 8 bit word length, no parity bits, one stop bit, FIFOs enabled
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  SYSCTL_RCGC2_R |= 0x00000001;     // 1) activate clock for Port A
  //UART0_IBRD_R = 1;                    // IBRD = int(80,000,000 / (8 * 8000000)) = int(1)= 1  baud rate = 8Mbps
  //UART0_FBRD_R = 16;                    // FBRD = round(0.25 * 64) = 16
  UART0_IBRD_R = 10;                    // IBRD = int(80,000,000 / (8 * 1000000)) = int(10)= 10  baud rate = 1Mbps
  UART0_FBRD_R = 0;                    // FBRD = 0
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= 0x00000021; // enable UART and High-Speed Enable The UART is clocked using the system clock divided by 8.
  //UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1,PA0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1,PA0
                                        // configure PA1,PA0 as UART0
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA1,PA0
  
}
//*****************************************************************************
//
//  SSI0 SERIAL PORT
//
//*****************************************************************************
void ssi0PutData( int instruction,long data,int num_byte)
{
   int i=0;
   SSI0_DR_R =  instruction; 
    while( num_byte )
     {
          while(!(SSI0_SR_R & SSI_SR_TNF)) {} // wait until there is space to write in the buffer
          SSI0_DR_R =  data >>(num_byte-1-i)*8;
          num_byte--;           
     }
    
    while( !( SSI0_SR_R & SSI_SR_TNF ) )
          {
            ;
          }
}
void init_SPI0(void)
{
    // Enable Peripheral SSI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enable GPIO port A pins which are used for SSI0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); 
    //GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    //GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 10000000, 8);
    SSIEnable(SSI0_BASE); // Enable the SSI 
}
/************************************************************/
/* Prototype - write_immediate                              */
/*                                                          */
/*  Description                                             */
/*  Reads incoming frequency and send to DDS immediately    */
/************************************************************/
void write_immediate()
{
  mot_32bits = ((unsigned long)(temp[3]) << 24) | ((unsigned long)(temp[2]) << 16) | ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[0]));
  amplitude_CH1 =  ((unsigned long)(temp[5]) << 8) | ((unsigned long)(temp[4]));
  FTW1 = (mot_32bits * 4294967296 / 500000000); //Convert to command for DDS  //ad9959 
  PD3 =0x00; // AD9959 CS= 0  
  //place in flash memory frequency //
  FTW1_memory[0] = (long)FTW1;
  ACR1_memory[0] = 0x1000 + amplitude_CH1;
  FlashErase(FLASH),FlashErase(FLASH1);
  FlashProgram(FTW1_memory, FLASH, sizeof(FTW1_memory));
  FlashProgram(ACR1_memory, FLASH1, sizeof(ACR1_memory));
  ssi0PutData(CSR_ADRESS,CSR1,CSR_NUM_BYTE); 
  ssi0PutData(FTW_ADRESS,FTW1,FTW_NUM_BYTE);
  ssi0PutData(ACR_ADRESS,0x1000 + amplitude_CH1,ACR_NUM_BYTE);
  SysCtlDelay(2000);
  PD1 =0x02; // AD9959 I/O update
  SysCtlDelay(20);
  PD1 =0x00;
  PD3 =0x08; // AD9959 CS= 1
}
//entrées TTL PA6 PA7 //
void interrupt_portA(void) 
{
	if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_6) 
           {
                GPIOIntRegister(GPIO_PORTA_BASE, interrupt_portA);   // Register our handler function for port A
                GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_RISING_EDGE);  // Configure PA6 for rising edge trigger
                GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);  // Clear interrupt flag 
		PD3 =0x00; // AD9959 CS= 0
                ssi0PutData(CSR_ADRESS,CSR1,CSR_NUM_BYTE); 
                ssi0PutData(FTW_ADRESS,0xA3D70A2,FTW_NUM_BYTE);
                SysCtlDelay(100);
                PD1 =0x02; // AD9959 I/O update
                SysCtlDelay(10);
                PD1 =0x00;
		GPIOIntRegister(GPIO_PORTA_BASE, interrupt_portA);	// Register our handler function for port A
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_RISING_EDGE);			// Configure PA6 for falling edge trigger
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_6);	// Clear interrupt flag
                PD3 =0x08; // AD9959 CS= 1
	}
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);

        if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_7) 
           {    
                GPIOIntRegister(GPIO_PORTA_BASE, interrupt_portA);   // Register our handler function for port A
                GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_RISING_EDGE);  // Configure PA7 for rising edge trigger
                GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_7);  // Clear interrupt flag 
		PD3 =0x00; // AD9959 CS= 0
                ssi0PutData(CSR_ADRESS,CSR1,CSR_NUM_BYTE); 
                ssi0PutData(FTW_ADRESS,0x369D036,FTW_NUM_BYTE);
                SysCtlDelay(100);
                PD1 =0x02; // AD9959 I/O update
                SysCtlDelay(10);
                PD1 =0x00;
		GPIOIntRegister(GPIO_PORTA_BASE, interrupt_portA);	// Register our handler function for port A
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_RISING_EDGE);			// Configure PA7 for falling edge trigger
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_7);	// Clear interrupt flag
                PD3 =0x08; // AD9959 CS= 1
	  }
          GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
}

int main(void)
{  
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // Set the clocking to run at 80MHz from the PLL.
    PeripheralEnableInit(); 
    PortA_Init();
    PortD_Init();
    PortF_Init();
    init_UART0();
    IntEnable(INT_UART0);  //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
    init_SPI0();
    SysCtlDelay(2000);
    //AD9959//
    PD4 =0x10; // reset DDS
    SysCtlDelay(200);
    PD4 =0x00; 
    PD0 =0x00;    // NO power down
    PD3 =0x00;    // AD9959 CS= 0
    PD2 =0x00; 
    
    read_byte1 =  *(unsigned  long *)FLASH;   //read FLASH for FTW1
    read_flash1 = read_byte1;  
    
    read_byte2 =  *(unsigned  long *)FLASH1;   //read FLASH for ACR1
    read_flash2 = read_byte2;  
       
       
    ssi0PutData(CSR_ADRESS,CSR0,CSR_NUM_BYTE);
    SysCtlDelay(2000);
    ssi0PutData(FTW_ADRESS,FTW0,FTW_NUM_BYTE);
    SysCtlDelay(2000);
    ssi0PutData(CSR_ADRESS,CSR1,CSR_NUM_BYTE); 
    SysCtlDelay(2000);
    ssi0PutData(FTW_ADRESS,read_flash1,FTW_NUM_BYTE);
    SysCtlDelay(2000);
    ssi0PutData(ACR_ADRESS,read_flash2,ACR_NUM_BYTE);
    SysCtlDelay(2000);
    PD1 =0x02; // AD9959 I/O update
    SysCtlDelay(2000);
    PD1 =0x00;
    SysCtlDelay(2000);
    
    
    PD5 =0x20;  // lock/unlock relais
    SysCtlDelay(20000);
    PD5 =0x00;
    
    // Interrupt setup
    GPIOIntDisable(GPIO_PORTA_BASE,GPIO_PIN_7|GPIO_PIN_6);        // Disable interrupt for PA6 PA7 (in case it was enabled)
    GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_7|GPIO_PIN_6);      // Clear pending interrupts for PA6 PA7
    GPIOIntRegister(GPIO_PORTA_BASE, interrupt_portA);     // Register our handler function for port A
    GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_7|GPIO_PIN_6,GPIO_RISING_EDGE); // Configure PA6 PA7 for falling edge trigger
    GPIOIntEnable(GPIO_PORTA_BASE,GPIO_PIN_7|GPIO_PIN_6);     // Enable interrupt for PA6 PA7
    
    IntMasterEnable();  // Enable processor interrupts.
    
    
    while(1){} // Loop forever while the timers run. 
   
} 
