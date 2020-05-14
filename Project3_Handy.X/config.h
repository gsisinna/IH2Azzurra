/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */



//CutiePies
// PIC18F4431 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1     // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RD1      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RD3 and RD2, respectively. SDO output is multiplexed with RD1.) //Hardware architecture
#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Low-Voltage ICSP Enable bit (Low-voltage ICSP enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (002000-002FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (003000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (002000-002FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (003000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (002000-002FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (003000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

///////////////////////////////////////////////////////////////////////////////

void config_DIO(void){
    //Single led light
    ANSEL0bits.ANS0=0; // analog input AN0 disabled (digital input/output)
    TRISAbits.RA0=0;   // define PORT A as OUTPUT
    // 7 segment led display
    TRISDbits.RD4=0; //
    TRISDbits.RD5=0; //
    TRISDbits.RD6=0; //
    TRISDbits.RD7=0; //
    TRISBbits.RB0=0; //
    TRISBbits.RB1=0; //
    TRISBbits.RB2=0; //
    //Switch sensor
    TRISBbits.RB3=1; //   define PORT A as INPUT
}

void config_interrupt(void){
    RCONbits.IPEN = 1;        //Interrupt Priority (In this case: high priorities are Enabled)    
    INTCONbits.GIE_GIEH = 1;
    INTCONbits.PEIE_GIEL = 1;
    INTCONbits.INT0IE = 1;
    INTCON2bits.INTEDG0 = 1;  //Counter Switch Setting for RISING EDGE DETECTION
}

void config_timer0(void){    // Register 10-1: INTCON
    T0CONbits.T016BIT   = 1; // 16-bit or 8-bit counter (In this case: 8-bit)
    T0CONbits.T0CS      = 0; // Use Internal Clock(FOSC/4)    
    T0CONbits.PSA       = 0; // Enable Pre-scale (In this case: Enabled)
    // Prescaler bits  
    T0CONbits.T0PS2     = 1; // In this case 1:128
    T0CONbits.T0PS1     = 1;
    T0CONbits.T0PS0     = 0; 
    //NOTE: READ USART TIMEOUT
    // We want to wait 2ms = 20,000 = Ticks*Prescaler
    // Since we chose prescaler 128, Ticks = 20,000 / 128 = 156.25
    // We chose 156 ticks to count 156*128 = 19,968 (almost 20,000)   
}

void config_timer1(void){  
    T1CONbits.RD16  = 1; //Enables read/write in one 16-bit operation    
    // We want to sample at 1kHz (a sample at every 1 ms)
    // Tout = Ticks * (4*Prescale/FOSC)
    // Prescale = 1:1 and FOSC = 40MHz the Ticks found from the above equation
    // 55.535 (in dec) = 11011000 11101111
    TMR1H = 0b11011000;
    TMR1L = 0b11101111;
        
    // Prescale Bits
    T1CONbits.T1CKPS1 = 0; // In this case 1:1
    T1CONbits.T1CKPS0 = 0;
    T1CONbits.T1OSCEN = 0; // Oscillator off
    T1CONbits.TMR1CS  = 0; // Internal Clock (FOSC/4)
    
    PIE1bits.TMR1IE = 1;   // TIMER1 Flag Enable
    IPR1bits.TMR1IP = 0;   // Timer0 has Low Priority
    T1CONbits.TMR1ON = 1;  // Timer1 Enabled
    
}

void configUSART(void){ 
    // Transmitting
    SPBRG=21;
    BAUDCONbits.BRG16 = 0; //Enable 8-bit Baud Rate
    TXSTAbits.BRGH    = 1; // High Baud Rate Select Bit (In this case: High Speed)
    TXSTAbits.SYNC    = 0; // EUSART Mode Select (In this case: Asyncronous)
    RCSTAbits.SPEN    = 1; // Serial Port Enable Bit (In this case: Enabled)
    TXSTAbits.TXEN    = 1; // Trasnmit Enable Bit (In this case: Enabled)
    // Output pins
    TRISCbits.RC6     = 1;
    TRISCbits.RC7     = 1; 
    // Receiving
    RCSTAbits.CREN = 1; // Continuous Receive Enable Bit (In this case: Reciever Enabled)
    PIE1bits.RCIE  = 1; // Interrupt Enable for Recieving 
    IPR1bits.RCIP  = 0; // USART has Low Priority
}

void configADC(void){  
    TRISEbits.RE2 = 1; // Potentiometer 1, Group A, AN8 (Input)
    TRISAbits.RA1 = 1; // Potentiometer 2, Group B, AN1 (Input)
    
    ANSEL0bits.ANS1 = 1; // Making AN1 an Analog Input
    ANSEL1bits.ANS8 = 1; // Making AN8 an Analog Input 
    
    // Group A Select bits
    ADCHSbits.GASEL1 = 1; // In this case: AN8
    ADCHSbits.GASEL0 = 0;
    // Group B Select bits
    ADCHSbits.GBSEL1 = 0; // In this case: AN1
    ADCHSbits.GBSEL0 = 0;
    
    //ADCON0 Register
    ADCON0bits.ACONV  = 0;  // Single Shot Mode Enabled
    ADCON0bits.ACSCH  = 1;  // Multi Channel Mode Enabled (Single Channel Mode Disabled)
    ADCON0bits.ACMOD1 = 1;  // Mode (In this case both Group A and Group B sampled simultaneously)
    ADCON0bits.ACMOD0 = 0;
    
    //ADCON1 Register
    ADCON1bits.VCFG1  = 0;  // Reference Voltage Choice
    ADCON1bits.VCFG0  = 0;  // In this case: VREF+ = AVDD(5V), VREF- = AVSS(0V)      
    ADCON1bits.FIFOEN = 1;  // FIFO is enabled
    
    //ADCON2 Register
    ADCON2bits.ADFM  = 1; // Right-justified Result: ADRESL->Full ADRESH-> First 2 bits 
    ADCON2bits.ACQT3 = 0; // A/D Acquisition Time Select bits 
    ADCON2bits.ACQT2 = 0; // In this case: No Delay (conversion starts immediately when GO/DONE is set)
    ADCON2bits.ACQT1 = 0; 
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 0; // A/D Conversion Clock Select bits
    ADCON2bits.ADCS1 = 1; // In this case: FOSC/32
    ADCON2bits.ADCS0 = 0;    
    ADCON0bits.ADON  = 1;
}
