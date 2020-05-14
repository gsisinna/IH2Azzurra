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

#define _XTAL_FREQ 40000000


// For sampling used in ADC() func
extern unsigned int flexor;
extern unsigned int extensor;
// For USART used in readUSART() func
extern unsigned char USARTinput_data[50];
extern unsigned char USARTinput_val[50];
extern unsigned char USARTinput_length;
extern unsigned char ir;

//For blinking the C see func blinkDisplayLetterC
unsigned int blinkCount = 0;

// For GraspControl
unsigned char Pos[4];           //Current position read from the hand.
unsigned char PrePos0[] = {245, 5, 30 , 50, 50};          //POWER  PRESHAPE
unsigned char PrePos2[] = {110, 5, 160, 245, 245};        //LATERAL PRESHAPE
unsigned char PrePos4[] = {245, 5, 80, 5, 5};             //PRECISION PRESHAPE

unsigned char closing1[] = {245, 170, 245, 245, 245};     //POWER closed limit
unsigned char closing3[] = {110 , 200, 160, 245, 245};    //LATERAL closed limit
unsigned char closing5[] = {5, 132, 132, 5, 5};           //PRECISION closed limit


//Introduces a delay of "time" [ms] between the execution of instructions 
void Delay_ms(unsigned int time){                             
    unsigned int timer;
    if(time>0)
        for(timer=0; timer<time; timer++){
            __delay_ms(1);
        }
}    

// Turns off the LED display
void Reset_segments(void){  
    PORTDbits.RD4=0;
    PORTDbits.RD5=0;
    PORTDbits.RD6=0;
    PORTDbits.RD7=0;
    PORTBbits.RB0=0;
    PORTBbits.RB1=0;
    PORTBbits.RB2=0;
}

// Turn on individual segments of the 7-segment LED display
void Display_segment(char segment,char on_off){   // on_off (0:off; 1: on)  
    switch(segment){
        case 1:
            PORTDbits.RD4=on_off;
            break;
        case 2:
            PORTDbits.RD5=on_off;
            break;
        case 3:
            PORTDbits.RD6=on_off;
            break;
        case 4:
            PORTDbits.RD7=on_off;
            break;
        case 5:
            PORTBbits.RB0=on_off;
            break;
        case 6:
            PORTBbits.RB1=on_off;
            break;
        case 7:
            PORTBbits.RB2=on_off;
            break;
    }
}

//Display specific lettersd (just C,E,H)
void Display_letter(char letter){
    Reset_segments();
    switch(letter){
        case 'H':
            Display_segment(1,1);
            Display_segment(3,1);
            Display_segment(4,1);       
            Display_segment(6,1);
            Display_segment(7,1);
            break;
        case 'E':
            Display_segment(1,1);
            Display_segment(2,1);
            Display_segment(5,1);
            Display_segment(6,1);
            Display_segment(7,1);
            break;
        case 'C':
            Display_segment(1,1);
            Display_segment(2,1);
            Display_segment(5,1);
            Display_segment(6,1);
            break;
    }
}

// Blicks C letter in LED Display as an indicator of currently running streaming operation
void blinkDisplayLetterC(void){
    unsigned int blinkLimit = 255; //For blinking to be visiable to the human eye don't blink everytime but wait a bit = blinkLimit x 10ms
    if(blinkCount < blinkLimit)           Reset_segments();
    else if(blinkCount < 2*blinkLimit)    Display_letter('C');
    blinkCount++;
    blinkCount = blinkCount % (2*blinkLimit);    
}

//Display a digit (0 to 9)
void Display_number(unsigned int number) {
Reset_segments();
    switch(number%10){
    case 0:
        Display_segment(1,1);
        Display_segment(2,1);
        Display_segment(3,1);
        Display_segment(4,1);
        Display_segment(5,1);
        Display_segment(6,1);
        break;
    case 1:  
        Display_segment(3,1);
        Display_segment(4,1);
        break;
    case 2:
        Display_segment(5,1);
        Display_segment(4,1);
        Display_segment(7,1);
        Display_segment(1,1);
        Display_segment(2,1);
        break;
    case 3:
        Display_segment(5,1);
        Display_segment(4,1);
        Display_segment(7,1);
        Display_segment(3,1);
        Display_segment(2,1);
        break;
    case 4:
        Display_segment(6,1);
        Display_segment(4,1);
        Display_segment(7,1);
        Display_segment(3,1);
        break;
    case 5:
        Display_segment(5,1);
        Display_segment(6,1);
        Display_segment(7,1);
        Display_segment(3,1);
        Display_segment(2,1);
        break;
    case 6:
        Display_segment(5,1);
        Display_segment(6,1);
        Display_segment(1,1);
        Display_segment(2,1);
        Display_segment(3,1);
        Display_segment(7,1);
        break;        
    case 7:
        Display_segment(5,1);
        Display_segment(4,1);
        Display_segment(3,1);
        break;
    case 8:
        Display_segment(1,1);
        Display_segment(2,1);
        Display_segment(3,1);
        Display_segment(4,1);
        Display_segment(5,1);
        Display_segment(6,1);
        Display_segment(7,1);
        break;
    case 9 :
        Display_segment(3,1);
        Display_segment(4,1);
        Display_segment(5,1);
        Display_segment(6,1);
        Display_segment(7,1);
        break;        
    }
}

//Send 1 byte with USART communication 
void sendUSART(unsigned char A){
    TXREG=A;
    while(!TXSTAbits.TRMT);
}

void sendUSART_int(unsigned int A){
    unsigned char lowpart;
    unsigned char highpart;
    
    lowpart = A;
    highpart = A>>8;
    sendUSART(highpart);    
    sendUSART(lowpart); 
}


// Read USART message (1 byte)
unsigned char readUSART(void){
    unsigned char value;
    while(USARTinput_val[ir] == 0);//wait
    value = USARTinput_data[ir];
    USARTinput_val[ir] = 0;      // The value is read
    ir = (ir+1) % USARTinput_length;
    return value;
}
// Read USART message (1byte). If no byte is received within a time window (see config), then return a default value 
unsigned char readUSART_withTIMEOUT(void){      
    unsigned char value;
    unsigned char time = 0;
    TMR0L = 0; //Reset the timer0 register 
    T0CONbits.TMR0ON = 1; // Start Timer0
    do{
        time = TMR0L; // Read Timer0
        if(USARTinput_val[ir] == 1){
            value = USARTinput_data[ir];
            USARTinput_val[ir] = 0;      // The value is read
            ir = (ir+1) % USARTinput_length;
            time = 255; // A condition to break the while if the value is read
        }else value = 0;
    }while(time < 156);   // see config_timer0 for the value of 156
    T0CONbits.TMR0ON = 0; // Stop Timer0
    return value;
}

// Stores ADC samples (10bits) from flexor and extensor electrodes. 
void ADC(void){ 
    unsigned char resultL;
    unsigned char resultH;   
    ADCON0bits.GO=1;
    while(ADCON0bits.GO) ; // Wait until conversion is over and GO Bit is cleared  
    // Validity = 0 -> not written //  Validity = 1 -> not read
    resultH = ADRESH;
    resultL = ADRESL;
    flexor =(resultH<<8) + resultL;
    resultH = ADRESH;
    resultL = ADRESL;
    extensor = (resultH<<8) + resultL;
}


// Converts the input in percentage to a variable ranging from 0 to 1023 
unsigned int percentage2threshold(unsigned char percentage){
    unsigned int threshold_int;
    if(percentage > 100) threshold_int = 1023;  //in other words, any value after 100 (reminder: the value is a percentage) will saturate to 100
    else if(percentage <= 6) threshold_int = 10; //similarly, if the threshold percentage is too low it saturates at the lower limit. The percentage lower limit is 6 because the hysterisis portion selected as 5% and as a result of user inputs lower than 6% percentage the threshold saturates at (6%-5%) 0.01*1023 = 10
    else{
        short long threshold_shortlong = (short long)percentage * (short long)1023 / (short long)100;
        threshold_int = threshold_shortlong;
    }
    return threshold_int;
    
    // During the multiplication of calculating the threshold value in ADC units from given percentage value,
    // in other words the numerator of the simple equation given above, multipling the value of 1023 with
    // any percentage value greater than 64 will result an overflow of an unsigned integer variable- since
    // the output of that operation will exceed the max value that an unsigned int variable can take (65535).
    // Thus, this limits the applicability of our function to any user-selected (via serial commutation from a PC)
    // percentages. If not variables are assigned to a short long type this function would have been valid only
    // percantages up to and including 64 (64*1023 = 65472 will not overflow an unsigned int)
    
    // Short long is selected because it is the minimum type that would not be overflown the theoritical max
    // this operation can take. 100*1023 = 102300 whereas the max value of short long is 8388607.
    // Besides those short long variables are local variables, which means they will be terminated when function
    // is over. Thus, they won't have a huge burden to the memory.
}

// Adjust the measured signal value w.r.t. input max values selected by the user
// Here "max value" variable we used is an indirect gain adjustments rather than a value that multiplies with the signal.
// We ask from the user a percentage(similar to threshold) that can be considered as Maximum Voluntary Contraction.
// In other words, a max value of signal reaches that user selects based on his/her observation of the measured EMG signals.
// Therefore this "max value" variable we are using should take a value between 0-100 (in percentage)
// We are not using this digital gain created here (by setting it to 100), because it causes a decrease in ADC resolution
// Only gain we are carefully adjusting is the one on the EMGs (hardware gain)
unsigned int amplifiedSignalValue(unsigned int signalVal, unsigned char maxVal){
    unsigned int amplifiedSignalValue_int;
    if(maxVal > 100) maxVal = 100;      //Saturates the max value at 100% (upper limit)
    else if(maxVal < 10) maxVal = 10;   //Saturates at the lower limit. This is assigned by us in order not to loose so much ADC resolution
    
    short long amplifiedSignalValue_shortlong = (short long)100 * (short long)signalVal / (short long)maxVal;
    if(amplifiedSignalValue_shortlong > 1023) amplifiedSignalValue_shortlong = 1023; //If there is a jump in the signal from the user-defined smaller range, this line will sature it at the upper bound
    //For example: User has defined the MVC as 50% (512 in 10-bit ADC) however at an instant the signal exceed this
    //and go up to 700 in ADC units. Without this line this signal would have been amplified to 1400, which is beyond the uint can hold   
    amplifiedSignalValue_int = amplifiedSignalValue_shortlong;

    return amplifiedSignalValue_int;
    // Variable type changes are similar to ones already made for above percentage2threshold function
}

unsigned char adjustEEPROMdata(unsigned char type, unsigned char data){
    // Used for adjusting the default values for data stored in EEPROM memory.
    // Once it is written to the memory it will revived as intended. However,
    // after a new programming of device the EEPROM memories automatically clears out
    // and a memory slot of the EEPROM is empty, it reads the value of 255.
    // Therefore, this piece of code replaces the default values of the data types in that mentioned circumstances
    unsigned char value;
    unsigned char defVal_th = 50;   //Default value for Thresholds (in percentage)
    unsigned char defVal_mv = 100;  //Default value for Max Value (in percentage)
    if(data == 255){
        switch(type){
            case 'T':   // T stands for Thresholds
                value = defVal_th;
                break;
            case 'M':   // M stands for Max value
                value = defVal_mv;
                break;
        }
    }else value = data;
    
    return value;
}

// Write new values in the EEPROM addresses.
void write_EEprom(unsigned char address, unsigned char data){
    EEADR   = address; //Assign adress to the EEPROM Adress Register Bit [0-255]
    EEDATA  = data;    //Assign data to the EEPROM Data Register Bit [0-255]
    
    EECON1bits.EEPGD = 0; //Selects EEPROM Memory
    EECON1bits.CFGS  = 0; //Access EEPROM Memory
    EECON1bits.WREN  = 1; //Enables writing
    
    // NOTE: It is recommended to disable interrputs during writing (since a interrupt may occur mid-way writing operation)
    INTCONbits.GIEH = 0;    //Disabling interrupts during writing
    INTCONbits.GIEL = 0;
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;    // Initiates the writing process
    while(EECON1bits.WR); // Waits until the data is written  
    
    INTCONbits.GIEH = 1;  // Enables back interrupts after writing
    INTCONbits.GIEL = 1;
    
    EECON1bits.WREN  = 0; //Inhibits writing
}

//Read EEPROM addresses
unsigned char read_EEprom(unsigned char address){
    EEADR   = address; //Assign adress to the EEPROM Adress Register Bit [0-255]
    EECON1bits.EEPGD = 0; //Selects EEPROM Memory
    EECON1bits.CFGS  = 0; //Access EEPROM Memory
    
    EECON1bits.RD = 1; //Enables Reading
    //while(EECON1bits.RD); // Wait (actually we dont really need it since READ takes one instruction cycle)    
    return EEDATA;  //Read and output the data
}

//Disables the Counter Switch Interrupt. (used when hand is in grasping states to prevent sudden change to computer mode)
void disableModeSwitch(void){ 
    INTCONbits.INT0IE = 0; // Disables the interrupt of Switch
    INTCONbits.INT0IF = 0; // Clears the flag
    // In case the flag is not cleared, if the user presses the switch during a grasping state
    // the mode is automatically switched when the system goes to a preshape state.
    // With this approach, user is expected to switch another time when the system is
    // in a preshape state- if he/she has the intent of mode switching.
}

unsigned int absFunc(unsigned int A, unsigned int B){ 
    unsigned int result;  
    if(A > B) result = A - B;
    else if(A < B) result = B - A;
    else result = 0;   
    return result;
}

// Sign function.  Used to determine which direction the hand motors should be driven
unsigned char signFunc(unsigned int F, unsigned int E){ 
    unsigned char signResult;    
    if(F > E) signResult = 1; 
    else      signResult = 0;   
    return signResult;    
}

 
unsigned int velMapping(unsigned int value, unsigned int threshold){
    // Maps the speed that hand motors should be driven based on sensory information (and threshold as a base)
    unsigned int resultingVel;
    unsigned int lowerVelLimit = 300; 
    
    if(value > threshold){
        unsigned short long aux =(unsigned short long)  (511 - lowerVelLimit) *  (unsigned short long) (value - threshold);
        resultingVel = aux / (unsigned short long) (1023 - threshold); 
        resultingVel = resultingVel + lowerVelLimit;
    }else resultingVel = 0;
    
    return resultingVel;
}


void MoveMotor(unsigned char S, unsigned char MA3210, unsigned int vel) {
    unsigned char D8 = vel>>8; 
    unsigned char D70 = vel;
    unsigned char FirstByte = (1<<7) | (S << 6) | (MA3210 << 2) | 0<<1 | (D8);
    sendUSART(FirstByte);
    sendUSART(D70);
}

unsigned char getFingerStatus(unsigned char MA3210){
    unsigned char statusFinger;
    sendUSART(0x4B);    //startByte
    sendUSART(MA3210);  //motor id
    statusFinger = readUSART();
    return statusFinger;
}

void waitFingerReached(unsigned char motorID){
    unsigned char status;
    unsigned char statusBit4;
    do{
        status = getFingerStatus(motorID);
        statusBit4 = (status>>4) & 0x1;
    }while(statusBit4 != 1);
}

void waitHandReached(void){
    unsigned char motorID;
    for(motorID = 0 ; motorID < 5 ; motorID++){      
        if(motorID != 1) waitFingerReached(motorID);  // Do except Thumb Flex/Ext (because it doesn't work due to a hardware problem)
    }
}

unsigned char getFingerPosition (unsigned char MA){    
        sendUSART(0x45); // Start byte
        sendUSART(MA);
        unsigned char value = readUSART();
        return value;
}

void getHandPosition(void){
    unsigned char i_finger;
    for(i_finger = 0; i_finger < 5; i_finger++){
        Pos[i_finger] = getFingerPosition(i_finger);
    }
}


void GraspPreshape(unsigned char preshapeState){
  sendUSART(0x48);
  switch (preshapeState){
      case 0: //Power
          sendUSART(PrePos0[0]);
          sendUSART(PrePos0[1]);
          sendUSART(PrePos0[2]);
          sendUSART(PrePos0[3]);
          sendUSART(PrePos0[4]);
          break;
      case 2: //Lateral
          sendUSART(PrePos2[0]);
          sendUSART(PrePos2[1]);
          sendUSART(PrePos2[2]);
          sendUSART(PrePos2[3]);
          sendUSART(PrePos2[4]);
          break;
      case 4: //Precision
          sendUSART(PrePos4[0]);
          sendUSART(PrePos4[1]);
          sendUSART(PrePos4[2]);
          sendUSART(PrePos4[3]);
          sendUSART(PrePos4[4]);
          break;
    }
    sendUSART(0x48);    
    waitHandReached();
}


void GraspControl(unsigned char graspState, unsigned char S, unsigned int speed){   
    // Speed Adjustments (only for power grasp and not required for other grasps)
    unsigned char indexpos = 125; 
    unsigned int thumbPercentage = 40;
    unsigned int auxspeed_thumb = (thumbPercentage * speed) / 100;

    unsigned int littlePercentage = 120; //LIMIT IS 128
    unsigned int auxspeed_little = (littlePercentage * speed) / 100;
    if(auxspeed_little > 511)   auxspeed_little = 511;   
    // With these speed adjustments we slower down the thumb and speed up the ring-little fingers a litte for a more natural graps charactheristics

    getHandPosition();
    switch(graspState){// S = 0 FINGER OPENS , S = 1 FINGER CLOSES        
        case 1: //Closing/Opening of Power           
            if ((Pos[1] >= closing1[1] && S == 1) || (Pos[1] <= PrePos0[1] && S == 0))  MoveMotor(S,1,0);              
            else if ((S == 1 && Pos[2] >= indexpos) || (S == 0)) MoveMotor(S,1,speed);
            else MoveMotor(S,1,auxspeed_thumb);
            
            if ((Pos[2] >= closing1[2] && S == 1) || (Pos[2] <= PrePos0[2] && S == 0))  MoveMotor(S,2,0);              
            else MoveMotor(S,2,speed); 
            
            if ((Pos[3] >= closing1[3] && S == 1) || (Pos[3] <= PrePos0[3] && S == 0))  MoveMotor(S,3,0);              
            else MoveMotor(S,3,speed);           
            
            if ((Pos[4] >= closing1[4] && S == 1) || (Pos[4] <= PrePos0[4] && S == 0))  MoveMotor(S,4,0);              
            else MoveMotor(S,4,auxspeed_little);          
            break;
        case 3: //Closing of Lateral         
            if ((Pos[1] >= closing3[1] && S == 1) || (Pos[1] <= PrePos2[1] && S == 0) )  MoveMotor(S,1,0);              
            else MoveMotor(S,1,speed);         
            break;
        case 5: //Closing of Precision      
            if ((Pos[1] >= closing5[1] && S == 1) || (Pos[1] <= PrePos4[1] && S == 0) )  MoveMotor(S,1,0);              
            else MoveMotor(S,1,speed); 
            if ((Pos[2] >= closing5[2] && S == 1) || (Pos[2] <= PrePos4[2] && S == 0) )  MoveMotor(S,2,0);              
            else MoveMotor(S,2,speed);          
            break;
    }
}

unsigned char conditionRelax(unsigned int closingState){
    unsigned char conditionMet;
    unsigned char motorID;
    switch(closingState){
        case 1: //Power
            if (Pos[0] <= PrePos0[0] && Pos[1] <= PrePos0[1] && Pos[2] <= PrePos0[2] && Pos[3] <= PrePos0[3] && Pos[4] <= PrePos0[4]) conditionMet=1;
            else conditionMet = 0;
            break;
        case 3: //Lateral
            if (Pos[1] <= PrePos2[1]) conditionMet=1;
            else conditionMet = 0;
            break;
        case 5: //Precision
             if (Pos[1] <= PrePos4[1] && Pos[2] <= PrePos4[2]) conditionMet=1;
            else conditionMet = 0;
            break;
    }      
    return conditionMet;
}

void clearUSARTbuffers(void){ //RESET THE RX, TX BUFFERS
    unsigned char dataClearBuffer;
    unsigned char i_count;
    for(i_count = 0; i_count < 5; i_count++){
        dataClearBuffer = RCREG;
    }
}

void waitFastCalibration(void){ //UNDER DEVELOPMENT
    unsigned char response;
//    do{
//        sendUSART(0x45); // Start byte of get finger position
//        sendUSART(2); // Position of the Index Finger
//        response = readUSART_withTIMEOUT();
//    }while(response == 255); //255 = Default value coming from readUSART_withTIMEOUT
//    clearUSARTbuffers();
    
    do{
        Delay_ms(100);
        sendUSART(0x45); // Start byte of get finger position
        sendUSART(2); // Position of the Index Finger
    }
    while(USARTinput_val[ir] == 0);
    response = readUSART();
}