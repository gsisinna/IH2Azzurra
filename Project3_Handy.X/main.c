
#include <xc.h>
#include "config.h"
#include "mylib.h"

// Global Variables
unsigned char controlRate = 10;         //Actuation commands are given to the hand each controlRate [ms]. The actuation commands are a funtion of the moving average (of window controlRate [ms]) of the input data coming from sensors   
unsigned char USARTinput_data[50];      //Circular buffer storing the incoming data from USART communication (hand query answers in hand mode or instructions in computer mode)
unsigned char USARTinput_val[50];       //Circular buffer of the Validity of USARTinput_data(semaphore) 
unsigned char USARTinput_length = 50;   //Size of the buffer (has to be manually edited in USARTinput_data and USARTinput_val definition )
unsigned char ir = 0;                   //Current index for reading(semaphore)
unsigned char iw = 0;                   //Current index for writing(semaphore)

unsigned int flexor_average;       //Backward Moving average (of window controlRate) of the input flexor signal 
unsigned int extensor_average;     //Backward Moving average (of window controlRate) of the input extensor signal 
unsigned int is;                   //Index (for loop) for the Moving Average computing. 
unsigned int flexor_averagePart;   //Iterative preliminar calculation of flexor_average
unsigned int extensor_averagePart; //Iterative preliminar calculation of extensor_average

unsigned int flexor;                //Input flexor signal raw sample
unsigned int extensor;              //Input flexor signal raw sample
unsigned char CT10F;                //Flag for verifying controlRate time has been reached (0:still intermediate sampling; 1:MA ready to send)
unsigned char deviceMode;           //State variable of the operation mode (0: Hand mode; 1: Computer mode)

unsigned char THE_ADR = 0x00;       //EEPROM address for the extensor threshold
unsigned char THF_ADR = 0x01;       //EEPROM address for the flexor threshold
unsigned char MVE_ADR = 0x02;       //EEPROM address for the maximum value (MV) of the extensor sensor
unsigned char MVF_ADR = 0x03;       //EEPROM address for the maximum value (MV) of the flexor sensor
unsigned char thEbyte = 0xAA;       //Communication protocol first and end byte for Writing thE instruction
unsigned char thFbyte = 0xBB;       //Communication protocol first and end byte for Writing thF instruction
unsigned char mvEbyte = 0xCC;       //Communication protocol first and end byte for Writing mvE instruction
unsigned char mvFbyte = 0xDD;       //Communication protocol first and end byte for Writing mvF instruction
unsigned char Sbyte   = 0xEE;       //Communication protocol first and end byte for Streaming information instruction
unsigned char STPbyte = 0xEF;       //Communication protocol (only 1 byte) for Stop Streaming instruction
unsigned char Dbyte   = 0xFF;       //Communication protocol first and end byte for Reading information instruction
unsigned char THE_DATA = 0;         //Extensor theshold data stored in EEPROM (in percentage)
unsigned char THF_DATA = 0;         //Flexor theshold data stored in EEPROM (in percentage)
unsigned char MVE_DATA = 100;       //Extensor Max Value data stored in EEPROM (in percentage)
unsigned char MVF_DATA = 100;       //Flexor Max Value data stored in EEPROM (in percentage)
unsigned int thE = 0;               //Extensor threshold (ranging from 0 to 1023)
unsigned int thF = 0;               //Flexor threshold (ranging from 0 to 1023)
unsigned int hysteresis = (5*1023)/(unsigned int)100;   //Hystheresis introduced so that   
unsigned int thE_h = 0;                                 // High threshold for Extensor
unsigned int thE_l = 0;                                 // Low threshold for Extensor
unsigned int thF_h = 0;                                 // High threshold for Flexor
unsigned int thF_l = 0;                                 // Low threshold for Flexor
unsigned char TCF_E = 0;                                // Extensor Threshold Change Flag (for updating each time when a change happened)
unsigned char TCF_F = 0;                                // Flexor Threshold Change Flag (for updating each time when a change happened)


//Interrupt for switching between operation modes (hand mode/computer mode) using manual switch
void interrupt InterruptAction(void){
        
    if(INTCONbits.INT0IF){ //Counter Switch
        deviceMode = (deviceMode + 1) % 2;      //Change the mode from 0 to 1 or from 1 to 2
                                                // deviceMode = 0 HAND (H)
                                                // deviceMode = 1 COMPUTER (C)
        //Displays the new mode
        if(deviceMode == 0){ 
            Display_letter('H');
            Delay_ms(1000);
        }else if(deviceMode == 1) Display_letter('C');
        else Display_letter('E');
        
        if(TCF_E == 1){ //If extensor threshold changed in computer mode
            THE_DATA = read_EEprom(THE_ADR);        //reads extensor threshold from EEPROM
            thE = percentage2threshold(THE_DATA);   //converts THE_DATA (in percentage) to a 10bit value 
            thE_h = thE;                            //Assigns thE as the HIGH threshold level value. 
            thE_l = thE - hysteresis;               //LOW threshold level
            TCF_E = 0;                              //Resets the flag to zero
        }
  
        if(TCF_F == 1){ //If flexor threshold changed in computer mode (comments are analogue to the previous section but for flexor)
            THF_DATA = read_EEprom(THF_ADR);
            thF = percentage2threshold(THF_DATA);
            thF_h = thF;
            thF_l = thF - hysteresis;
            TCF_F = 0;
        }
        clearUSARTbuffers();
        INTCONbits.INT0IF = 0;
    }     
}
  

//Timer Interrupt for Sampling at specified frequency (1ms)
void interrupt low_priority InterruptAction_low(void){
    if(PIR1bits.TMR1IF){ //Timer1 overflows at 1ms
        //PORTAbits.RA0=!PORTAbits.RA0;   // Uncomment this line (keeping all the rest the same) to check frequency with the oscilloscope)  
        //Next lines are for timer1 post-scale 
        TMR1H = 0b11011000; 
        TMR1L = 0b11101111;
       
        ADC(); // Writes the 2 signals to the buffers
   
        flexor_averagePart   = (flexor + is * flexor_averagePart)/(is+1);       //Computes the running moving average for flexor (auxiliary variable)
        extensor_averagePart = (extensor + is * extensor_averagePart)/(is+1);   //Same for extensor
        is++;     //Increase the index for the computation
        if (is == controlRate) {  //After controlRate samples update the flexor and extensor averages, reset the auxiliary variables and the index  
            // Adjust the measured signals with user-selected Max Values
            flexor_average = amplifiedSignalValue(flexor_averagePart,MVF_DATA);
            extensor_average = amplifiedSignalValue(extensor_averagePart,MVE_DATA);
            // Reseting counting averages so that they can be used correctly in the next step
            flexor_averagePart = 0;
            extensor_averagePart = 0;   
            is = 0;
            CT10F = 1; //Set the flag that enable interfacing with the hand
        }     
        PIR1bits.TMR1IF = 0;        //Reset the timer flag
    }
    
    if(PIR1bits.RCIF){ // USART Reciver recived something
        USARTinput_data[iw] = RCREG;        // Read the incoming data
        USARTinput_val[iw] = 1;             // Set validity to WRITTEN
        iw = (iw+1) % USARTinput_length;    // Increase the write index in a circular fashion
        } 
}


void main(void) {
    //Call config functions
    config_DIO();
    config_interrupt();
    config_timer0();
    config_timer1();
    configUSART();
    configADC();

    //Set and initialize Variables
    unsigned char comState   = 0;       //Computer Mode State
    unsigned char firstByte  = 0;       //First, second, and third bytes creating a sequence for PC communication
    unsigned char secondByte = 0;
    unsigned char thirdByte  = 0;
    unsigned char forthbyte  = 0;       //This byte is an aux byte, which only used in streaming operation. It is the check if new command is given from the USART to break the streaming
    
    unsigned char S = 0;        // Sign of the movement (0: extensor/finger-open; 1: flexor/finger-close)
    unsigned int speed = 0;     // Speed value to be send to the hand (when in grasping states, i.e. 1,3,5)
    unsigned int speedExt = 0;  // Speed for extension
    unsigned int speedFlex = 0; // Speed for flexion
    
    unsigned char state = 0;            // Finite state machine state for grasping and preshape modes of the three stereotyped grasping. //Initialized as 0 (POWER grasping preshape) 
    unsigned char SCFLAG = 1;           // State Change Flag: when cocontraction is detected SCFLAG is set to one. When cocontraction ends, SCFLAG goes back to 0
    unsigned int Prealert = 0;          // When in states 0,2,4 && flexor_active==1 && extensor NOT active BUT higher than the low threshold, transition to Grasping is confirmed if no extensor activation is detected before Prealert>PrealertLimit (aims at confirmation of the grasping and prevents closing the hand when intention is switch preshape)
    unsigned int PrealertLimit = 100;   // Prealertlimit is in units of  (x 10 ms) 
    unsigned char extensor_active = 0;  // Extensor muscle activity (0:Inactive; 1: Active)
    unsigned char flexor_active = 0;    // Same for flexor
    
    //Initialize global variables (Defined above)
    CT10F = 0; 
    is = 0;
    deviceMode = 0;
    
    //Tasks at start
    MVE_DATA = read_EEprom(MVE_ADR);                  // Read last saved extensor max value
    MVF_DATA = read_EEprom(MVF_ADR);                  // Read last saved flexor max value
    THE_DATA = read_EEprom(THE_ADR);                  // Read last saved thresholds (extensor) 
    THF_DATA = read_EEprom(THF_ADR);                  // Read last saved thresholds (flexor) 
    MVE_DATA = adjustEEPROMdata('M',MVE_DATA);        // Adjust the read EEPROM data by assigning to their default values if needed
    MVF_DATA = adjustEEPROMdata('M',MVF_DATA);        // T and M stands for threshold and max value, respectively
    THE_DATA = adjustEEPROMdata('T',THE_DATA);        // It is used for the first reading after re-programming
    THF_DATA = adjustEEPROMdata('T',THF_DATA);        // Threshold defaults value is 50, whereas for max value is 100 (both represents percentage) for more info see the function in mylib
    thE =  percentage2threshold(THE_DATA);            // Converts to 10bit value
    thF =  percentage2threshold(THF_DATA);            // Same
    thE_h = thE;                                      // High threshold for Extensor
    thE_l = thE - hysteresis;                         // Low threshold for Extensor
    thF_h = thF;                                      // High threshold for Flexor
    thF_l = thF - hysteresis;                         // Low threshold for Flexor
    
    Reset_segments();           // Clears any inications in the LED Display
    sendUSART(0x46);            // FastCalibration
    //waitFastCalibration();    //THIS FUNCTION IS UNDER DEVELOPMENT FOR THE MOMENT THE FOLLOWING DELAY WILL DO A SIMILAR JOB
    Delay_ms(2000);
    GraspPreshape(state);       //Preshape the hand into the initialized preshape (POWER grasp or state 0)

    while(1){      
        switch(deviceMode){
            case 0: // HAND MODE            
                if (CT10F == 1){    // If 10 samples are collected (1 sample at 1ms therefore control rate 10ms = 10 samples)  
                    CT10F = 0;      // Reset the software flag so the system could get a new set of samples
                    
                    // Activeness/Inactiveness of the signals are assigned based on their coresponding values read from the sensors
                    if (extensor_average > thE_h && extensor_active == 0)       extensor_active = 1;
                    else if (extensor_average < thE_l && extensor_active == 1)  extensor_active = 0;                                
                    if (flexor_average > thF_h && flexor_active == 0)           flexor_active = 1;
                    else if (flexor_average < thF_l && flexor_active == 1)      flexor_active = 0; 
                    //Note that muscle group activaton is detected if the signal overcome the HIGH threshold and they become inactive if the signal falls below the LOW threshold 
                    
                    
                    // Assess whether co-contraction was detected in the previous cycle. This prevents consecutive preshape switches if muscles keep co-contracted.                  
                    if (SCFLAG == 1){
                        if (state == 0 || state == 2 || state == 4) {
                            if(conditionRelax(state) != 1) GraspPreshape(state);
                            Prealert = 0;
                        }
                        if (flexor_active == 0 && extensor_active == 0) { // User has to relax muscles to reset the flag and therefore enable actuation of the hand.      
                            SCFLAG=0;
                        }
                    }else{ // FINITE STATE MACHINE FOR PRESHAPES AND GRASPING if (SCFLAG == 0)
                        switch(state){
                            case 0: //Preshape Power 
                                INTCONbits.INT0IE = 1;      //Enables interrupt to switch to computer mode
                                Display_number(state);      //Displays state number on the LED display
                                if (flexor_active == 1 && extensor_active == 1){    //Co-contraction detected
                                    state = 2;              // Move on to Lateral preshape (cocontraction happened)
                                    SCFLAG = 1;             
                                }else if (flexor_active == 1){      // Flexor IS active bu extensor IS NOT. Transition to grasping is suspected but confirmation is required
                                    if (extensor_average < thE_l){  // Extensor signal is unlikely to be inentional so transition to the grasping state is confirmed.
                                        state = 1; //Closing of Power
                                        Prealert = 0;
                                    }else {                     //Even though extensor is INACTIVE, the signal is "barely" below the high threshold (thE_l<extensor_average<thE_h).
                                        Prealert++;             //The Prealert counter is increased to keep track of the time since the onset of the alert. If after successive cycles the same condition persists, Prealert will reach PrealertLimit and confirm the transition to state 1
                                        if (Prealert > PrealertLimit) { // Timeout of the prealert 
                                            state = 1;                  // State transition is confirmed 
                                            Prealert = 0;               // Reset of the prealert
                                        }
                                    }                           
                                }else Prealert = 0;
                                break;
                            case 1: // Grasping Power
                                disableModeSwitch();            //During grasping, system cannot switch to Computer mode for safety
                                Display_number(state);                                 
                                if ( flexor_active == 0 && extensor_active == 0){   // If no muscle activity is detected                                    
                                    if(conditionRelax(state) == 1) state = 0;       // If hand is in openmost position (preshape posture), then system goes back to the preshape state
                                    else speed = 0;                                 // The hand is not in the preshape position so the grasping position is mantained (zero speed).
                                }                      
                                else{ // Program determines which is the dominant muscle activity by comparing the magnitudes of the flexor and extensor computed speed
                                    speedFlex = velMapping(flexor_average,thF_l);   // flexor speed based on its respective activation level and thresholds
                                    speedExt = velMapping(extensor_average,thE_l);  // Same for extensor
                                    S = signFunc(speedFlex,speedExt);               // Comparison of the speeds. Direction with greater speed is chosen
                                    if(S == 1) speed = speedFlex;                   // If flexor is dominant then the magnitude of the speed fed to the hand is the flexor speed
                                    else       speed = speedExt;                    // If extensor is dominant then the magnitude of the speed fed to the hand is the extensor one
                                }                                                   //Note: Decision is done by comparing the speeds because the use of different thresholds for extensor and flexor  
                                                                                    //make direct comparison of flexor_average and extensor_average inconsistent (same numerical values of flexor_average and extensor_average may imply different activation levels).  
                                GraspControl(state, S, speed);                      //Send actuation instruction to the hand 
                                break;
                            case 2: //Preshape Lateral   //Comments are equivalent to those of case 0
                                INTCONbits.INT0IE = 1;
                                Display_number(state);
                                if (flexor_active == 1 && extensor_active == 1){
                                    state = 4; // Move on to Precision (cocontraction happened)
                                    SCFLAG = 1;
                                }else if (flexor_active == 1){
                                     if (extensor_average < thE_l){
                                        state = 3; //Closing of Lateral
                                        Prealert = 0;
                                    }
                                    else {
                                        Prealert++;
                                        if (Prealert>PrealertLimit){ 
                                            state = 3;
                                            Prealert = 0;
                                        }
                                    }                           
                                }else Prealert = 0;                                 
                                break;
                            case 3: //Grasping Lateral  //Comments are equivalent to those of case 1
                                disableModeSwitch();
                                Display_number(state);                                 
                                if (flexor_active == 0 && extensor_active == 0){                                    
                                    if(conditionRelax(state) == 1) state = 2; 
                                    else speed = 0;
                                }                      
                                else{
                                    speedFlex = velMapping(flexor_average,thF_l);
                                    speedExt = velMapping(extensor_average,thE_l);
                                    S = signFunc(speedFlex,speedExt);
                                    if(S==1) speed = speedFlex;
                                    else     speed = speedExt;
                                }
                                GraspControl(state, S, speed);                               
                                break;
                            case 4: //Preshape Precision //Comments are equivalent to those of case 0
                                INTCONbits.INT0IE = 1;
                                Display_number(state);
                                if (flexor_active == 1 && extensor_active == 1){
                                    state = 0; // Move on to Power (cocontraction happened)
                                    SCFLAG=1;
                                }else if (flexor_active == 1){
                                     if (extensor_average < thE_l){
                                        state = 5; //Closing of Precision
                                        Prealert = 0;
                                    }
                                    else {
                                        Prealert++;                                       
                                        if (Prealert>PrealertLimit) { 
                                            state=5;
                                            Prealert=0;
                                        }
                                    }                           
                                }else Prealert = 0;  
                                break;
                            case 5: //Grasping Precision //Comments are equivalent to those of case 1
                                disableModeSwitch();
                                Display_number(state);                                 
                                if (flexor_active == 0 && extensor_active == 0){                                    
                                    if(conditionRelax(state) == 1) state = 4; 
                                    else speed = 0;
                                }                      
                                else{
                                    speedFlex = velMapping(flexor_average,thF_l);
                                    speedExt = velMapping(extensor_average,thE_l);
                                    S = signFunc(speedFlex,speedExt);
                                    if (S==1) speed = speedFlex;
                                    else speed = speedExt;
                                 }
                                GraspControl(state, S, speed);                                                               
                                break;
                        }
                    }        
                }
                break;
            case 1: // COMPUTER MODE                
                switch(comState){        //Finite state machine for communication protocol
                    case 0:  // Idle state. Tries to receive a message until timeout. Decides if the received first byte corresponds to one of the protocols.
                        firstByte = readUSART_withTIMEOUT();
                        if(firstByte == thEbyte)      comState = 1;  //Instruction may be Write Extension threshold.
                        else if(firstByte == thFbyte) comState = 3;  //Instruction may be Write Flexion threshold.
                        else if(firstByte == Dbyte)   comState = 5;  //Instruction may be Read information.
                        else if(firstByte == Sbyte)   comState = 7;  //Instruction may be Stream Sensor Data.
                        else if(firstByte == mvEbyte) comState = 9;  //Instruction may be Write Extension MV.
                        else if(firstByte == mvFbyte) comState = 11; //Instruction may be Write Flexion MV.
                        break;
                    case 1:  // Pre-allert for treshold E.          //Reads second byte and proceed.
                        secondByte = readUSART_withTIMEOUT();
                        comState = 2;
                        break;
                    case 2:  // Ready for treshold E.               //Confirms the Writing of the Extension threshold if the protocol is correct.
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == thEbyte){
                            THE_DATA = secondByte;
                            write_EEprom(THE_ADR, THE_DATA);
                            TCF_E = 1; //Extensor Threshold changed
                        }
                        comState = 0;
                        break;
                    case 3:  // Pre-allert for treshold F.              //Analogue to case 1
                        secondByte = readUSART_withTIMEOUT();
                        comState = 4;
                        break;
                    case 4:  // Ready for treshold F.                   //Analogue to case 2 
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == thFbyte){
                            THF_DATA = secondByte;
                            write_EEprom(THF_ADR, THF_DATA);
                            TCF_F = 1; //Flexor Threshold changed
                        }
                        comState = 0;
                        break;
                    case 5: // Pre-allert for display                   //Analogue to case 1
                        secondByte = readUSART_withTIMEOUT();
                        comState = 6;
                        break;
                    case 6: // Ready for display                        //Analogue to case 2
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == Dbyte){
                            if(secondByte == thEbyte)       sendUSART(THE_DATA);
                            else if(secondByte == thFbyte)  sendUSART(THF_DATA);
                            else if(secondByte == mvEbyte)  sendUSART(MVE_DATA);
                            else if(secondByte == mvFbyte)  sendUSART(MVF_DATA);
                        }    
                        comState = 0;
                        break;
                    case 7: // Pre-allert for streaming information     //Analogue to case 1
                        secondByte = readUSART_withTIMEOUT();
                        comState = 8;
                        break;
                    case 8: // Ready for streaming information          //Analogue to case 2
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == Sbyte){
                            disableModeSwitch(); //Disabling device mode switch during streaming operation
                            do{ //Do until a stop command arrives from the input
                                blinkDisplayLetterC(); //blink the LED Display letter C (indicating system is streaming)
                                if(secondByte == thEbyte)       sendUSART_int(extensor_average); // Stream extensor information
                                else if(secondByte == thFbyte)  sendUSART_int(flexor_average);   // Stream flexor information
                                else if(secondByte == Sbyte)    sendUSART_int(absFunc(flexor_average,extensor_average));  // Stream difference between flexor and extesor
                                
                                forthbyte = readUSART_withTIMEOUT();    // The default value of readUSART_withTIMEOUT is 0. In other words, when the time elapse and there is no input it will return 0                     
                            }while(forthbyte != STPbyte); 
                            INTCONbits.INT0IE = 1; //Enabling back the switch interrupt
                        }
                        Display_letter('C');   //To stop blinking LED when streaming operation is terminated
                        comState = 0;
                        break;
                    case 9: // Pre-allert for Extensor Gain         //Analogue to case 1
                        secondByte = readUSART_withTIMEOUT();
                        comState = 10;
                        break;
                    case 10: // Ready for Extensor Gain             //Analogue to case 2
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == mvEbyte){
                            MVE_DATA = secondByte;
                            write_EEprom(MVE_ADR, MVE_DATA);
                        }
                        comState = 0;
                        break;
                    case 11: // Pre-allert for Flexor Gain          //Analogue to case 1
                        secondByte = readUSART_withTIMEOUT();
                        comState = 12;
                        break;
                    case 12: // Ready for Flexor Gain                //Analogue to case 2
                        thirdByte = readUSART_withTIMEOUT();
                        if(thirdByte == mvFbyte){
                            MVF_DATA = secondByte;
                            write_EEprom(MVF_ADR, MVF_DATA);
                        }
                        comState = 0;
                        break;
                }
                break;
        }
    }
}
