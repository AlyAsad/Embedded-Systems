/*Group 21
 * 
 *
 * Aly Asad Gilani
 * Affan Ahmed
 * Syed Ebad Hyder
 * 
 * 
 * Our design is based on Uluc Hoca's implementation of serial communication.
 * At first, we initialized the tristate registers necessary for 
 * input and outputs. Then we initialized the UART to enable the transmission.
 * Then we used the recitation code to initialize the Analog to
 *  Digital Converter. Then we read the slides and datasheet to initialize the 
 * ADCON interrupt, and how  to handle the interrupt. Then we initialized the 
 * other interrupts and set the timers to display the outputs for every 
 * interval. We used the GetCommand() function to take the command from the 
 * serial input. Then we processed the taken command accordingly and switched 
 * the modes of the simulation according to the commands. The PORTB interrupts
 * and ADC interrupts are handled in a high priority interrupt handler. Whenever 
 * we were in manual mode, we could set GODONE bit to 1 to start the ADC 
 * conversion and whenever we left it we could set it to 0. In the interrupt 
 * we could return it to 1 as soon as the previous conversion finished to 
 * keep reading the value.
 */

#ifndef PRAGMAS_H
#define	PRAGMAS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <p18cxxx.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#define _XTAL_FREQ   40000000

#pragma config  OSC = HSPLL, FCMEN = OFF, IESO = OFF
#pragma config  PWRT = OFF, BOREN = OFF, BORV = 3
#pragma config  WDT = OFF, WDTPS = 32768
#pragma config  MODE = MC, ADDRBW = ADDR20BIT, DATABW = DATA16BIT, WAIT = OFF
#pragma config  CCP2MX = PORTC, ECCPMX = PORTE, LPT1OSC = OFF, MCLRE = ON
#pragma config  STVREN = ON, LVP = OFF, BBSIZ = BB2K, XINST = OFF
#pragma config  CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF, CP4 = OFF, CP5 = OFF
#pragma config  CP6 = OFF, CP7 = OFF
#pragma config  CPB = OFF, CPD = OFF
#pragma config  WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF, WRT4 = OFF
#pragma config  WRT5 = OFF, WRT6 = OFF, WRT7 = OFF
#pragma config  WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config  EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTR4 = OFF
#pragma config  EBTR5 = OFF, EBTR6 = OFF, EBTR7 = OFF
#pragma config  EBTRB = OFF
#pragma config DEBUG = ON


#ifdef	__cplusplus
}
#endif

#endif	/* PRAGMAS_H */


void DistanceResponse();
void AltitudeResponse();
void ButtonPressResponse();

// To avoid race conditions
inline void DisableUART() {PIE1bits.RC1IE = 0; PIE1bits.TX1IE = 0;}
inline void EnableUART()  {PIE1bits.RC1IE = 1; PIE1bits.TX1IE = 1;}

// Input and output buffers for serial communication
typedef enum {INPUT = 0, OUTPUT = 1} Buffer;


#define BUFSIZE 128        /* Static buffer size. Maximum amount of data */
uint8_t inputBuffer[BUFSIZE];   /* Preallocated buffer for incoming data */
uint8_t outputBuffer[BUFSIZE];  /* Preallocated buffer for outgoing data  */
uint8_t head[2] = {0, 0}; /* head for pushing, tail for popping */
uint8_t tail[2] = {0, 0};


/* Check if a buffer had data or not */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_isempty( Buffer buffer ) {
  if (head[buffer] == tail[buffer]) return 1;
  return 0;
}

/* Place new data in buffer */
#pragma interrupt_level 2 // Prevents duplication of function
void buf_push( uint8_t v, Buffer buffer) {
    uint8_t next;

    if (buffer == INPUT) inputBuffer[head[buffer]] = v;
    else outputBuffer[head[buffer]] = v;
    next = head[buffer] + 1;

    if (next == BUFSIZE) next = 0;
    if (next != tail[buffer]) head[buffer] = next;
}

/* Retrieve data from buffer */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_pop( Buffer buffer ) {
    uint8_t v = 0;
    if (!buf_isempty(buffer)) {
        if (buffer == INPUT) v = inputBuffer[tail[buffer]];
        else v = outputBuffer[tail[buffer]];
        tail[buffer]++;
        if (tail[buffer] == BUFSIZE) tail[buffer] = 0;
    }
    return v;
}


// Functions
void DistanceResponse();
void AltitudeResponse();
void ButtonPressResponse();



// Global Variables
// Max size of a command is 7
uint8_t CurrCommand[7];
uint8_t CurrIndex = -1;
int validCommand = 0;

// 0 = IDLE, 1 = ACTIVE, 2 = END
int CurrMode = 0;
int RemainingDistance = 0;
int Speed = 0;

int AltitudePeriod = 0;
int IsAltitudeControl = 0;
int AltitudeResponseCounter = -1;
int AltitudeResponseMaxCounter = -1;

int IsManual = 0;
int Led = -1;

int DistFlag = 0;
int EndFlag = 0;

int prevB, tempB;
int pressedB4, pressedB5, pressedB6, pressedB7;
int turnOnB4 = 0, turnOnB5 = 0, turnOnB6 = 0, turnOnB7 = 0;
int sendManual = 0;

unsigned int ADC_Result;

void InitializePorts() {
  TRISA = 0x00; PORTA = 0x00; LATA = 0x00;
  TRISB = 0xF0; PORTB = 0x00; LATB = 0x00;
  TRISC = 0x00; PORTC = 0x00; LATC = 0x00;
  TRISD = 0x00; PORTD = 0x00; LATD = 0x00;

  TRISCbits.RC6 = 0;
  TRISCbits.RC7 = 1;

}

void InitializeUART() {
  // Transmit enabled and high speed
  TXSTA1 = 0x24;

  // Serial port enabled and continuous receive enabled
  RCSTA1 = 0x90;

  SPBRG = 0x15;
  BAUDCONbits.BRG16 = 0;
}

void InitializeInterrupt() {
  // Enable RC1IE interrupt
  PIR1 = 0;
  PIE1bits.RCIE = 1;

  INTCONbits.RBIE = 1;

  EnableUART();
  INTCONbits.PEIE = 1; // Enable peripheral interrupts
  INTCONbits.GIE = 1;  // Enable global interrupts
}

void InitializeTimer() {
  T0CON = 0x87;
  //TMR0 = 0xC2F6;
  TMR0 = 0xF0BD;
}


void Transmit() {
  if (buf_isempty(OUTPUT)) PIE1bits.TX1IE = 0;
  else TXREG1 = buf_pop(OUTPUT);
}

void Recieve() {
  PIR1bits.RC1IF = 0;
  buf_push(RCREG1, INPUT);
}

__interrupt(high_priority)
void InterruptHandler() {

    // Timer Overflow Interrupt
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0 = 0xF0BD;

        if(DistFlag) DistanceResponse();
        
        if(IsAltitudeControl) {
            AltitudeResponseCounter = (AltitudeResponseCounter + 1) % AltitudeResponseMaxCounter;
            if(AltitudeResponseCounter == 0) AltitudeResponse();
        } 
        
        if(IsManual && sendManual && (pressedB4 || pressedB5 || pressedB6 || pressedB7)) ButtonPressResponse();
    }

    
    // Transmit and Receive Interrupt
    if (PIR1bits.TX1IF) Transmit();
    if (PIR1bits.RC1IF) Recieve();
    
    // ADC Interrupt
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        ADC_Result = (ADRESH << 8) + ADRESL;
        GODONE = 1;
    }
    
    // PORTB Interrupt for Manual Mode
    if(INTCONbits.RBIF) {
        //read value to satisfy the interrupt
        tempB = PORTB;
        INTCONbits.RBIF = 0;

        //rest of the stuff
        if (turnOnB7 && ((~prevB & tempB) | (prevB & ~tempB)) & 0b10000000) {
            turnOnB7 = 0;
            pressedB7 = 1;
        }
        if (turnOnB6 && ((~prevB & tempB) | (prevB & ~tempB)) & 0b01000000) {
            turnOnB6 = 0;
            pressedB6 = 1;
        }
        
        if (turnOnB5 && ((~prevB & tempB) | (prevB & ~tempB)) & 0b00100000) {
            pressedB5 = 1;
            turnOnB5 = 0;
        }
        if (turnOnB4 && ((~prevB & tempB) | (prevB & ~tempB)) & 0b00010000) {
            turnOnB4 = 0;
            pressedB4 = 1;
        }    

        prevB = tempB;
    }
  
}



void GetCommand() {
    DisableUART();

    while(!buf_isempty(INPUT)) {
    DistFlag = 1;
        uint8_t temp;

        temp = buf_pop(INPUT);

        if (temp == '$') CurrIndex = 0;
        else if (temp == '#') {
            validCommand = 1;
            break;
        }
        else if (isdigit(temp) || isalpha(temp)) {
            CurrCommand[CurrIndex] = temp;
            CurrIndex++;
        }
        else break;
    }

    EnableUART();
}

void EvaluateCommand() {
    if (!validCommand) return;
    validCommand = 0;
    
    // IDLE
    if (CurrMode == 0) {
        if (CurrCommand[0] == 'G' && CurrCommand[1] == 'O' && CurrCommand[2] == 'O') {
            sscanf((char *)CurrCommand+3, "%04x", &RemainingDistance);
            INTCONbits.TMR0IE = 1; //Enable Timer
            CurrMode = 1;
        }
    }

    // ACTIVE
    else if(CurrMode == 1) {
        // END Command
        if (CurrCommand[0] == 'E' && CurrCommand[1] == 'N' && CurrCommand[2] == 'D') CurrMode = 2;
        
        // SPD Command
        else if (CurrCommand[0] == 'S' && CurrCommand[1] == 'P' && CurrCommand[2] == 'D') {
            sscanf((char *)CurrCommand+3, "%04x", &Speed);
            RemainingDistance -= Speed;
        }
        
        // ALT Command
        else if (CurrCommand[0] == 'A' && CurrCommand[1] == 'L' && CurrCommand[2] == 'T') {
            sscanf((char *)CurrCommand+3, "%04x", &AltitudePeriod);
            // TODO
            
            if(AltitudePeriod == 0) {
                GODONE = 0;
                IsAltitudeControl = 0;
                AltitudeResponseMaxCounter = -1;
            }
            else {
                GODONE = 1;
                IsAltitudeControl = 1;
                AltitudeResponseCounter = 0;
                
                if(AltitudePeriod == 200) AltitudeResponseMaxCounter = 2;                
                else if(AltitudePeriod == 400) AltitudeResponseMaxCounter = 4;                      
                else if(AltitudePeriod == 600) AltitudeResponseMaxCounter = 6;
            }
            
        }
        
        // MAN Command
        else if (CurrCommand[0] == 'M' && CurrCommand[1] == 'A' && CurrCommand[2] == 'N') {
            // TODO 
            if (CurrCommand[3] == '0' && CurrCommand[4] == '1') {
                prevB = PORTB;
                IsManual = 1;
            }
            else if(CurrCommand[3] == '0' && CurrCommand[4] == '0') {
                IsManual = 0;
            }  
        }
        
        // LED Command
        else if (CurrCommand[0] == 'L' && CurrCommand[1] == 'E' && CurrCommand[2] == 'D') {
            // sscanf((char *)CurrCommand+3, "%04x", &Led);
            // TODO
            
            if(!IsManual) return;
            // $LED00#
            // Clear all LEDs
            if(CurrCommand[3] == '0' && CurrCommand[4] == '0') {                 
                
                sendManual = 0;
                
                LATA = LATA & (0xFE);
                LATB = LATB & (0xFE);
                LATC = LATC & (0xFE);
                LATD = LATD & (0xFE);
            }
            
            // $LED01#
            // Turn on RD0
            else if(CurrCommand[3] == '0' && CurrCommand[4] == '1') {
                sendManual = 1;
                LATD = LATD | (0x01);
                turnOnB4 = 1;
            }
            
            // $LED02#
            // Turn on RC0
            else if(CurrCommand[3] == '0' && CurrCommand[4] == '2') {
                sendManual = 1;
                LATC = LATC | (0x01);
                turnOnB5 = 1;
            }
            
            // $LED03#
            // Turn on RB0
            else if(CurrCommand[3] == '0' && CurrCommand[4] == '3') {
                sendManual = 1;
                LATB = LATB | (0x01);
                turnOnB6 = 1;
            }            
            
            // $LED04#
            // Turn on RA0
            else if(CurrCommand[3] == '0' && CurrCommand[4] == '4') {
                sendManual = 1;
                LATA = LATA | (0x01);
                turnOnB7 = 1;
            }            
        }
    }

    // END
    else {
        CurrMode = 0;
        INTCONbits.TMR0IE = 0; // Disable Timer
        EndFlag = 1;
    }

}

void DistanceResponse() {
    DisableUART();

    uint8_t Distance[4];
    sprintf(Distance, "%04x", RemainingDistance);

    buf_push('$', OUTPUT);
    buf_push('D', OUTPUT);
    buf_push('S', OUTPUT);
    buf_push('T', OUTPUT);
    buf_push(Distance[0], OUTPUT);
    buf_push(Distance[1], OUTPUT);
    buf_push(Distance[2], OUTPUT);
    buf_push(Distance[3], OUTPUT);
    buf_push('#', OUTPUT);

    EnableUART();
    TXSTA1bits.TXEN = 1;
}

void AltitudeResponse() {
    DisableUART();
    
    int CurrAltitude;
    uint8_t Altitude[4];
    
    
    if(ADC_Result >= 768 && ADC_Result <= 1023)
        CurrAltitude = 12000;
   
    else if(ADC_Result >= 512 && ADC_Result < 768)
        CurrAltitude = 11000;
    
    else if(ADC_Result >= 256 && ADC_Result < 512)
        CurrAltitude = 10000;
    
    else if(ADC_Result >= 0 && ADC_Result < 256)
        CurrAltitude = 9000;
    
    sprintf(Altitude, "%04x", CurrAltitude);
    
    
    buf_push('$', OUTPUT);
    buf_push('A', OUTPUT);
    buf_push('L', OUTPUT);
    buf_push('T', OUTPUT);
    buf_push(Altitude[0], OUTPUT);
    buf_push(Altitude[1], OUTPUT);
    buf_push(Altitude[2], OUTPUT);
    buf_push(Altitude[3], OUTPUT);
    buf_push('#', OUTPUT);
    
    
    EnableUART();
    TXSTA1bits.TXEN = 1;
}

void ButtonPressResponse() {
    DisableUART();
    buf_push('$', OUTPUT);
    buf_push('P', OUTPUT);
    buf_push('R', OUTPUT);
    buf_push('S', OUTPUT);
    buf_push('0', OUTPUT);        
    
    if(pressedB4) {
        pressedB4 = 0;
        buf_push('4', OUTPUT);
    }    
    else if(pressedB5) {
        pressedB5 = 0;
        buf_push('5', OUTPUT);
    }
    else if(pressedB6) {
        pressedB6 = 0;
        buf_push('6', OUTPUT);
    }
    else if(pressedB7) {
        pressedB7 = 0;
        buf_push('7', OUTPUT);
    }
    
    buf_push('#', OUTPUT);    
    
    EnableUART();
    TXSTA1bits.TXEN = 1;
}

void InitailizeADC() {
    TRISH = 0x10;
            
    ADCON0 = 0x31;
    ADCON1 = 0x00;
    ADCON2 = 0xAA;
    ADRESH = 0x00;
    ADRESL = 0x00;
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
}


void main() {
    InitializePorts();
    InitializeUART();
    InitailizeADC();
    InitializeInterrupt();
    InitializeTimer();

    while(!EndFlag) {
        GetCommand();
        EvaluateCommand();
    }
    return;
}