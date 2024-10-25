PROCESSOR 18F8722

#include <xc.inc>

; CONFIGURATION (DO NOT EDIT)
; CONFIG1H
CONFIG OSC = HSPLL      ; Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
CONFIG FCMEN = OFF      ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
CONFIG IESO = OFF       ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
; CONFIG2L
CONFIG PWRT = OFF       ; Power-up Timer Enable bit (PWRT disabled)
CONFIG BOREN = OFF      ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
; CONFIG2H
CONFIG WDT = OFF        ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
; CONFIG3H
CONFIG LPT1OSC = OFF    ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
CONFIG MCLRE = ON       ; MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
; CONFIG4L
CONFIG LVP = OFF        ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
CONFIG XINST = OFF      ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
CONFIG DEBUG = OFF      ; Disable In-Circuit Debugger


global count1
global count2
global runB
global runC
global prevE
global swapB
global swapC




; Define space for the variables in RAM
PSECT udata_acs
count1:
    DS 1 ; Allocate 1 byte for counter1
count2:
    DS 1
runB:
    DS 1
runC:
    DS 1
prevE:
    DS 1
swapB:
    DS 1
swapC:
    DS 1


PSECT resetVec,class=CODE,reloc=2
resetVec:
    goto       main

PSECT CODE
main:
    call init
    call busy_wait
    call busy_wait
    call disinit
    goto main_loop

init:
    clrf TRISB	; setting inputs and outputs
    clrf TRISC
    clrf TRISD
    setf TRISE
    
    setf PORTB	; setting default outputs
    setf PORTC
    setf PORTD
    
    movlw 20
    movwf count1
    clrf count2
    movlw 1
    
    clrf runB
    clrf runC
    clrf prevE
    clrf swapB
    clrf swapC
    
    return


busy_wait:
    infsnz count1
    return ; keep incrementing until it reaches 0
    loop2:
        infsnz count2
        goto busy_wait
        nop
        nop
        nop
        goto loop2


disinit:
    clrf PORTB
    clrf PORTC
    clrf PORTD
    return

    




; above stuff done, now only work on stuff below this comment





main_loop:
    
    movlw 176
    movwf count1
    call check_buttons_and_loop  
    
    movlw 1
    xorwf PORTD    ; update portD0
    
    call updateB
    call updateC
    
    goto main_loop




updateB:
    btfss runB, 0
    call runBis0
    btfsc runB, 0
    call runBis1
    return


runBis0:
    btfss swapB, 0
    return  ; if swapB is 0, return instantly
    setf runB
    clrf swapB
    return
    

runBis1:
    btfss swapB, 0
    call progressB   ; if swapB is 0, change value of B
    btfsc swapB, 0
    call disableB   ; if swapB is 1, disable B
    return


progressB:
    rlncf PORTB
    incf PORTB
    return

disableB:
    clrf PORTB
    clrf runB
    clrf swapB
    return




updateC:
    btfss runC, 0
    call runCis0
    btfsc runC, 0
    call runCis1
    return


runCis0:
    btfss swapC, 0
    return  ; if swapC is 0, return instantly
    setf runC
    clrf swapC
    return
    

runCis1:
    btfss swapC, 0
    call progressC   ; if swapC is 0, change value of C
    btfsc swapC, 0
    call disableC   ; if swapC is 1, disable C
    return


progressC:
    btfsc PORTC, 0  ; if portc is 255, reset it to 0
    goto resetC
    goto advanceC

advanceC:
    rrncf PORTC
    bsf PORTC, 7
    return
    

resetC:
    clrf PORTC
    return

disableC:
    clrf PORTC
    clrf runC
    clrf swapC
    return






check_buttons_and_loop:
    infsnz count1
    return ; keep incrementing until it reaches 0
    loop3:
        infsnz count2
        goto check_buttons_and_loop
    btfss prevE, 0
    call prevE0is0
    btfsc prevE, 0
    call prevE0is1
    
    btfss prevE, 1
    call prevE1is0
    btfsc prevE, 1
    call prevE1is1
    
    goto loop3


prevE0is0:  ; if prevE0 is 0, and E0 changes to 1, change prevE0 to 1
    btfsc PORTE, 0
    bsf prevE, 0
    return
    
prevE0is1:  ; if prevE0 is 1, and E0 changes to 0, change prevE0 to 0 AND update swapC
    btfsc PORTE, 0
    return
    bcf prevE, 0
    setf swapC
    return
    

prevE1is0:
    btfsc PORTE, 1
    bsf prevE, 1
    return
    
prevE1is1:
    btfsc PORTE, 1
    return
    bcf prevE, 1
    setf swapB
    return




end resetVec