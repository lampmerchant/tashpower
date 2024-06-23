;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashPower: PS/2 to ADB Converter and Power Relay Controller
;;;
;


;;; Connections ;;;

;;;                                                           ;;;
;                            .--------.                         ;
;                    Supply -|01 \/ 08|- Ground                 ;
;              ADB <--> RA5 -|02    07|- RA0 <--- PS/2 Data     ;
;    ADB Power Key ---> RA4 -|03    06|- RA1 <--- PS/2 Clock    ;
;       Line Clock ---> RA3 -|04    05|- RA2 ---> Relay         ;
;                            '--------'                         ;
;                                                               ;
;    Relay output is active high.                               ;
;                                                               ;
;;;                                                           ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1501, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1501.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	errorlevel	-224	;Suppress TRIS instruction not recommended msgs
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
	__config	_CONFIG2, _WRT_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF &_LVP_OFF
			;_WRT_OFF	Write protection off
			;_STVREN_ON	Stack over/underflow causes reset
			;_BORV_LO	Brownout reset voltage low trip point
			;_LPBOR_OFF	Low power brownout reset disabled
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

AR_ADDR	equ	0x7	;Relay default address
AR_HDLR	equ	0x22	;Relay handler ID
AR_VERS	equ	0x3	;Relay firmware version (1.4)

AK_NONE	equ	0x7E	;Placeholder for when there is no ADB key
AK_ACC	equ	0x39	;Caps lock ADB key code

;Pin Assignments:
ADB_PIN	equ	RA5	;Pin where ADB is connected
APK_PIN	equ	RA4	;Pin where ADB power key line is connected
LCK_PIN	equ	RA3	;Pin where line clock (50/60 Hz) is connected
RLY_PIN	equ	RA2	;Pin where relay is connected
PCK_PIN	equ	RA1	;Pin where PS/2 clock is connected

;FLAGS:
AU_TXON	equ	7	;Set when the selected device wants TX events
AU_SEKB	equ	6	;Set when SRQ is enabled for keyboard
AU_SRKB	equ	5	;Set when the keyboard is requesting service
LC_IOCF	equ	4	;Set when the line clock pulses
PK_IOCF	equ	3	;Set when ADB power key line goes low
AK_CAPS	equ	2	;Set when ADB caps lock key is on (down)

;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too

;PP_FLAG:
PP_IOCF	equ	7	;Set if the PS/2 clock went low
PP_PAR	equ	6	;Set if the PS/2 peripheral expects a 1 parity bit
PK_F0	equ	5	;Set if the PS/2 keyboard sent an 0xF0
PK_E0	equ	4	;Set if the PS/2 keyboard sent an 0xE0
PK_E1	equ	3	;Set if the PS/2 keyboard sent an 0xE1
PK_IGNX	equ	2	;Set if the PS/2 keyboard should ignore next keystroke

;AR_FLAG:
AR_COL	equ	7	;Set if there was an ADB collision on relay's address
AR_R1OF	equ	6	;Set if relay was closed because of register 1 overflow
AR_PKEY	equ	5	;Set if relay was closed because of power key press
AR_ADR3	equ	3	;Relay ADB address
AR_ADR2	equ	2	; "
AR_ADR1	equ	1	; "
AR_ADR0	equ	0	; "

;AR_R0H:
AR_TLK0	equ	0	;Set if relay responds to Talk 0

;AK_R2H:
K2H_DEL	equ	6	;Delete
K2H_CAP	equ	5	;CapsLock
K2H_RST	equ	4	;Reset
K2H_CTL	equ	3	;Control
K2H_SHF	equ	2	;Shift
K2H_OPT	equ	1	;Option
K2H_CMD	equ	0	;Command

;AK_R2L:
K2L_CLR	equ	7	;Clear
K2L_SLK	equ	6	;ScrollLock

;AK_MOD:
KMD_LCT	equ	7	;Left Control
KMD_RCT	equ	6	;Right Control
KMD_LSH	equ	5	;Left Shift
KMD_RSH	equ	4	;Right Shift
KMD_LOP	equ	3	;Left Option
KMD_ROP	equ	2	;Right Option

;AK_R3H:
K3H_COL	equ	7	;Collision flag (not part of ADB standard)
K3H_EXE	equ	6	;Exceptional event flag
K3H_SRE	equ	5	;Service request enable flag
K3H_AD3	equ	3	;Keyboard ADB address
K3H_AD2	equ	2	; "
K3H_AD1	equ	1	; "
K3H_AD0	equ	0	; "


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB FSA
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	PP_FLAG	;PS/2 peripheral flags
	PP_FSAP	;Pointer to where to resume PS/2 peripheral state machine
	PP_SR	;PS/2 peripheral shift register
	AU_FSAP	;Pointer to where to resume device state machine
	AU_TEMP	;Temporary holding variables between states
	AU_TMP2	; "
	AU_TMP3	; "
	X2
	X1
	X0
	
	endc

	cblock	0x40	;High 16 bytes of bank 0 registers
	
	AK_LAST	;Last ADB keyboard key code (to suppress repeat)
	AK_R2H	;Register 2 high byte
	AK_R2L	;Register 2 low byte
	AK_MOD	;Keyboard modifier key state
	AK_R3H	;Register 3 high byte
	AK_R3L	;Register 3 low byte
	AR_FLAG	;Relay flags
	AR_R0H	;Relay register 0 high byte
	AR_R1_3	;Relay register 1 (relay close timer)
	AR_R1_2	; "
	AR_R1_1	; "
	AR_R1_0	; "
	AR_R2_3	;Relay register 2 (relay open timer)
	AR_R2_2	; "
	AR_R2_1	; "
	AR_R2_0	; "
	
	endc

	;Linear memory:
	;0x2000-0x201F - ADB keyboard queue (FSR1 = push, FSR0 = pop)
	;0x2020-0x202F - High 16 bytes of bank 0 registers


;;; Vectors ;;;

	org	0x0		;Reset vector

	banksel	OSCCON		;Select 16 MHz high-freq internal oscillator
	movlw	B'01111000'
	movwf	OSCCON

	goto	Init		;Join full init routine below


	org	0x4		;Interrupt vector
	;fall through


;;; Interrupt Handler ;;;

Interrupt
	bcf	STATUS,C	;Copy the Timer0 flag into the carry bit so it
	btfsc	INTCON,TMR0IF	; doesn't change on us mid-stream
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;Grab the current interrupt-on-change flags and
	movf	IOCAF,W		; clear bits that were set
	xorwf	IOCAF,F		; "
	btfsc	WREG,PCK_PIN	;Set flag in PS/2 flag register if there was a
	bsf	PP_FLAG,PP_IOCF	; falling edge on PS/2 clock
	btfsc	WREG,LCK_PIN	;Set flag in flag register if there was a
	bsf	FLAGS,LC_IOCF	; falling edge on line clock
	btfsc	WREG,APK_PIN	;Set flag in flag register if there was a
	bsf	FLAGS,PK_IOCF	; falling edge on ADB power key line
	btfsc	WREG,ADB_PIN	;If the ADB pin has had a negative or positive
	call	IntAdbEdge	; edge, handle it as an event for the ADB state
	retfie			; machine

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsaIdle	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADB_PIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,ADB_PIN	;Clear the interrupt flag
	btfsc	IOCAN,ADB_PIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsaIdle	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsaIdle	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	movlw	low AdbFsaIdle	;Reset the ADB state machine
	movwf	AP_FSAP		; "
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return


;;; Hardware Initialization ;;;

Init
	movlw	B'01010011'	;Timer0 uses instruction clock, 1:16 prescaler,
	movwf	OPTION_REG	; thus ticking every 4 us; weak pull-ups on

	banksel	IOCAN		;ADB, PS/2 clock, and line clock pins set IOCAF
	movlw	(1 << ADB_PIN)|(1 << PCK_PIN)|(1 << LCK_PIN); on negative edge
	movwf	IOCAN

	banksel	T1CON		;Timer1 uses instruction clock
	bsf	T1CON,TMR1ON
	
	movlw	B'00011110'	;Timer2 has 1:16 prescaler and 1:4 postscaler,
	movwf	T2CON		; thus overflowing after 4.096 ms

	banksel	CLC2CON		;CLC2 is a DFF which clocks in data from the
	clrf	CLC2SEL0	; PS/2 data pin (CLC2IN1) on the falling edge of
	movlw	B'01010000'	; the PS/2 clock pin (CLC2IN0)
	movwf	CLC2SEL1
	movlw	B'00000001'
	movwf	CLC2POL
	movlw	B'00000010'
	movwf	CLC2GLS0
	movlw	B'10000000'
	movwf	CLC2GLS1
	clrf	CLC2GLS2
	clrf	CLC2GLS3
	movlw	B'10000100'
	movwf	CLC2CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	movlw	high KeyLut	;Point high flash memory control address reg to
	movwf	PMADRH		; key translation lookup table

	banksel	LATA		;ADB pin ready to be pulled low, default state
	clrf	LATA		; of relay is off (low)

	banksel	TRISA		;Relay pin is an output
	bcf	TRISA,RLY_PIN

	movlw	0x20		;Initialize key globals
	movwf	FSR0H
	movwf	FSR1H
	clrf	FSR0L
	clrf	FSR1L
	movlw	low AdbFsaIdle
	movwf	AP_FSAP
	clrf	AP_FLAG
	clrf	AU_FSAP
	clrf	FLAGS
	movlw	low Ps2FsaStart
	movwf	PP_FSAP
	clrf	PP_FLAG

	movlw	B'10001000'	;On-change interrupt and interrupt subsystem on
	movwf	INTCON

	;fall through


;;; Mainline ;;;

Main
	call	SvcAdb		;Service the ADB peripheral
	call	SvcPs2		;Service the PS/2 peripheral
	movlb	0		;If ADB power key line went low, close relay
	btfsc	FLAGS,PK_IOCF	; "
	call	RelayPowerKey	; "
	btfsc	FLAGS,LC_IOCF	;If line clock pulsed, tick the relay timers
	call	TickRelayTimers	; "
	bra	Main		;Loop

SvcAdb
	movlb	0		;If the reset flag is set, handle it
	btfsc	AP_FLAG,AP_RST	; "
	call	SvcAdbReset	; "
	btfsc	AP_FLAG,AP_RXCI	;If the received-command flag is set, handle it
	call	SvcAdbCommand	; "
	btfss	AP_FLAG,AP_RXDI	;If either the received-data or collision flag
	btfsc	AP_FLAG,AP_COL	; is set, handle it
	call	SvcAdbData	; "
	btfsc	AP_FLAG,AP_DONE	;If the transmission-done flag is set, handle
	call	SvcAdbData	; it
	btfss	FLAGS,AU_TXON	;If the transmit-buffer-ready flag is set and
	return			; the state machine wishes to be notified of
	btfss	AP_FLAG,AP_TXI	; this, handle it
	call	SvcAdbData	; "
	return			; "

SvcAdbReset
	movlw	AR_ADDR		;Relay register 3 puts it at address 0x7, all
	movwf	AR_FLAG		; flags clear
	clrf	AR_R0H		;Clear register 0 high byte
	clrf	AR_R1_3		;Reset relay timers
	clrf	AR_R1_2		; "
	clrf	AR_R1_1		; "
	clrf	AR_R1_0		; "
	clrf	AR_R2_3		; "
	clrf	AR_R2_2		; "
	clrf	AR_R2_1		; "
	clrf	AR_R2_0		; "
	movlw	0x62		;Keyboard register 3 puts it at address 0x2,
	movwf	AK_R3H		; with SRQ enabled and handler ID 2 (Extended
	movlw	0x02		; Keyboard)
	movwf	AK_R3L		; "
	movlw	B'11111111'	;Keyboard register 2 has all modifier keys up
	movwf	AK_R2H		; and reserved bits set to 1
	movwf	AK_R2L		; "
	movwf	AK_MOD		; "
	bsf	FLAGS,AU_SEKB	;Set the SRQ enable bits for the keyboard
	bcf	AP_FLAG,AP_RST	;Clear the reset flag, if it was set
	return

SvcAdbCommand
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_TXI	;Ensure we don't accidentally echo the command
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	SvcAdbReset	; "
	call	SvcAdC1		;Attempt to match the command's address
	movlp	high AUFsaIgnore;Call into the initial state of the selected
	callw			; device's state machine
	movwf	AU_FSAP		;On returning, save the address returned in W
	;TODO the device being serviced should not call for SRQ
SvcAdC0	bsf	AP_FLAG,AP_SRQ	;If the keyboard is calling for service and its
	btfsc	FLAGS,AU_SEKB	; SRQ enable bit is high, set the SRQ flag for
	btfss	FLAGS,AU_SRKB	; the ADB peripheral
	bcf	AP_FLAG,AP_SRQ	; "
	return
SvcAdC1	swapf	AK_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the keyboard, load the start state
	andlw	B'11110000'	; of the keyboard's state machine
	btfsc	STATUS,Z	; "
	retlw	low AKFsaCommand; "
	swapf	AR_FLAG,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the relay, load the state state of
	andlw	B'11110000'	; the relay's state machine
	btfsc	STATUS,Z	; " 
	retlw	low ARFsaCommand; "
	retlw	low AUFsaIgnore	;No device matched the given address

SvcAdbData
	movlp	high AUFsaIgnore;Resume the ADB user program state machine
	movf	AU_FSAP,W	; "
	callw			; "
	movwf	AU_FSAP		;On returning, save the address returned in W
SvcAdD0	bcf	AP_FLAG,AP_RXDI	;Clear the flags that could have brought us
	bcf	AP_FLAG,AP_COL	; here
	bcf	AP_FLAG,AP_DONE	; "
	return

SvcPs2
	movlb	0		;If we got a negative edge on the PS/2 clock
	btfsc	PP_FLAG,PP_IOCF	; pin, handle the bit that came in
	bra	SvcPs20		; "
	btfsc	PIR1,TMR2IF	;If Timer2 interrupted, reset the PS/2 state
	bra	SvcPs21		; machine
	return			;Else, return
SvcPs20	bcf	PP_FLAG,PP_IOCF	;Clear the interrupt
	clrf	TMR2		;Reset Timer2 as we've received a bit
	bcf	PIR1,TMR2IF	; "
	movlb	30		;Copy the received bit into carry
	lsrf	CLCDATA,W	; "
	lsrf	WREG,W		; "
	movlp	high Ps2FsaStart;Resume the PS/2 peripheral state machine
	movf	PP_FSAP,W	; "
	callw			; "
	movwf	PP_FSAP		;On returning, save the address returned in W
	return			; "
SvcPs21	bcf	PIR1,TMR2IF	;Clear the interrupt
	movlw	low Ps2FsaStart	;Reset the PS/2 peripheral state machine
	movwf	PP_FSAP		; "
	clrf	PP_FLAG		; "
	return			;Return

TickRelayTimers
	bcf	FLAGS,LC_IOCF	;Clear interrupt
	movf	AR_R1_3,F	;If register 1's MSB is zero, we leave it alone
	btfsc	STATUS,Z	; "
	bra	TRTime0		; "
	incf	AR_R1_0,F	;Increment register 1, carrying through all four
	btfsc	STATUS,Z	; bytes
	incf	AR_R1_1,F	; "
	btfsc	STATUS,Z	; "
	incf	AR_R1_2,F	; "
	btfsc	STATUS,Z	; "
	incf	AR_R1_3,F	; "
	btfss	STATUS,Z	;If it didn't overflow, skip ahead
	bra	TRTime0		; "
	btfsc	PORTA,RLY_PIN	;If the relay is already closed, skip ahead
	bra	TRTime0		; "
	bsf	AR_FLAG,AR_R1OF	;Flag that the relay was closed by an overflow
	movlb	2		; of register 1 and close the relay
	bsf	LATA,RLY_PIN	; "
TRTime0	movlb	0		;If register 2's MSB is zero, we leave it alone
	movf	AR_R2_3,F	; "
	btfsc	STATUS,Z	; "
	return			; "
	incf	AR_R2_0,F	;Increment register 2, carrying through all four
	btfsc	STATUS,Z	; bytes
	incf	AR_R2_1,F	; "
	btfsc	STATUS,Z	; "
	incf	AR_R2_2,F	; "
	btfsc	STATUS,Z	; "
	incf	AR_R2_3,F	; "
	btfss	STATUS,Z	;If it didn't overflow, we're done
	return			; "
	btfss	PORTA,RLY_PIN	;If the relay is already open, we're done
	return			; "
	bcf	AR_FLAG,AR_R1OF	;Clear the flags that give the cause of the
	bcf	AR_FLAG,AR_PKEY	; relay closing
	movlb	2		;Open the relay
	bcf	LATA,RLY_PIN	; "
	return			;Done


;;; State Machines ;;;

	org	0x100

AUFsaIgnore
	retlw	low AUFsaIgnore	;Utility state to ignore until next command

AKFsaCommand
	btfsc	AP_BUF,2	;Talk is the only command that sets bit 2
	bra	AKFsaTalk	; "
	btfsc	AP_BUF,3	;Only talk and listen set bit 3, so if it's not
	bra	AKFsaListen	; talk, it's listen
	retlw	low AUFsaIgnore	;Ignore flush

AKFsaListen
	btfss	AP_BUF,1	;Keyboard only responds to listens on registers
	retlw	low AUFsaIgnore	; 2 and 3
	btfss	AP_BUF,0	;Go to the appropriate state and wait for a
	retlw	low AKFsaLstn2H	; data byte
	retlw	low AKFsaLstn3H	; "

AKFsaLstn2H
	retlw	low AKFsaLstn2L	;Keyboard listen 2 only has effect on low byte

AKFsaLstn2L
	;TODO mechanism to signal something that LEDs have changed?
	movlw	B'11111000'	;Accept the state of the keyboard LEDs from the
	andwf	AK_R2L,F	; listen command but ignore all other bits
	movlw	B'00000111'	; "
	andwf	AP_BUF,W	; "
	iorwf	AK_R2L,F	; "
	retlw	low AUFsaIgnore	; "

AKFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AU_TEMP		; what the low byte (handler ID) is, so store
	retlw	low AKFsaLstn3L	; it in temporary space

AKFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	AKFL3L1		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	AKFL3L0		; detected
	addlw	-4		;If handler ID is not 0x02/0x03, ignore this
	andlw	B'11111110'	; command, we don't understand any other
	btfss	STATUS,Z	; handlers
	retlw	low AUFsaIgnore	; "
	movf	AP_BUF,W	;If handler ID is 0x02/0x03, accept it as our
	movwf	AK_R3L		; new handler ID, because we as an extended
	retlw	low AUFsaIgnore	; keyboard understand those
AKFL3L0	btfss	AK_R3H,K3H_COL	;If a collision has not been detected, skip
	bra	AKFL3L2		; ahead to change the address; if one has been
	bcf	AK_R3H,K3H_COL	; detected, clear it and ignore this command
	retlw	low AUFsaIgnore	; "
AKFL3L1	bcf	FLAGS,AU_SEKB	;Copy the state of the SRQ enable bit to the SRQ
	bcf	AK_R3H,K3H_SRE	; enable flag and to our copy of register 3
	btfsc	AU_TEMP,K3H_SRE	; "
	bsf	FLAGS,AU_SEKB	; "
	btfsc	AU_TEMP,K3H_SRE	; "
	bsf	AK_R3H,K3H_SRE	; "
AKFL3L2	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	AU_TEMP,F	; byte as our new address and we're done
	movf	AK_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	AU_TEMP,W	; "
	movwf	AK_R3H		; "
	retlw	low AUFsaIgnore	; "

AKFsaTalk
	movf	AP_BUF,W	;Branch to the appropriate handler for the
	andlw	B'00000011'	; register (0, 2, or 3) being ordered to talk;
	brw			; if it's 1, we have no register 1, so ignore
	bra	AKFsaTalk0H	; "
	retlw	low AUFsaIgnore	; "
	bra	AKFsaTalk2H	; "
	;fall through

AKFsaTalk3H
	movf	AK_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movf	TMR1H,W		;Get a pseudorandom four-bit number and put it
	xorwf	TMR1L,W		; into the low nibble of the buffer; this way
	andlw	B'00001111'	; we replace address (which host already knows)
	iorwf	AP_BUF,F	; with a random, which helps detect collisions
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk3L	;Deal with what happened in the next state

AKFsaTalk3L
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	bra	AKFT0L0		; "
	movf	AK_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk23E;Deal with what happened in the next state

AKFsaTalk0H
	bcf	FLAGS,AU_SRKB	;Clear the SRQ flag, will re-set it if need be
	movf	FSR1L,W		;If the pop pointer is equal to the push
	xorwf	FSR0L,W		; pointer, we have no data to return, so ignore
	btfsc	STATUS,Z	; the command
	retlw	low AUFsaIgnore	; "
	moviw	FSR0++		;Get first byte off queue, advance and wrap the
	bcf	FSR0L,5		; pop pointer
	btfss	AK_R3L,0	;If we're handler 2, change the keycodes for
	call	AKSquashMods	; right shift/ctrl/option to the left ones
	movwf	AP_BUF		;Load keycode into the buffer
	call	AKUpdateR2	;Update register 2 based on keycode
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk0L	;Deal with what happened in the next state

AKFsaTalk0L
	bsf	FLAGS,AU_SRKB	;Set SRQ flag - if we're here, we have data
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	AKFT0L1		; bit of register 3, clear the transmit events
AKFT0L0	bsf	AK_R3H,K3H_COL	; flag, and we're done until we get another
	bcf	FLAGS,AU_TXON ; command
	retlw	low AUFsaIgnore	; "
AKFT0L1	movf	AP_BUF,W	;If the byte on top of the queue is an 0x7F or
	andlw	B'01111111'	; an 0xFF, i.e. the power key, we have to send
	xorlw	0x7F		; it twice, so skip ahead
	btfsc	STATUS,Z	; "
	bra	AKFT0L2		; "
	movf	FSR0L,W		;If there's only one event on the queue, skip
	xorwf	FSR1L,W		; ahead to load 0xFF as the second byte for
	btfsc	STATUS,Z	; transmission
	bra	AKFT0L3		; "
	moviw	FSR0++		;Get the second byte off the queue, advance and
	bcf	FSR0L,5		; wrap the pointer
	btfss	AK_R3L,0	;If we're handler 2, change the keycodes for
	call	AKSquashMods	; right shift/ctrl/option to the left ones
	movwf	AP_BUF		;Load keycode into the buffer
	call	AKUpdateR2	;Update register 2 based on keycode
AKFT0L2	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk0E	;Deal with what happened in the next state
AKFT0L3	movlw	0xFF		;Load placeholder 0xFF into the buffer
	movwf	AP_BUF		; "
	bra	AKFT0L2		;Rejoin above

AKFsaTalk0E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	bra	AKFT0L0		; "
	movf	FSR0L,W		;If the pop pointer has caught up with the push
	xorwf	FSR1L,W		; pointer, we no longer need service so clear
	btfsc	STATUS,Z	; our SRQ flag
	bcf	FLAGS,AU_SRKB	; "
	retlw	low AUFsaIgnore	;And we're done

AKFsaTalk2H
	movf	AK_R2H,W	;Load the high byte of register 2 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low AKFsaTalk2L	;Deal with what happened in the next state

AKFsaTalk2L
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	bra	AKFT0L0		; "
	movf	AK_R2L,W	;Load the low byte of register 2 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low AKFsaTalk23E;Deal with what happened in the next state

AKFsaTalk23E
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	bra	AKFT0L0		; "
	retlw	low AUFsaIgnore	;Either way, we're done

ARFsaCommand
	btfsc	AP_BUF,2	;Talk is the only command that sets bit 2
	bra	ARFsaTalk	; "
	btfsc	AP_BUF,3	;Only talk and listen set bit 3, so if it's not
	bra	ARFsaListen	; talk, it's listen
	retlw	low AUFsaIgnore	;Ignore flush

ARFsaListen
	movf	AP_BUF,W	;Go to the appropriate state for the register
	andlw	B'00000011'	; being told to listen and wait for a data byte
	brw			; "
	retlw	low ARFsaLstn0H	; "
	retlw	low ARFsaLstn13	; "
	retlw	low ARFsaLstn23	; "
	retlw	low ARFsaLstn3H	; "

ARFsaLstn0H
	movf	AP_BUF,W	;Store the received byte as the relay's register
	movwf	AR_R0H		; 0 high byte, ignore everything else
	retlw	low AUFsaIgnore	; "

ARFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AU_TEMP		; what the low byte (handler ID) is, so store
	retlw	low ARFsaLstn3L	; it in temporary space

ARFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	ARFL3L0		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfss	STATUS,Z	; device's address if no collision
	retlw	low AUFsaIgnore	;Ignore other handlers, handler cannot change
	btfss	AR_FLAG,AR_COL	;If a collision has not been detected, skip
	bra	ARFL3L0		; ahead to change the address; if one has been
	bcf	AR_FLAG,AR_COL	; detected, clear it and ignore this command
	retlw	low AUFsaIgnore	; "
ARFL3L0	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	AU_TEMP,F	; byte as our new address and we're done
	movf	AR_FLAG,W	; "
	andlw	B'11110000'	; "
	iorwf	AU_TEMP,W	; "
	movwf	AR_FLAG		; "
	retlw	low AUFsaIgnore	; "

ARFsaLstn13
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TEMP		; "
	retlw	low ARFsaLstn12	;Transition to expect the next

ARFsaLstn12
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TMP2		; "
	retlw	low ARFsaLstn11	;Transition to expect the next

ARFsaLstn11
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TMP3		; "
	retlw	low ARFsaLstn10	;Transition to expect the next

ARFsaLstn10
	movf	AP_BUF,W	;Set the value of register 1 all at once and
	movwf	AR_R1_0		; we're done
	movf	AU_TMP3,W	; "
	movwf	AR_R1_1		; "
	movf	AU_TMP2,W	; "
	movwf	AR_R1_2		; "
	movf	AU_TEMP,W	; "
	movwf	AR_R1_3		; "
	retlw	low AUFsaIgnore	; "

ARFsaLstn23
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TEMP		; "
	retlw	low ARFsaLstn22	;Transition to expect the next

ARFsaLstn22
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TMP2		; "
	retlw	low ARFsaLstn21	;Transition to expect the next

ARFsaLstn21
	movf	AP_BUF,W	;Store the byte for when we have all four
	movwf	AU_TMP3		; "
	retlw	low ARFsaLstn20	;Transition to expect the next

ARFsaLstn20
	movf	AP_BUF,W	;Set the value of register 2 all at once and
	movwf	AR_R2_0		; we're done
	movf	AU_TMP3,W	; "
	movwf	AR_R2_1		; "
	movf	AU_TMP2,W	; "
	movwf	AR_R2_2		; "
	movf	AU_TEMP,W	; "
	movwf	AR_R2_3		; "
	retlw	low AUFsaIgnore	; "

ARFsaTalk
	movf	AP_BUF,W	;Branch to the appropriate handler for the
	andlw	B'00000011'	; register being ordered to talk
	brw			; "
	bra	ARFsaTalk0H	; "
	bra	ARFsaTalk13	; "
	goto	GARFsaTalk23	; "
	;fall through

ARFsaTalk3H
	movf	TMR1H,W		;Get a pseudorandom four-bit number and put it
	xorwf	TMR1L,W		; into the low nibble of the buffer; this way
	andlw	B'00001111'	; we replace address (which host already knows)
	iorlw	B'01000000'	; with a random, which helps detect collisions;
	movwf	AP_BUF		; set exceptional event flag, SRQ always clear
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low ARFsaTalk3L	;Deal with what happened in the next state

ARFsaTalk0H
	btfss	AR_R0H,AR_TLK0	;If Talk 0 is not enabled, ignore the command
	retlw	low AUFsaIgnore	; "
	movf	AR_R0H,W	;Reply with value of stored register 0 high byte
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low ARFsaTalk0L	;Deal with what happened in the next state

ARFsaTalk0L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ARFT0L1		; bit of register 3, clear the transmit events
ARFT0L0	bsf	AR_FLAG,AR_COL	; flag, and we're done until we get another
	bcf	FLAGS,AU_TXON ; command
	retlw	low AUFsaIgnore	; "
ARFT0L1	movf	AR_FLAG,W	;Copy cause bits from AR_FLAG
	andlw	B'01100000'	; "
	btfsc	PORTA,RLY_PIN	;Set high bit if relay is closed
	iorlw	B'10000000'	; "
	iorlw	AR_VERS		;Low bits represent firmware version
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low ARFsaTalkE	;Deal with what happened in the next state

ARFsaTalkE
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	bra	ARFT0L0		; "
	retlw	low AUFsaIgnore	;And we're done

ARFsaTalkX2
	goto	GARFsaTalkX2	;Rejoin state moved due to lack of space

ARFsaTalkX1
	goto	GARFsaTalkX1	;Rejoin state moved due to lack of space

ARFsaTalkX0
	goto	GARFsaTalkX0	;Rejoin state moved due to lack of space

ARFsaTalk3L
	goto	GARFsaTalk3L	;Rejoin state moved due to lack of space

ARFsaTalk13
	movf	AR_R1_3,W	;Grab the full data of register 1 into temp vars
	movwf	AP_BUF		; and send the first byte of it
	movf	AR_R1_2,W	; "
	movwf	AU_TEMP		; "
	movf	AR_R1_1,W	; "
	movwf	AU_TMP2		; "
	movf	AR_R1_0,W	; "
	movwf	AU_TMP3		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low ARFsaTalkX2	;Deal with what happened in the next state

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	low AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,ADB_PIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,ADB_PIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-65		;Shorten the timeout period to 260 us, which is
	movwf	TMR0		; slightly longer than Tlt is expected to be
	bsf	INTCON,TMR0IE	;Timer interrupts whether we transmit or not
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	retlw	low AdbFsaTxBitD;Bring us to the transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,ADB_PIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,ADB_PIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,ADB_PIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,ADB_PIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,ADB_PIN	; set doesn't immediately get reset
	bcf	IOCAF,ADB_PIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	btfsc	BSR,0		;If we're here because of a timer interrupt, the
	bra	AFTltE0		; transmission never started, so skip ahead
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data
AFTltE0	bsf	AP_FLAG,AP_DONE	;Set the done flag because the data payload is
	retlw	low AdbFsaIdle	; effectively done and return to idle

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBU0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBU0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "

Ps2FsaStart
	btfsc	STATUS,C	;If for some reason the start bit is a 1, ignore
	retlw	low Ps2FsaStart	; it
	movlw	0x80		;Otherwise, initialize shift register with a
	movwf	PP_SR		; sentinel bit, ready to take data LSb first
	bsf	PP_FLAG,PP_PAR	;Start off expecting a 1 (odd) parity bit
	retlw	low Ps2FsaBit	;Transition to accept first bit

Ps2FsaIgnore
	retlw	low Ps2FsaIgnore;Utility state to ignore bus until timeout

Ps2FsaBit
	movlw	1 << PP_PAR	;If the received bit (in C) was a 1, complement
	btfsc	STATUS,C	; the expected odd parity bit
	xorwf	PP_FLAG,F	; "
	rrf	PP_SR,F		;Rotate received bit (in C) into SR from left
	btfss	STATUS,C	;If a 1 didn't fall out of the SR, we haven't
	retlw	low Ps2FsaBit	; yet completed a byte
	btfsc	PP_FLAG,PP_PAR	;If we completed a byte, transition to expect
	retlw	low Ps2FsaParP	; the appropriate parity bit
	retlw	low Ps2FsaParN	; "

Ps2FsaParP
	btfsc	STATUS,C	;If we got the expected parity bit, transition
	retlw	low Ps2FsaStop	; to expect the stop bit, else ignore this byte
	retlw	low Ps2FsaIgnore; "

Ps2FsaParN
	btfss	STATUS,C	;If we got the expected parity bit, transition
	retlw	low Ps2FsaStop	; to expect the stop bit, else ignore this byte
	retlw	low Ps2FsaIgnore; "

Ps2FsaStop
	btfss	STATUS,C	;Stop bit should be high; if it's not, something
	retlw	low Ps2FsaIgnore; is wrong, wait for bus timeout
	movf	PP_SR,W		;Get the received byte
	xorlw	0x83		;If the received byte was 0x83 (F7), act like it
	btfsc	STATUS,Z	; was 0x7F so we can halve the size of the LUT
	movlw	0x7F ^ 0x83	; "
	xorlw	0xF0 ^ 0x83	;If the received byte was 0xF0, set that flag
	btfsc	STATUS,Z	; "
	bsf	PP_FLAG,PK_F0	; "
	xorlw	0xE0 ^ 0xF0	;If the received byte was 0xE0, set that flag
	btfsc	STATUS,Z	; "
	bsf	PP_FLAG,PK_E0	; "
	xorlw	0xE1 ^ 0xE0	;If the received byte was 0xE1, set that flag
	btfsc	STATUS,Z	; "
	bsf	PP_FLAG,PK_E1	; "
	xorlw	0x00 ^ 0xE1	;If the received byte's MSb was set (except if
	btfsc	WREG,7		; it was 0x83 for F7), do nothing else and
	retlw	low Ps2FsaStart	; wait for the next byte
	btfss	PP_FLAG,PK_IGNX	;If we were flagged to ignore the next event,
	bra	PFStop0		; reset flag instead of continuing
	bcf	PP_FLAG,PK_IGNX	; "
	retlw	low Ps2FsaStart	; "
PFStop0	btfss	PP_FLAG,PK_E1	;If the 0xE1 flag was set, change lookup index
	bra	PFStop1		; to 0x7F for Pause key, raise the 0xE0 flag,
	movlw	0x7F		; and raise the flag to ignore the Num Lock
	bsf	PP_FLAG,PK_IGNX	; press that will inevitably follow
	bsf	PP_FLAG,PK_E0	; "
	bcf	PP_FLAG,PK_E1	; "
PFStop1	movlb	3		;Save this byte as the lookup index
	movwf	PMADRL		; "
	bsf	PMADRL,7	; "
	bcf	INTCON,GIE	;Look up the corresponding ADB keyboard key code
	bsf	PMCON1,RD	; in the LUT
	nop			; "
	nop			; "
	bsf	INTCON,GIE	; "
	lslf	PMDATL,W	;Move MSb of looked-up word into carry
	movf	PMDATL,W	;Put low 7 bits of word into W
	andlw	B'01111111'	; "
	btfsc	PP_FLAG,PK_E0	;If 0xE0 (or 0xE1) was received, put high 7 bits
	rlf	PMDATH,W	; of word into W
	xorlw	AK_NONE		;If the looked-up code is AK_NONE, that means
	btfsc	STATUS,Z	; there is no corresponding key on the ADB
	retlw	low Ps2FsaStart	; keyboard
	xorlw	AK_ACC ^ AK_NONE;If the key is caps lock, skip ahead to treat it
	btfsc	STATUS,Z	; specially (ADB keyboards assume it's locking,
	bra	PFStop3		; PS/2 keyboards don't)
	xorlw	0 ^ AK_ACC	; "
	btfsc	PP_FLAG,PK_F0	;Move the state of the 0xF0 flag into the MSb
	iorlw	B'10000000'	; "
	bcf	PP_FLAG,PK_F0	; "
PFStop2	movlb	0		;If this is the same as the last code that was
	xorwf	AK_LAST,W	; pushed onto the keyboard queue, suppress it
	btfsc	STATUS,Z	; "
	retlw	low Ps2FsaStart	; "
	xorwf	AK_LAST,W	;Otherwise, update the last code
	movwf	AK_LAST		; "
	movwi	FSR1++		;Push it onto the keyboard queue, advance and
	bcf	FSR1L,5		; wrap the pointer
	bsf	FLAGS,AU_SRKB	;Set the flag so the keyboard requests service
	call	RPK0		;Make sure the relay is closed
	retlw	low Ps2FsaStart	;Transition to receive the next byte
PFStop3	btfsc	PP_FLAG,PK_F0	;If this is a key release, skip ahead
	bra	PFStop5		; "
	movlw	AK_ACC		;Ready the key press code to be pushed
PFStop4	btfsc	FLAGS,AK_CAPS	;If the flag was already on (for a push) or was
	retlw	low Ps2FsaStart	; just turned on (for a release), suppress the
	bra	PFStop2		; key event, otherwise pass it through
PFStop5	bcf	PP_FLAG,PK_F0	;This is a key release, clear the flag and
	movlw	1 << AK_CAPS	; invert the internal caps lock flag (have to do
	xorwf	FLAGS,F		; this on release, presses may be repeated)
	movlw	AK_ACC | 0x80	;Ready the key release code to be pushed and
	bra	PFStop4		; rejoin above

GARFsaTalkX2
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	goto	ARFT0L0		; "
	movf	AU_TEMP,W	;Send the second byte of register 1 or 2
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	retlw	low ARFsaTalkX1	;Deal with what happened in the next state

GARFsaTalkX1
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	goto	ARFT0L0		; "
	movf	AU_TMP2,W	;Send the third byte of register 1 or 2
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	retlw	low ARFsaTalkX0	;Deal with what happened in the next state

GARFsaTalkX0
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	goto	ARFT0L0		; "
	movf	AU_TMP3,W	;Send the fourth byte of register 1 or 2
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low ARFsaTalkE	;Deal with what happened in the next state

GARFsaTalk23
	movf	AR_R2_3,W	;Grab the full data of register 2 into temp vars
	movwf	AP_BUF		; and send the first byte of it
	movf	AR_R2_2,W	; "
	movwf	AU_TEMP		; "
	movf	AR_R2_1,W	; "
	movwf	AU_TMP2		; "
	movf	AR_R2_0,W	; "
	movwf	AU_TMP3		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,AU_TXON	; and we are interested in transmission events
	retlw	low ARFsaTalkX2	;Deal with what happened in the next state

GARFsaTalk3L
	btfsc	AP_FLAG,AP_COL	;If there was a collision, handle it
	goto	ARFT0L0		; "
	movlw	AR_HDLR		;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,AU_TXON	; and that we no longer want transmit events
	retlw	low ARFsaTalkE	;Deal with what happened in the next state


;;; Subprograms ;;;

AKSquashMods
	addlw	-123		;Change 0x7B (release right shift) to 0x38
	btfsc	STATUS,Z	; (release shift)
	retlw	0x38		; "
	addlw	-1		;Change 0x7C (release right option) to 0x3A
	btfsc	STATUS,Z	; (release option)
	retlw	0x3A		; "
	addlw	-1		;Change 0x7D (release right control) to 0x36
	btfsc	STATUS,Z	; (release control)
	retlw	0x36		; "
	addlw	-126		;Change 0xFB (press right shift) to 0xB8 (press
	btfsc	STATUS,Z	; shift)
	retlw	0xB8		; "
	addlw	-1		;Change 0xFC (press right option) to 0xBA
	btfsc	STATUS,Z	; (press option)
	retlw	0xBA		; "
	addlw	-1		;Change 0xFD (press right control) to 0xB6
	btfsc	STATUS,Z	; (press control)
	retlw	0xB6		; "
	addlw	-3		;Otherwise, leave as is
	return			; "

AKUpdateR2
	;TODO "exceptional event" for ADB keyboards means reset was pressed
	btfss	WREG,7		;If MSB of the key pressed is set (because the
	bra	AKUpR20		; key is being released), snuff it to zero and
	andlw	B'01111111'	; complement the register 2 registers before
	comf	AK_R2H,F	; and after setting them, this way we don't
	comf	AK_R2L,F	; have to copypaste code
	comf	AK_MOD,F	; "
	call	AKUpR22		; "
	comf	AK_R2H,F	; "
	comf	AK_R2L,F	; "
	comf	AK_MOD,F	; "
	bra	AKUpR21		; "
AKUpR20	call	AKUpR22		; "
AKUpR21	bsf	AK_R2H,K2H_CTL	;If either control key is down, reflect that in
	btfsc	AK_MOD,KMD_LCT	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_RCT	; "
	bcf	AK_R2H,K2H_CTL	; "
	bsf	AK_R2H,K2H_SHF	;If either shift key is down, reflect that in
	btfsc	AK_MOD,KMD_LSH	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_RSH	; "
	bcf	AK_R2H,K2H_SHF	; "
	bsf	AK_R2H,K2H_OPT	;If either option key is down, reflect that in
	btfsc	AK_MOD,KMD_LOP	; the appropriate bits in keyboard register 2
	btfss	AK_MOD,KMD_ROP	; "
	bcf	AK_R2H,K2H_OPT	; "
	return
AKUpR22	addlw	-51		;0x33 (delete/backspace)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_DEL
	addlw	-3		;0x36 (left control)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LCT
	addlw	-1		;0x37 (command)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_CMD
	addlw	-1		;0x38 (left shift)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LSH
	addlw	-1		;0x39 (caps lock)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_CAP
	addlw	-1		;0x3A (left option)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_LOP
	addlw	-13		;0x47 (clear/num lock)
	btfsc	STATUS,Z
	bcf	AK_R2L,K2L_CLR
	addlw	-36		;0x6B (F14/scroll lock)
	btfsc	STATUS,Z
	bcf	AK_R2L,K2L_SLK
	addlw	-16		;0x7B (right shift)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_RSH
	addlw	-1		;0x7C (right option)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_ROP
	addlw	-1		;0x7D (right control)
	btfsc	STATUS,Z
	bcf	AK_MOD,KMD_RCT
	addlw	-2		;0x7F (reset)
	btfsc	STATUS,Z
	bcf	AK_R2H,K2H_RST
	return

RelayPowerKey
	bcf	FLAGS,PK_IOCF	;Clear the interrupt
RPK0	btfsc	PORTA,RLY_PIN	;If the relay is already closed, make no change
	return			; "
	bcf	AR_FLAG,AR_R1OF	;Flag that a press of the power key closed the
	bsf	AR_FLAG,AR_PKEY	; relay
	movlb	2		;Close the relay
	bsf	LATA,RLY_PIN	; "
	movlb	0		; "
	return


;;; Lookup Tables ;;;

	org	0x380

;Key lookup table for translating PS/2 scan codes to ADB keyboard codes
;Index is PS/2 scan code; bits 6-0 are corresponding ADB key code, bits 13-7 are
; corresponding ADB key code when PS/2 key code was preceded by 0xE0
;Index 0x7F is special - bits 6-0 are corresponding ADB key code to PS/2 scan 
; code 0x83 (F7), bits 13-7 are corresponding ADB key code when PS/2 scan code
; is preceded by 0xE1 (used for Pause key)
;Value of 0x7E in bits 13-7 or 6-0 indicates that there is no corresponding ADB
; key code
KeyLut
	dw	0x3F7E,0x3F65,0x3F7E,0x3F60,0x3F63,0x3F7A,0x3F78,0x3F6F
	dw	0x3F7E,0x3F6D,0x3F64,0x3F61,0x3F76,0x3F30,0x3F32,0x3F7E
	dw	0x3F7E,0x3E3A,0x3F38,0x3F7E,0x3EB6,0x3F0C,0x3F12,0x3F7E
	dw	0x3F7E,0x3F7E,0x3F06,0x3F01,0x3F00,0x3F0D,0x3F13,0x1BFE
	dw	0x3F7E,0x3F08,0x3F07,0x3F02,0x3F0E,0x3F15,0x3F14,0x1BFE
	dw	0x3F7E,0x3F31,0x3F09,0x3F03,0x3F11,0x3F0F,0x3F17,0x3FFE
	dw	0x3F7E,0x3F2D,0x3F0B,0x3F04,0x3F05,0x3F10,0x3F16,0x3FFE
	dw	0x3F7E,0x3F7E,0x3F2E,0x3F26,0x3F20,0x3F1A,0x3F1C,0x3FFE
	dw	0x3F7E,0x3F2B,0x3F28,0x3F22,0x3F1F,0x3F1D,0x3F19,0x3F7E
	dw	0x3F7E,0x3F2F,0x25AC,0x3F25,0x3F29,0x3F23,0x3F1B,0x3F7E
	dw	0x3F7E,0x3F7E,0x3F27,0x3F7E,0x3F21,0x3F18,0x3F7E,0x3F7E
	dw	0x3F39,0x3F7B,0x2624,0x3F1E,0x3F7E,0x3F2A,0x3FFE,0x3F7E
	dw	0x3F7E,0x3F7E,0x3F7E,0x3F7E,0x3F7E,0x3F7E,0x3F33,0x3F7E
	dw	0x3F7E,0x3BD3,0x3F7E,0x1DD6,0x39D9,0x3F7E,0x3F7E,0x3F7E
	dw	0x3952,0x3AC1,0x1ED4,0x3F57,0x1E58,0x1F5B,0x3F35,0x3F47
	dw	0x3F67,0x3F45,0x3CD5,0x3F4E,0x34C3,0x3A5C,0x3F6B,0x38E2


;;; End of Program ;;;

	end
