; Stop Go C/ASM Mix Example
; Jason Losh

;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz

; Hardware configuration:
; Red LED:
;   PF1 drives an NPN transistor that powers the red LED
; Green LED:
;   PF3 drives an NPN transistor that powers the green LED
; Pushbutton:
;   SW1 pulls pin PF4 low (internal pull-up is used)

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------
   .def pushfake
   .def pushR4R11
   .def popR4R11
   .def setPSP
   .def setMSP
   .def getPSP
   .def getMSP
   .def getR0



;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.text
GPIO_PORTF_DATA_R       .field   0x400253FC

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

; Blocking function that returns only when SW1 is pressed
;-----------------------------------------------------------------------------
pushfake:
		;MOV	R3, R0
		;MRS	R0, PSP
		;MRS	R1, XPSR
		;ADD R0, R0, #4
		MRS	R2, PSP
		SUB	R2,	R2, #4
		MOV	R3,	R0
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	R1
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#13
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#12
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#4
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#3
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#2
		STR R3,	[R2]
		SUB	R2,	R2, #4
		MOV	R3,	#1
		STR R3,	[R2]
		MSR PSP, R2
    	;MOV	 R3, #0x88D
		BX	 LR





pushR4R11:
		MRS	R0, PSP
		SUB R0, R0, #4
    	STR	R4, [R0]
    	SUB R0, R0, #4
    	STR	R5, [R0]
    	SUB R0, R0, #4
    	STR	R6, [R0]
    	SUB R0, R0, #4
    	STR	R7, [R0]
    	SUB R0, R0, #4
    	STR	R8, [R0]
    	SUB R0, R0, #4
    	STR	R9, [R0]
    	SUB R0, R0, #4
    	STR	R10, [R0]
    	SUB R0, R0, #4
    	STR	R11, [R0]
    	MSR PSP, R0
		BX	LR
popR4R11:
    	MRS	R0, PSP
		;MRS	R1, XPSR
		LDR	R11, [R0]
    	ADD R0, R0, #4
    	LDR	R10, [R0]
    	ADD R0, R0, #4
    	LDR	R9, [R0]
    	ADD R0, R0, #4
    	LDR	R8, [R0]
    	ADD R0, R0, #4
    	LDR	R7, [R0]
    	ADD R0, R0, #4
    	LDR	R6, [R0]
    	ADD R0, R0, #4
    	LDR	R5, [R0]
    	ADD R0, R0, #4
    	LDR	R4, [R0]
    	ADD R0, R0, #4
    	;MOV	 R3, #0x88D
    	MSR PSP, R0
		BX	 LR
setPSP:
		MSR	PSP, R0
		BX	LR
getMSP:
		MRS R0, MSP
		BX LR
getPSP:
		MRS	R0, PSP
		BX	LR
setMSP:
		MSR	MSP, r0
		BX LR
getR0:
		BX LR
.endm
