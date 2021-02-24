;*******************************************************************************
;*******************************************************************************
;**                                                                           **
;**             ��������� ��� ����� �� ������� CGM12864GSLI.                  **
;**                                                                           **
;**               ��� �� ATmega8L �������� ������� 4.00 ��                    **
;**                                                                           **
;*******************************************************************************
;*******************************************************************************
/*  Fuse Bits:
 CKSEL0=1
 CKSEL1=0
 CKSEL2=0
 CKSEL3=0
 SUT0=0
 SUT1=1
 BODEN=1
 BODLEVEL=1
 BOOTRST=1
 BOOTSZ0=0
 BOOTSZ1=0
 EESAVE=1
 CKOPT=1
 WDTON=1
 RSTDISBL=1
 ��� ���������=0 ������ �������, ��� ��������=1 ��������� �����.
*/

.include "m8def.inc"
 
    
	; ���������� ����������

.def	mulL=r0
.def	mulH=r1
.def	adwL=r2
.def	adwH=r3
.def	count=r4
.def	char_ind=r5
.def	count_row=r6
.def	temp_count=r7
.def	d1=r8
.def	GLCDX=r9
.def	GLCDY=r10
.def	temp3=r11
.def	temp4=r12
;.def	=r13
;.def	=r14
;.def	=r15

.def	temp0=r16
.def	temp=r17
.def	temp1=r18
.def	temp2=r19
.def	cicle_count=r20
.def	DigPoz=r21
.def i2cstat=r22
.def i2cdelay=r23
.def i2cdata=r24
.def i2cadr=r25
;.def	=r26
;.def	=r27
;.def	=r28
;.def	=r29
;.def	=r30
;.def	=r31




	; ���������� ���������� ������� ����������� ������� � ��

.equ	GLCD_DATA_PORT= PORTD				; ���� ����� ������
.equ	GLCD_DATA_PIN = PIND
.equ	GLCD_DATA_DDR = DDRD

.equ	GLCD_CMD_PORT= PORTC				; ���� ����� ����������
.equ	GLCD_CMD_PIN = PINC
.equ	GLCD_CMD_DDR = DDRC

.equ	GLCD_RS = 2							; ������ �������� ������/������� (1-������, 0-�������)
.equ	GLCD_WR = 3							; ������ �������� ������/������ (1-������, 0-������)
.equ	GLCD_E  = 4							; ������ �������������/�������������
.equ	GLCD_CS = 5							; ������ ����� ���� (0-��������, 1-����������)

.equ	GLCD_DEL= 5							; ����������� �������� ��� �������

.equ	BIAS_SET                 =0xa2		; ST7565P
.equ	ADC_SELECT               =0xa0
.equ	COMMON_OUTPUTMODE_SELECT =0xc8		; normal direction
.equ	RES_RATIO                =0x25		; Ratio=5.5
.equ	ELECVOLUME_SET           =0x81
.equ	VOLREGISTOR_SET          =0x2f		; 0 - 3F
.equ	POWERCONTROL_SET         =0x2f		; Boost on,Volt_reg on,Volt_follow on
.equ	GLCD_NOP                 =0xe3
.equ	PAGE0_SET                =0xb0
.equ	COLUM_UP_ADD             =0x10
.equ	COLUM_LO_ADD             =0X04
.equ	STARTLINE0_SET           =0x40
.equ	DISPLAY_ON               =0xaf
.equ	DISPLAY_OFF              =0xae
.equ	DISPALL_ON               =0xa5
.equ	DISPALL_OFF              =0xa4
.equ	BOOST_RATIO              =0xf8
.equ	READ_MODWRT              =0xe0
.equ	END                      =0xee
.equ	RESET_DISPLAY            =0xE2



;------------------------------------------------------------------------------
;   ��� � ������ ��� I2C

.equ PORTI2C=PORTC							; ���� I2C
.equ SCLP=0	  								; SCL Pin master
.equ SDAP=1 								; SDA Pin master

.equ b_dir=0                                ; transfer direction bit in i2cadr

.equ i2crd=1
.equ i2cwr=0

	; ��� ������� ������. � ��� ���������� ����������� ������

.dseg

Digit:		.byte 6
BCD:		.byte 5
TIME:		.byte 10
HOUR_H:		.byte 1
HOUR_L:		.byte 1
MINUTES_H:	.byte 1
MINUTES_L:	.byte 1
SECOND_H:	.byte 1
SECOND_L:	.byte 1
DAY_L:		.byte 1
DAY_H:		.byte 1
MONTH_L:	.byte 1
MONTH_H:	.byte 1
WEEKDAYS:	.byte 1

.cseg

; ***** INTERRUPT VECTORS ************************************************
.org	$000
		rjmp RESET                             ; ��������� ������
.org	$0001
		rjmp aINT0                              ; 0x0001 - ������� ���������� 0
.org	$0002
		rjmp aINT1                              ; 0x0002 - ������� ���������� 1
.org	$0003
		rjmp aOC2                               ; 0x0003 - ���������� �������/�������� �2
.org	$0004
		rjmp aOVF2                              ; 0x0004 - ������������ �������/�������� �2
.org	$0005
		rjmp aICP1                              ; 0x0005 - ������ �������/�������� �1
.org	$0006
		rjmp aOC1A                              ; 0x0006 - ���������� � �������/�������� �1
.org	$0007
		rjmp aOC1B                              ; 0x0007 - ���������� � �������/�������� �1
.org	$0008
		rjmp aOVF1                              ; 0x0008 - ������������ �������/�������� �1
.org	$0009
		rjmp aOVF0                              ; 0x0009 - ������������ �������/�������� �0
.org	$000a
		rjmp aSPI                               ; 0x000a - �������� �� SPI ���������
.org	$000b
		rjmp aURXC                              ; 0x000b - USART, ����� ��������
.org	$000c
		rjmp aUDRE                              ; 0x000c - ������� ������ USART ����
.org	$000d
		rjmp aUTXC                              ; 0x000d - USART, �������� ���������
.org	$000e
		rjmp aADCC                              ; 0x000e - �������������� ADC ���������
.org	$000f
		rjmp aERDY                              ; 0x000f - EEPROM �����
.org	$0010
		rjmp aACI                               ; 0x0010 - Analog Comparator
.org	$0011
		rjmp aTWI                               ; 0x0011 - ���������� �� 2-wire Serial Interface
.org	$0012
		rjmp aSPMR                              ; 0x0012 - ���������� SPM





aINT0:                              ; 0x0001 - ������� ���������� 0
aINT1:                              ; 0x0002 - ������� ���������� 1
aOC2:                               ; 0x0003 - ���������� �������/�������� �2
aOVF2:                              ; 0x0004 - ������������ �������/�������� �2
aICP1:                              ; 0x0005 - ������ �������/�������� �1
aOC1A:                              ; 0x0006 - ���������� � �������/�������� �1
aOC1B:                              ; 0x0007 - ���������� � �������/�������� �1
aOVF1:                              ; 0x0008 - ������������ �������/�������� �1
aOVF0:                              ; 0x0009 - ������������ �������/�������� �0
aSPI:                               ; 0x000a - �������� �� SPI ���������
aURXC:                              ; 0x000b - USART, ����� ��������
aUDRE:                              ; 0x000c - ������� ������ USART ����
aUTXC:                              ; 0x000d - USART, �������� ���������
aADCC:                              ; 0x000e - �������������� ADC ���������
aERDY:                              ; 0x000f - EEPROM �����
aACI:                               ; 0x0010 - Analog Comparator
aTWI:                               ; 0x0011 - ���������� �� 2-wire Serial Interface
aSPMR:                              ; 0x0012 - ���������� SPM



RESET:

	; ������������� ����� 
 ldi 	temp, LOW(RAMEND)
 out	spl, Temp
 ldi 	temp, HIGH(RAMEND)
 out	sph, Temp



	; ������������� ������
		
 ldi	temp,0b00000010                       ; ������������� ����� B
 out	DDRB,temp

 ldi	temp,0b00111100                       ; ������������� ����� C
 out	DDRC,temp

 ldi	temp,0b11111111                       ; ������������� ����� D
 out	DDRD,temp
 
 ldi	temp,0b11000101                       ; ������ �������� �� ����� ����� B
 out	PORTB,temp      
        
 ldi	temp,0b00000000                       ; ������ �������� �� ����� ����� C
 out	PORTC,temp      

 ldi	temp,0b00000000                       ; ������ �������� �� ����� ����� D
 out	PORTD,temp      


/*
;------------------------------------------------------------------------------
     ; ��������� �������/�������� T0 � ������ ������������
 ldi Temp,0b00000010
    ; 7 - 
    ; 6 - 
    ; 5 - 
    ; 4 - 
    ; 3 - 
    ; 2 - CS02 - 1 \
    ; 1 - CS01 - 0  _ ���������� �������� �������� (clk/256)
    ; 0 - CS00 - 0 /
 out TCCR0,Temp

 ldi Temp,0b00000001                        ; ��������� ���������� �� "������"
    ; 7 - OCIE2 ���� �� ���������� ���������� �� "����������"
    ; 6 - TOIE2 ���� �� ���������� ���������� �� ������������ �������/�������� �1
    ; +5 - TICIE1 ���� �� ���������� ���������� �� "������" ������� �������� �1
    ; 4 - OCIE1A ���� �� ���������� ���������� �� "���������� �"
    ; 3 - OCIE1B ���� �� ���������� ���������� �� "���������� �"
    ; 2 - TOIE1 ���� �� ���������� ���������� �� ������������ �������/�������� �1
    ; 1 - 
    ; +0 - TOIE0 ���� �� ���������� ���������� �� ������������ �������/�������� �0
 out TIMSK,Temp
*/


;******************************************************************************
;*                                                                            *
;*                                                                            *
;*         ���������� ��������� � ������������� ���������� ���������          *
;*                                                                            *
;*                                                                            *
;******************************************************************************



 rcall	GLCD_INIT							; ������������� �������
 rcall	GLCD_CLR							; ������� ����������

 sbi	PORTB,1								; �������� ��������� �������
 clr	DigPoz
 clr	count_row
 clr	adwL

 rcall	d100ms

;******************************************************************************
;******************************************************************************
;**                                                                          **
;**                                                                          **
;**                     �������� ���� ������ ���������                       **
;**                                                                          **
;**                                                                          **
;******************************************************************************
;******************************************************************************

MAIN_LOOP:
 sbis	PINB,2								; ��������� ������� ������ "1" 
 rjmp	DAYS_SET							; ��������� �� ��������� ��������� ����

 sbis	PINB,6								; ��������� ������� ������ "2" 
 rjmp	MONTH_SET							; ��������� �� ��������� ��������� ������

 sbis	PINB,7								; ��������� ������� ������ "3" 
 rjmp	HOUR_SET							; ��������� �� ��������� ��������� �����

 sbis	PINB,0								; ��������� ������� ������ "4" 
 rjmp	MINUTES_SET							; ��������� �� ��������� ��������� �����


; ������ ������� �� ������ 0x02 �� PCF8583
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$02							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata
;	��������� �������
; sts	TIME,temp
 andi	temp,0xF0							; �������� ������� ����� ����� (������� ������)
 swap	temp								; ������ ������� �������
 sts	SECOND_H,temp						; ��������� �������� ������� ������
 mov	temp,i2cdata
 andi	temp,0x0F							; �������� ������� ����� ����� (������� ������)
 sts	SECOND_L,temp						; ��������� �������� ������ ������
nop


; ������ ������ �� ������ 0x03 �� PCF8583
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$03							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata
;	��������� ������
; sts	TIME+1,temp
 andi	temp,0xF0							; �������� ������� ����� ����� (������� �����)
 swap	temp								; ������ ������� �������
 sts	MINUTES_H,temp						; ��������� �������� ������� �����
 mov	temp,i2cdata
 andi	temp,0x0F							; �������� ������� ����� ����� (������� �����)
 sts	MINUTES_L,temp						; ��������� �������� ������ �����
nop


; ������ ��� �� ������ 0x04 �� PCF8583
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$04							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata
;	��������� ����
; sts	TIME+2,temp
 swap	temp								; ������ ������� ������� ��������
 andi	temp,0x03							; �������� ��������� ��� ������� ��������
 sts	HOUR_H,temp							; ��������� �������� ������� �����
 mov	temp,i2cdata
 andi	temp,0x0F							; �������� ������� ������� (������� �����)
 sts	HOUR_L,temp							; ��������� �������� ������ �����
nop


; ������ ����/��� �� ������ 0x05 �� PCF8583
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$05							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata
;	��������� ����
 swap	temp
 andi	temp,0x03							; ��������� ������� ���� [0,1,2,3]
 sts	DAY_H,temp							; ��������� ������� ���
 mov	temp,i2cdata
 andi	temp,0x0F							; ��������� ������� ���� [0,1,2,3,4,5,6,7,8,9]
 sts	DAY_L,temp							; ��������� ������� ����
nop


; ������ ������/����� �� ������ 0x06 �� PCF8583
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$06							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata
 swap	temp
 andi	temp,0x01							; ��������� ������� ������ [0,1]
 sts	MONTH_H,temp
 mov	temp,i2cdata
 andi	temp,0x0F							; ��������� ������� ������ [0,1,2,3,4,5,6,7,8,9]
 sts	MONTH_L,temp                   
 mov	temp,i2cdata
 swap	temp
 andi	temp,0x0F
 lsr	temp								; ��������� ���� ������
 sts	WEEKDAYS,temp
nop




;***********************************************************************************
;	������� ����, ������ � ������� �� �������


;	������� ������� ����� �� �������
 ldi	temp,0
 mov	GLCDX,temp
 rcall	GLCD_X
 ldi	temp,1
 mov	GLCDY,temp
 rcall	GLCD_Y
 lds	temp,HOUR_H
 rcall	GLCD_PUT_CHAR_B


;	������� ������� ����� �� �������
 ldi	temp,30
 mov	GLCDX,temp
 rcall	GLCD_X
 ldi	temp,1
 mov	GLCDY,temp
 rcall	GLCD_Y
 lds	temp,HOUR_L
 rcall	GLCD_PUT_CHAR_B

 ldi	temp,55								; ��������� ���� � ������ ��������� ������
 rcall	GLCD_X
 ldi	temp,3
 rcall	GLCD_Y

 lds	temp,SECOND_H
 rcall	GLCD_PUT_CHAR						; ������� ������� ������
 lds	temp,SECOND_L
 rcall	GLCD_PUT_CHAR						; ������� ������� ������


;	������� ������� ����� �� �������
 ldi	temp,70
 mov	GLCDX,temp
 rcall	GLCD_X
 ldi	temp,1
 mov	GLCDY,temp
 rcall	GLCD_Y
 lds	temp,MINUTES_H
 rcall	GLCD_PUT_CHAR_B


;	������� ������� ����� �� �������
 ldi	temp,100
 mov	GLCDX,temp
 rcall	GLCD_X
 ldi	temp,1
 mov	GLCDY,temp
 rcall	GLCD_Y
 lds	temp,MINUTES_L
 rcall	GLCD_PUT_CHAR_B


;***********************************************************************************
;	������� �����, ����� � ���� ������ 

 ldi	temp,30
 rcall	GLCD_X
 ldi	temp,7
 rcall	GLCD_Y


 lds	temp,DAY_H							; ������� ����� ����� ���
 rcall	GLCD_PUT_CHAR
 lds	temp,DAY_L							; ������� ����� ����� ���
 rcall	GLCD_PUT_CHAR
 ldi	temp,32
 rcall	GLCD_PUT_CHAR

;	����������� �����
 ldi	temp0,10
 lds	temp1,MONTH_H						; ��������� ������� ����� ����� ������
 lds	temp,MONTH_L						; ��������� ������� ����� ����� ������
 sbrc	temp1,0								; ��������� �������� ������� ����� ����� ������
 add	temp,temp0							; ���� ������� ����� �����=1, �� �� ���������� 10 � ������� ����� 
 clr	temp_count 
 cpi	temp,1
 breq	IAR
 cpi	temp,2
 breq	FEV
 cpi	temp,3
 breq	MRT
 cpi	temp,4
 breq	APR
 cpi	temp,5
 breq	MAI
 cpi	temp,6
 breq	IUN
 cpi	temp,7
 breq	IUL
 cpi	temp,8
 breq	AVG
 cpi	temp,9
 breq	STB
 cpi	temp,10
 breq	OCT
 cpi	temp,11
 breq	NEB
 cpi	temp,12
 breq	DIC

IAR:
 rjmp	FRAZA1_V
FEV:
 rjmp	FRAZA2_V
MRT:
 rjmp	FRAZA3_V
APR:
 rjmp	FRAZA4_V
MAI:
 rjmp	FRAZA5_V
IUN:
 rjmp	FRAZA6_V
IUL:
 rjmp	FRAZA7_V
AVG:
 rjmp	FRAZA8_V
STB:
 rjmp	FRAZA9_V
OCT:
 rjmp	FRAZA10_V
NEB:
 rjmp	FRAZA11_V
DIC:
 rjmp	FRAZA12_V

FRAZA1_V:									; ������
 ldi	ZL,low(FRAZA1*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA1*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA1_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA1_V							; ���������� �������� ����� �� �������
FRAZA1_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA2_V:									; �������
 ldi	ZL,low(FRAZA2*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA2*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA2_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA2_V							; ���������� �������� ����� �� �������
FRAZA2_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA3_V:									; ����
 ldi	ZL,low(FRAZA3*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA3*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA3_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA3_V							; ���������� �������� ����� �� �������
FRAZA3_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA4_V:									; ������
 ldi	ZL,low(FRAZA4*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA4*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA4_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA4_V							; ���������� �������� ����� �� �������
FRAZA4_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA5_V:									; ���
 ldi	ZL,low(FRAZA5*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA5*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA5_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA5_V							; ���������� �������� ����� �� �������
FRAZA5_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA6_V:									; ����
 ldi	ZL,low(FRAZA6*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA6*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA6_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA6_V							; ���������� �������� ����� �� �������
FRAZA6_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA7_V:									; ����
 ldi	ZL,low(FRAZA7*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA7*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA7_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA7_V							; ���������� �������� ����� �� �������
FRAZA7_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA8_V:									; ������
 ldi	ZL,low(FRAZA8*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA8*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA8_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA8_V							; ���������� �������� ����� �� �������
FRAZA8_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA9_V:									; ��������
 ldi	ZL,low(FRAZA9*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA9*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA9_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA9_V							; ���������� �������� ����� �� �������
FRAZA9_END:									; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA10_V:									; �������
 ldi	ZL,low(FRAZA10*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA10*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA10_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA10_V							; ���������� �������� ����� �� �������
FRAZA10_END:								; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA11_V:									; ������
 ldi	ZL,low(FRAZA11*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA11*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA11_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA11_V							; ���������� �������� ����� �� �������
FRAZA11_END:								; ������� �� ����� ������ ����� �� ������� 
 rjmp	MONTH_END


FRAZA12_V:									; �������
 ldi	ZL,low(FRAZA12*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA12*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA12_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	GLCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA12_V							; ���������� �������� ����� �� �������
FRAZA12_END:								; ������� �� ����� ������ ����� �� ������� 


MONTH_END:

;	������� ���� ������
WEEKDAY:
; ldi	temp,0
; rcall	GLCD_X
; ldi	temp,7
; rcall	GLCD_Y

; lds	temp,WEEKDAYS						;

; rcall	GLCD_PUT_CHAR

 ldi	temp,116
 rcall	GLCD_X
 ldi	temp,7
 rcall	GLCD_Y

 lds	temp,WEEKDAYS						;

 ldi	XH,143
 ldi	XL,141
 cpi	temp,0
 breq	WEEKDAY_END
 ldi	XH,130
 ldi	XL,146
 cpi	temp,1
 breq	WEEKDAY_END
 ldi	XH,145
 ldi	XL,144
 cpi	temp,2
 breq	WEEKDAY_END
 ldi	XH,151
 ldi	XL,146
 cpi	temp,3
 breq	WEEKDAY_END
 ldi	XH,143
 ldi	XL,146
 cpi	temp,4
 breq	WEEKDAY_END
 ldi	XH,145
 ldi	XL,129
 cpi	temp,5
 breq	WEEKDAY_END
 ldi	XH,130
 ldi	XL,145

WEEKDAY_END:
 mov	temp,XH
 rcall	GLCD_PUT_CHAR 
 mov	temp,XL
 rcall	GLCD_PUT_CHAR 



 rcall	d13ms

rjmp	MAIN_LOOP




;***********************************************************************************
;***********************************************************************************
;***********************************************************************************


;-----------------------------------------------------------------------------------
DAYS_SET:
 lds	temp0,DAY_L							; ��������� ������� �������� ���
 lds	temp1,DAY_H
 inc	temp0								; �������������� ������� ����
 cpi	temp0,10							; ���� ����� ������ 10, 
 breq	DAY_SET1							;  ����� ��������� � ���������� �������� ����
 rjmp	DAY_SET_END							;  ����� ��������� �������� � �������
DAY_SET1:
 clr	temp0								; ���������� ������� ����
 inc	temp1								; �������������� ������� ����
 cpi	temp1,3								; ��������� ������� ����� � 6
 breq	DAY_SET_0						; ���� ������ ������ 60, ����� ���������� �������� �����
 rjmp	DAY_SET_END						;
DAY_SET_0:
 clr	temp0
 clr	temp1

DAY_SET_END:
 mov	temp,temp0
 andi	temp1,0x03
 swap	temp1
 or		temp,temp1
;	��������� � ������ ������� ����������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 ldi	i2cdata,0x05						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d13ms								; ���� ���� ������ ���������� � ������ ����

rjmp	MAIN_LOOP



;-----------------------------------------------------------------------------------
MONTH_SET:
 lds	temp0,MONTH_L						; ��������� ������� �������� �����
 andi	temp0,0x0F
 lds	temp1,MONTH_H
 andi	temp1,0x0F
 inc	temp0								; �������������� ������� �����
 cpi	temp0,10							; ���� ����� ������ 10, 
 breq	MONTH_SET1							;  ����� ��������� � ���������� �������� �����
 rjmp	MONTH_SET_END						;  ����� ��������� �������� � �������
MONTH_SET1:
 clr	temp0								; ���������� ������� �����
 inc	temp1								; �������������� ������� �����
 cpi	temp1,1								; ��������� ������� ����� � 6
 breq	MONTH_SET_0							; ���� ������ ������ 60, ����� ���������� �������� �����
 rjmp	MONTH_SET_END						;
MONTH_SET_0:
 ldi	temp0,1
 clr	temp1
 lds	temp,WEEKDAYS
 inc	temp
 andi	temp,0x07
 sts	WEEKDAYS,temp

MONTH_SET_END:
 lds	temp,WEEKDAYS
 andi	temp,0x07							; �������� �� ������ ������ 5 ���, ��������� ��������� ���
 swap	temp								; ����� ������� �������
 lsl	temp								; �������� ����� �� ���
 andi	temp,0b11100000
 or		temp,temp0
 andi	temp1,0x01
 swap	temp1
 or		temp,temp1
;	��������� � ������ ������� ����������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 ldi	i2cdata,0x6							; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d13ms								; ���� ���� ������ ���������� � ������ ����

rjmp	MAIN_LOOP




;-----------------------------------------------------------------------------------
MINUTES_SET:
;	������������� ������
 lds	temp0,MINUTES_L						; ��������� ������� �������� �����
 lds	temp1,MINUTES_H
 inc	temp0								; �������������� ������� �����
 cpi	temp0,10							; ���� ����� ������ 10, 
 breq	MINUTES_SET1						;  ����� ��������� � ���������� �������� �����
 rjmp	MINUTES_SET_END						;  ����� ��������� �������� � �������
MINUTES_SET1:
 clr	temp0								; ���������� ������� �����
 inc	temp1								; �������������� ������� �����
 cpi	temp1,6								; ��������� ������� ����� � 6
 breq	MINUTES_SET_0						; ���� ������ ������ 60, ����� ���������� �������� �����
 rjmp	MINUTES_SET_END						;
MINUTES_SET_0:
 clr	temp0
 clr	temp1

MINUTES_SET_END:
 mov	temp,temp0
 swap	temp1
 andi	temp1,0xF0
 or		temp,temp1
;	��������� � ������ ������� ����������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 ldi	i2cdata,0x03						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d13ms								; ���� ���� ������ ���������� � ������ ����

;	������������� �������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 ldi	i2cdata,0x02						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cdata,0x00						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d13ms								; ���� ���� ������ ���������� � ������ ����

rjmp	MAIN_LOOP



;-----------------------------------------------------------------------------------
HOUR_SET:
;	������������� ����
 lds	temp0,HOUR_L						; ��������� ������� �������� �����
 lds	temp1,HOUR_H
 inc	temp0								; �������������� ������� �����
 cpi	temp0,10							; ���� ����� ������ 10, 
 breq	HOUR_SET1							;  ����� ��������� � ���������� �������� �����
 rjmp	HOUR_SET_END						;  ����� ��������� �������� � �������
HOUR_SET1:
 clr	temp0								; ���������� ������� �����
 inc	temp1								; �������������� ������� �����
 cpi	temp1,2								; ��������� ������� ����� � 6
 breq	HOUR_SET_0							; ���� ������ ������ 60, ����� ���������� �������� �����
 rjmp	HOUR_SET_END						;
HOUR_SET_0:
 clr	temp0
 clr	temp1

HOUR_SET_END:
 mov	temp,temp0
 swap	temp1
 andi	temp1,0xF0
 or		temp,temp1
;	��������� � ������ ������� ����������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 ldi	i2cdata,0x04						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������ FF
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d13ms								; ���� ���� ������ ���������� � ������ ����

rjmp	MAIN_LOOP




;***********************************************************************************
;*                                                                                 *
;*                                                                                 *
;*                     ��������� ��� ������� CGM12864GSLI                          *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;-----------------------------------------------------------------------------------
;	������������� �������
;-----------------------------------------------------------------------------------
GLCD_INIT:
 cbi	GLCD_CMD_PORT,GLCD_RS				; RS=0
 cbi	GLCD_CMD_PORT,GLCD_WR				; WR=0
 cbi	GLCD_CMD_PORT,GLCD_E				; E =0
 sbi	GLCD_CMD_PORT,GLCD_CS				; CS=1

;	�������� ������� ��������� �������������

 ldi	temp,RESET_DISPLAY
 rcall	GLCD_CMD_WR
 rcall	d1ms
 ldi	temp,BIAS_SET
 rcall	GLCD_CMD_WR
 ldi	temp,ADC_SELECT
 rcall	GLCD_CMD_WR
 ldi	temp,COMMON_OUTPUTMODE_SELECT
 rcall	GLCD_CMD_WR
 ldi	temp,RES_RATIO
 rcall	GLCD_CMD_WR
 ldi	temp,ELECVOLUME_SET
 rcall	GLCD_CMD_WR
 ldi	temp,VOLREGISTOR_SET
 rcall	GLCD_CMD_WR
 ldi	temp,POWERCONTROL_SET
 rcall	GLCD_CMD_WR
 ldi	temp,BOOST_RATIO
 rcall	GLCD_CMD_WR
 ldi	temp,0x01
 rcall	GLCD_CMD_WR
 ldi	temp,STARTLINE0_SET
 rcall	GLCD_CMD_WR
 ldi	temp,DISPALL_ON
 rcall	GLCD_CMD_WR
 rcall	d50ms
 ldi	temp,DISPALL_OFF
 rcall	GLCD_CMD_WR
 ldi	temp,0xA6							; display normal
 rcall	GLCD_CMD_WR
 ldi	temp,DISPLAY_ON
 rcall	GLCD_CMD_WR

ret


;-----------------------------------------------------------------------------------
;	�������� ������� � �������
;-----------------------------------------------------------------------------------
GLCD_CMD_WR:
 out	GLCD_DATA_PORT,temp					; ���������� ������ � ���� D7-D0
 cbi	GLCD_CMD_PORT,GLCD_RS				; RS=0
 cbi	GLCD_CMD_PORT,GLCD_WR				; WR=0
 cbi	GLCD_CMD_PORT,GLCD_CS				; CS=0
 sbi	GLCD_CMD_PORT,GLCD_E				; E =1

 rcall	GLCD_DELAY							; ��������
 
 cbi	GLCD_CMD_PORT,GLCD_E				; E =0
 sbi	GLCD_CMD_PORT,GLCD_CS				; CS=1 

 rcall	GLCD_DELAY							; ��������

ret

;-----------------------------------------------------------------------------------
;	����� ������ � �������
;-----------------------------------------------------------------------------------
GLCD_DATA_WR:
 out	GLCD_DATA_PORT,temp					; ���������� ������ � ���� D7-D0
 sbi	GLCD_CMD_PORT,GLCD_RS				; RS=1
 cbi	GLCD_CMD_PORT,GLCD_WR				; WR=0
 cbi	GLCD_CMD_PORT,GLCD_CS				; CS=0
 sbi	GLCD_CMD_PORT,GLCD_E				; E =1

 rcall	GLCD_DELAY							; ��������
 
 cbi	GLCD_CMD_PORT,GLCD_E				; E =0
 cbi	GLCD_CMD_PORT,GLCD_RS				; RS=0
 sbi	GLCD_CMD_PORT,GLCD_CS				; CS=1 

 rcall	GLCD_DELAY							; ��������

ret



;-----------------------------------------------------------------------------------
;	������������� ���������� X
;-----------------------------------------------------------------------------------
GLCD_X:
 ldi	temp1,4
 add	temp,temp1							; ���������� �������� �� ��� X
 push	temp								; ��������� �������� ������ �������
 swap	temp								; ������ ������� � ��������
 cbr	temp,0xF0
 ori	temp,0x10							; ������ ����� �� ����� �������
 rcall	GLCD_CMD_WR							; �������� ������� ������� ������ �������
 pop	temp								; ��������������� �������� ������ �������
 cbr	temp,0xF0
 ori	temp,0x00							; ������ ����� �� ����� �������
 rcall	GLCD_CMD_WR							; �������� ������� ������� ������ �������
ret



;-----------------------------------------------------------------------------------
;	������������� ���������� Y
;-----------------------------------------------------------------------------------
GLCD_Y:
 ori	temp,0xB0							; ������ ����� �� ����� ������
 rcall	GLCD_CMD_WR

ret



;-----------------------------------------------------------------------------------
;	������ ������ �� �������
;-----------------------------------------------------------------------------------
GLCD_DATA_RD:
ret



;-----------------------------------------------------------------------------------
;	������ ������� �� �������
;-----------------------------------------------------------------------------------
GLCD_CMD_RD:
ret



;-----------------------------------------------------------------------------------
;	����� ������ ������� �� �������
;-----------------------------------------------------------------------------------
GLCD_PUT_CHAR:
 ldi	ZL,low(FONT_TAB*2)					; ���������� ������ ������� ����� 
 ldi	ZH,high(FONT_TAB*2)					;  ������� �� ��������� �������

GLCD_PUT_ARRAY:								; ����� ������� �� ������ �������
                                            ;  [temp = char] - ����� �������
 clr	temp1
 clr	temp0
 mov	char_ind,temp						; �������� ������ �� �������
                                            ; adres=n*6+adres0
 add	temp,char_ind						; ���������� temp (n*2) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*3) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*4) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*5) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*6) 
 adc	temp1,temp0
 
 add	ZL,temp								; ������� ����� ������� n � �������
 adc	ZH,temp1							;  adres=adres+n*6

 ldi	temp1,6
 clr	r0
GLCD_PUT_ARRAY_6:
 lpm										; ������ �� ������� ���
 adiw	ZL,1								; ������� ��������� �������
 mov	temp,r0								; �������� ��������� ��� �� ������� � ������� ��� ������ �� �������
 rcall	GLCD_DATA_WR						; ������� �� ������� ����� �������
 dec	temp1
 brne	GLCD_PUT_ARRAY_6					; ��������� ���� ����� ���

ret


;-----------------------------------------------------------------------------------
;	����� �������� ������� �� ������� Fonts 24*40
;-----------------------------------------------------------------------------------
GLCD_PUT_CHAR_B:
 mov	char_ind,temp
 ldi	ZL,low(FONT2_TAB*2)					; ���������� ������ ������� ����� 
 ldi	ZH,high(FONT2_TAB*2)				;  ������� �� ��������� �������

 ldi	temp0,120
 clr	temp1
 mul	temp,temp0
 add	ZL,mulL
 adc	ZH,mulH

 ldi	temp1,24							; ������������� ������� �������� ������� (24 �������)
 clr	temp0								; ���������� ������� �����
 clr	r0
GLCD_PUT_ARRAY_24:
 lpm										; ������ �� ������� ���
 adiw	ZL,1								; ������� ��������� �������
 mov	temp,r0								; �������� ��������� ��� �� ������� � ������� ��� ������ �� �������
 rcall	GLCD_DATA_WR						; ������� �� ������� ����� �������
 dec	temp1								; ��������� ������� ��������
 brne	GLCD_PUT_ARRAY_24					; ��������� ���� 24 ����
 mov	temp,GLCDY
 inc	temp0								; ����������� ������� �����
 add	temp,temp0							; ��������� �� ��������� �������	
 rcall	GLCD_Y
 mov	temp,GLCDX
 rcall	GLCD_X
 ldi	temp1,24
 cpi	temp0,5
 brne	GLCD_PUT_ARRAY_24					; ��������� ���� 5 ���
ret


;-----------------------------------------------------------------------------------
;	������� �������  �� ������, �.�. ���������� ��� 0x00 (8 ����� * 128 �������)
;-----------------------------------------------------------------------------------
GLCD_CLR:
 clr	cicle_count
 clr	temp_count
CLR_GLCD_8:
 ldi	temp,0
 rcall	GLCD_X
 mov	temp,temp_count
 rcall	GLCD_Y								; ������ ���������� Y
 clr	cicle_count
CLR_GLCD_132:
 clr	temp
 rcall	GLCD_DATA_WR						; ����� ������ �������
 inc	cicle_count							; ����������� ������� �������
 cpi	cicle_count,132						; ���� �� ����� �� ��������� ������� � ������,
 brne	CLR_GLCD_132						;   ����� ���������� ���������� 0x00

 inc	temp_count							; ����������� ������� �����
 mov	temp,temp_count
 cpi	temp,8								; ���� ��������� ����� �� ����� 8,
 brne	CLR_GLCD_8							;   ����� ������� ��������� ������

 ldi	temp,0								; ������������� ������ �� ������ ��������� X=0,Y=0
 rcall	GLCD_X
 ldi	temp,0
 rcall	GLCD_Y

ret



;-----------------------------------------------------------------------------------
;	�������� ��� �������
;-----------------------------------------------------------------------------------
GLCD_DELAY:
 ldi	temp,GLCD_DEL						; ��������� ����������� � �������
GLCD_D:
 dec	temp								; �������� 1
 brne	GLCD_D
ret









;***********************************************************************************
;*                                                                                 *
;*                                                                                 *
;*            �������������� ��������� ����� � ����� ��� ����������                *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;   ���������� ����������: temp, temp1, adwL, adwH, mulL, mulH
                          
; Bin2ToBcd5
; ==========
; converts a 16-bit-binary to a 5-digit-BCD
; In: 16-bit-binary in adwH,adwL
; Out: 5-digit-BCD
; Used registers:temp
; Called subroutines: Bin2ToDigit
;
Bin2ToBCD5:
 ldi	temp,high(10000)						; ��������� 10000 ������
 mov	mulH,temp
 ldi	temp,low(10000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������, ��������� � ���������� temp
 sts	BCD,temp							; ���� �� ����, ����� ���������� ���������
Bin2ToBCD4:
 ldi	temp,high(1000)						; ��������� 1000 ������
 mov	mulH,temp
 ldi	temp,low(1000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+1,temp							; ��������� � ���������� temp 
Bin2ToBCD3:
 clr	mulH								;  ��������� 100 ������
 ldi	temp,100
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+2,temp							; ��������� � ���������� temp
Bin2ToBCD2:
 clr	mulH								; ��������� 10 ������
 ldi	temp,10
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+4,adwL							; ������� �������� � adiw0
 sts	BCD+3,temp							; ��������� � ���������� temp
ret

; Bin2ToDigit
; ===========
; converts one decimal digit by continued subraction of a binary coded decimal
; Used by: Bin2ToBcd5
; In: 16-bit-binary in adw1,adw0, binary coded decimal in mul0,mul1
; Out: Result in temp
; Used registers: adiw0,adiw1, mul0,mul1, temp
; Called subroutines: -

Bin2ToDigit:
 clr	temp								; digit count is zero
Bin2ToDigita:
 cp		adwH,mulH							; Number bigger than decimal?
 brcs	Bin2ToDigitc						; MSB smaller than decimal
 brne	Bin2ToDigitb						; MSB bigger than decimal
 cp		adwL,mulL							; LSB bigger or equal decimal
 brcs	Bin2ToDigitc						; LSB smaller than decimal
Bin2ToDigitb:
 sub	adwL,mulL							; Subtract LSB decimal
 sbc	adwH,mulH							; Subtract MSB decimal
 inc	temp								; Increment digit count
 rjmp	Bin2ToDigita						; Next loop
Bin2ToDigitc:
ret											; done







;******************************************************************************
;*	i2c_hp_delay
;*	i2c_qp_delay
;******************************************************************************

i2c_hp_delay:
 ldi	i2cdelay,2
i2c_hp_delay_loop:
 dec	i2cdelay
 brne	i2c_hp_delay_loop
ret

i2c_qp_delay:
 ldi	i2cdelay,1	
i2c_qp_delay_loop:
 dec	i2cdelay
 brne	i2c_qp_delay_loop
ret


;******************************************************************************
;* FUNCTION
;*	i2c_rep_start
;******************************************************************************

i2c_rep_start:
 sbi	DDRC,SCLP							; force SCL low
 cbi	DDRC,SDAP							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP							; release SCL
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;* FUNCTION
;*	i2c_start
;******************************************************************************

i2c_start:				
 mov	i2cdata,i2cadr						; copy address to transmitt register
 sbi	DDRC,SDAP							; force SDA low
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;* FUNCTION
;*	i2c_write
;******************************************************************************

i2c_write:
 sec										; set carry flag
 rol	i2cdata								; shift in carry and out bit one
 rjmp	i2c_write_first
i2c_write_bit:
 lsl	i2cdata								; if transmit register empty
i2c_write_first:
 breq	i2c_get_ack							; goto get acknowledge
 sbi	DDRC,SCLP							; force SCL low

 brcc	i2c_write_low						; if bit high
 nop										; (equalize number of cycles)
 cbi	DDRC,SDAP							; release SDA
 rjmp	i2c_write_high
i2c_write_low:								; else
 sbi	DDRC,SDAP							; force SDA low
 rjmp	i2c_write_high						; (equalize number of cycles)
i2c_write_high:
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP							; release SCL
 rcall	i2c_hp_delay						; half period delay

 rjmp	i2c_write_bit


;******************************************************************************
;* FUNCTION
;*	i2c_get_ack
;******************************************************************************

i2c_get_ack:
 sbi	DDRC,SCLP							; force SCL low
 cbi	DDRC,SDAP							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP							; release SCL

i2c_get_ack_wait:
 sbis	PINC,SCLP							; wait SCL high 
					                        ;(In case wait states are inserted)
 rjmp	i2c_get_ack_wait

 clc										; clear carry flag
 sbic	PINC,SDAP							; if SDA is high
 sec										; set carry flag
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;* FUNCTION
;*	i2c_do_transfer
;******************************************************************************

i2c_do_transfer:
 sbrs	i2cadr,b_dir						; if dir = write
 rjmp	i2c_write							; goto write data


;******************************************************************************
;* FUNCTION
;*	i2c_read
;******************************************************************************

i2c_read:
 rol	i2cstat								; store acknowledge
					                        ; (used by i2c_put_ack)
 ldi	i2cdata,0x01						; data = 0x01
i2c_read_bit:                               ; do
 sbi	DDRC,SCLP							; force SCL low
 rcall	i2c_hp_delay						; half period delay

 cbi	DDRC,SCLP							; release SCL
 rcall	i2c_hp_delay						; half period delay

 clc										; clear carry flag
 sbic	PINC,SDAP							; if SDA is high
 sec										; set carry flag

 rol	i2cdata								; store data bit
 brcc	i2c_read_bit						; while receive register not full


;******************************************************************************
;* FUNCTION
;*	i2c_put_ack
;******************************************************************************

i2c_put_ack:
 sbi	DDRC,SCLP							; force SCL low

 ror	i2cstat								; get status bit
 brcc	i2c_put_ack_low						; if bit low goto assert low
 cbi	DDRC,SDAP							; release SDA
 rjmp	i2c_put_ack_high
i2c_put_ack_low:							; else
 sbi	DDRC,SDAP							; force SDA low
i2c_put_ack_high:

 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP							; release SCL
i2c_put_ack_wait:
 sbis	PINC,SCLP							; wait SCL high
 rjmp	i2c_put_ack_wait
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;* FUNCTION
;*	i2c_stop
;******************************************************************************

i2c_stop:
 sbi	DDRC,SCLP							; force SCL low
 sbi	DDRC,SDAP							; force SDA low
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP							; release SCL
 rcall	i2c_qp_delay						; quarter period delay
 cbi	DDRC,SDAP							; release SDA
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;* FUNCTION
;*	i2c_init
;******************************************************************************

i2c_init:
 clr	i2cstat								; clear I2C status register (used
											; as a temporary register)
ret


/*
;------------------------------------------------------------------------------
;  ������ ������ � I2C 


rcall	i2c_init		; initialize I2C interface

;**** Write data => Adr(00) = 0x55 ****

 ldi	i2cadr,$A0+i2cwr					; Set device address and write
 rcall	i2c_start							; Send start condition and address

 ldi	i2cdata,$00							; Write word address (0x00)
 rcall	i2c_do_transfer						; Execute transfer

 ldi	i2cdata,$55							; Set write data to 01010101b
 rcall	i2c_do_transfer						; Execute transfer

 rcall	i2c_stop							; Send stop condition

;**** Read data => i2cdata = Adr(00) ****

 ldi	i2cadr,$A0+i2cwr					; Set device address and write
 rcall	i2c_start							; Send start condition and address

 ldi	i2cdata,$00							; Write word address
 rcall	i2c_do_transfer						; Execute transfer

 ldi	i2cadr,$A0+i2crd					; Set device address and read
 rcall	i2c_rep_start						; Send repeated start condition and address

 sec										; Set no acknowledge (read is followed by a stop condition)
                        ; ����� sec ���� �� ��������� ��������� ����, ����� ������ �� �����
 rcall	i2c_do_transfer						; Execute transfer (read)

 rcall	i2c_stop							; Send stop condition - releases bus


*/





;********************************************************************************
;**                                                                            **
;**                                                                            **
;**      ��������� ���������� - 0x0009 - ������������ �������/�������� �0      **
;**                                                                            **
;**                                                                            **
;**                                                                            **
;********************************************************************************
/*
aOVF0:


reti

*/










;************************************************************************************
;************************************************************************************
;**                                                                                **
;**                                                                                **
;**                 ������������ �������� ��� ������� ������ 4���                  **
;**                                                                                **
;**                                                                                **
;************************************************************************************
;************************************************************************************
;-------------------------------------------------------------------------
;	�������� �� 0,25�� (250���)

d025ms:
 ldi YL,low(248)                            ; �������� � YH:YL ��������� 497
 ldi YH,high(248)

d025_1:
 sbiw YL,1                                  ; ��������� �� ����������� YH:YL
                                            ;  �������
 brne d025_1                                ; ���� ���� Z<>0 (��������� ����������
                                            ;  ���������� ������� �� ����� ����), ��
									        ;  ������� �� ����� d05_1
ret





;-------------------------------------------------------------------------
;	�������� �� 0,5�� (500���)

d05ms:
 ldi	YL,low(497)							; �������� � YH:YL ��������� 497
 ldi	YH,high(497)

d05_1:
 sbiw	YL,1								; ��������� �� ����������� YH:YL �������
 brne	d05_1								; ���� ���� Z<>0 (��������� ����������
											;  ���������� ������� �� ����� ����), ��
											;  ������� �� ����� d05_1
ret





;-------------------------------------------------------------------------
;	�������� 1 ms

d1ms:  
 ldi	temp,5
m2:
 ldi	temp1,255
m3:
 dec	temp1
 brne	m3
 dec	temp
 brne	m2
ret





;-------------------------------------------------------------------------
;	�������� 2,8 ms

d2_8ms:  
 ldi	temp,15
m:
 ldi	temp1,255
m1:
 dec	temp1
 brne	m1
 dec	temp
 brne	m
ret





;-------------------------------------------------------------------------
;	�������� 13 ms

d13ms:  
 ldi	temp,70								; 100-19ms, 70-13ms
ms:
 ldi	temp1,255
ms1:
 dec	temp1
 brne	ms1
 dec	temp
 brne	ms
ret





;-------------------------------------------------------------------------
;	�������� 20 ms

d20ms:  
 ldi	temp,100
m20s:
 ldi	temp1,255
m20s1:
 dec	temp1
 brne	m20s1
 dec	temp
 brne	m20s
ret





;-------------------------------------------------------------------------
;	�������� �� 50��

d50ms:
 ldi	temp,100

d50_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 dec	temp								; ��������� ������� �� temp
 brne	d50_1								; ���� ��������� �� ����� ����, ������� �� ����� d50_1
ret





;-------------------------------------------------------------------------
;	�������� �� 100��

d100ms:
 ldi	temp,200							; �������� � temp ��������� 200

d100_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 dec	temp								; ��������� ������� �� temp
 brne	d100_1								; ���� ��������� �� ����� ����, ������� �� ����� d100_1
ret





;-------------------------------------------------------------------------
;	�������� �� 300��

d300ms:
 ldi	XL,low(700)							; �������� � YH:YL ��������� 700
 ldi	XH,high(700)

d300_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 sbiw	XL,1								; ��������� ������� �� ����������� XH:XL
 brne	d300_1								; ���� ��������� �� ����� ����, ������� �� ����� d500_1
ret





;-------------------------------------------------------------------------
;	�������� �� 500��

d500ms:
 ldi	XL,low(1000)						; �������� � YH:YL ��������� 1000
 ldi	XH,high(1000)

d500_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 sbiw	XL,1								; ��������� ������� �� ����������� XH:XL
 brne	d500_1								; ���� ��������� �� ����� ����, ������� �� ����� d500_1
ret









;***********************************************************************************
;***********************************************************************************
;*
;*
;*         ������� ���� � �������
;*
;*
;***********************************************************************************
;***********************************************************************************

;-----------------------------------------------------------------------------------
;	"������"
FRAZA1:
.db 159,173,162,160,176,191,32,32,32,23


;-----------------------------------------------------------------------------------
;	"�������"
FRAZA2:
.db 148,165,162,176,160,171,191,32,32,23


;-----------------------------------------------------------------------------------
;	"�����"
FRAZA3:
.db 140,160,176,178,160,32,32,32,32,23


;-----------------------------------------------------------------------------------
;	"������"
FRAZA4:
.db 128,175,176,165,171,191,32,32,32,23


;-----------------------------------------------------------------------------------
;	"���"
FRAZA5:
.db 140,160,191,32,32,32,32,32,32,23


;-----------------------------------------------------------------------------------
;	"����"
FRAZA6:
.db 136,190,173,191,32,32,32,32,32,23


;-----------------------------------------------------------------------------------
;	"����"
FRAZA7:
.db 136,190,171,191,32,32,32,32,32,23


;-----------------------------------------------------------------------------------
;	"�������"
FRAZA8:
.db 128,162,163,179,177,178,160,32,32,23


;-----------------------------------------------------------------------------------
;	"��������"
FRAZA9:
.db 145,165,173,178,191,161,176,191,32,23


;-----------------------------------------------------------------------------------
;	"�������"
FRAZA10:
.db 142,170,178,191,161,176,191,32,32,23


;-----------------------------------------------------------------------------------
;	"������"
FRAZA11:
.db 141,174,191,161,176,191,32,32,32,23


;-----------------------------------------------------------------------------------
;	"�������"
FRAZA12:
.db 132,165,170,160,161,176,191,32,32,23







;������� ����� ���������������
;------------------------------------------------------------------------------
FONT_TAB:         ; �������� ������ ������
;------------------------------------------------------------------------------
											;������  Dec   Hex
.db 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E 		;   0    (0)   (0)
.db 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 		;   1    (1)   (1)
.db 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 		;   2    (2)   (2)
.db 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 		;   3    (3)   (3)
.db 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 		;   4    (4)   (4)
.db 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 		;   5    (5)   (5)
.db 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 		;   6    (6)   (6)
.db 0x00, 0x03, 0x01, 0x71, 0x09, 0x07 		;   7    (7)   (7)
.db 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 		;   8    (8)   (8)
.db 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E 		;   9    (9)   (9)
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 		; �����  (10)  (A)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (11)  (B)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (12)  (C)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (13)  (D)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (14)  (E)
.db 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00		;        (15)  (F)
.db 0x00, 0x7F, 0x3E, 0x1C, 0x08, 0x00 		;        (16)  (10)
.db 0x00, 0x08, 0x1C, 0x3E, 0x7F, 0x00 		;        (17)  (11)
.db 0x00, 0x06, 0x05, 0x00, 0x06, 0x05 		;        (18)  (12)
.db 0x00, 0x05, 0x03, 0x00, 0x05, 0x03 		;        (19)  (13)
.db 0x00, 0x44, 0x66, 0x77, 0x66, 0x44 		;        (20)  (14)
.db 0x00, 0x11, 0x33, 0x77, 0x33, 0x11 		;        (21)  (15)
.db 0x00, 0x1C, 0x3E, 0x3E, 0x3E, 0x1C 		;        (22)  (16)
.db 0x00, 0x10, 0x38, 0x54, 0x10, 0x1F 		;        (23)  (17)
.db 0x00, 0x04, 0x02, 0x7F, 0x02, 0x04 		;        (24)  (18)
.db 0x00, 0x10, 0x20, 0x7F, 0x20, 0x10 		;        (25)  (19)
.db 0x00, 0x10, 0x10, 0x54, 0x38, 0x10 		;        (26)  (1A)
.db 0x00, 0x10, 0x38, 0x54, 0x10, 0x10 		;        (27)  (1B)
.db 0x00, 0x40, 0x44, 0x4A, 0x51, 0x40 		;        (28)  (1C)
.db 0x00, 0x40, 0x51, 0x4A, 0x44, 0x40 		;        (29)  (1D)
.db 0x00, 0x20, 0x30, 0x38, 0x30, 0x20 		;        (30)  (1E)
.db 0x00, 0x08, 0x18, 0x38, 0x18, 0x08 		;        (31)  (1F)
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00		; space  (32)  (20)
.db 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00 		;   !    (33)  (21)
.db 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 		;   "    (34)  (22)
.db 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14 		;   #    (35)  (23)
.db 0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12 		;   $    (36)  (24)
.db 0x00, 0x23, 0x13, 0x08, 0x64, 0x62 		;   %    (37)  (25)
.db 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 		;   &    (38)  (26)
.db 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 		;   '    (39)  (27)
.db 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00 		;   (    (40)  (28)
.db 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00 		;   )    (41)  (29)
.db 0x00, 0x0A, 0x04, 0x1F, 0x04, 0x0A 		;   *    (42)  (2A)
.db 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 		;   +    (43)  (2B)
.db 0x00, 0x00, 0x50, 0x30, 0x00, 0x00 		;   ,    (44)  (2C)
.db 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 		;   -    (45)  (2D)
.db 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 		;   .    (46)  (2E)
.db 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 		;   /    (47)  (2F)
.db 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E 		;   0    (48)  (30)
.db 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 		;   1    (49)  (31)
.db 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 		;   2    (50)  (32)
.db 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 		;   3    (51)  (33)
.db 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 		;   4    (52)  (34)
.db 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 		;   5    (53)  (35)
.db 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 		;   6    (54)  (36)
.db 0x00, 0x03, 0x01, 0x71, 0x09, 0x07 		;   7    (55)  (37)
.db 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 		;   8    (56)  (38)
.db 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E 		;   9    (57)  (39)
.db 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 		;   :    (58)  (3A)
.db 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 		;   ;    (59)  (3B)
.db 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 		;   <    (60)  (3C)
.db 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 		;   =    (61)  (3D)
.db 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 		;   >    (62)  (3E)
.db 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 		;   ?    (63)  (3F)
.db 0x00, 0x32, 0x49, 0x79, 0x41, 0x3E 		;   @    (64)  (40)
;--------------------------------------------
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E 		;   A    (65)  (41)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 		;   B    (66)  (42)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 		;   C    (67)  (43)
.db 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C 		;   D    (68)  (44)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 		;   E    (69)  (45)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 		;   F    (70)  (46)
.db 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A 		;   G    (71)  (47)
.db 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F 		;   H    (72)  (48)
.db 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 		;   I    (73)  (49)
.db 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 		;   J    (74)  (4A)
.db 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 		;   K    (75)  (4B)
.db 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 		;   L    (76)  (4C)
.db 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F 		;   M    (77)  (4D)
.db 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F 		;   N    (78)  (4E)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E 		;   O    (79)  (4F)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 		;   P    (80)  (50)
.db 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E 		;   Q    (81)  (51)
.db 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 		;   R    (82)  (52)
.db 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 		;   S    (83)  (53)
.db 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 		;   T    (84)  (54)
.db 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F 		;   U    (85)  (55)
.db 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F 		;   V    (86)  (56)
.db 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F 		;   W    (87)  (57)
.db 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 		;   X    (88)  (58)
.db 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 		;   Y    (89)  (59)
.db 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 		;   Z    (90)  (5A)
.db 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00 		;   [    (91)  (5B)
.db 0x00, 0x02, 0x04, 0x08, 0x10, 0x20 		;   \    (92)  (5C)
.db 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 		;   ]    (93)  (5D)
.db 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 		;   ^    (94)  (5E)
.db 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 		;   _    (95)  (5F)
.db 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 		;   '    (96)  (60)
;--------------------------------------------
.db 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 		;   a    (97)  (61)
.db 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 		;   b    (98)  (62)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 		;   c    (99)  (63)
.db 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F 		;   d    (100)  (64)
.db 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 		;   e    (101)  (65)
.db 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 		;   f    (102)  (66)
.db 0x00, 0x0C, 0x52, 0x52, 0x52, 0x3E 		;   g    (103)  (67)
.db 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 		;   h    (104)  (68)
.db 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 		;   i    (105)  (69)
.db 0x00, 0x20, 0x40, 0x44, 0x3D, 0x00 		;   j    (106)  (6A)
.db 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 		;   k    (107)  (6B)
.db 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 		;   l    (108)  (6C)
.db 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 		;   m    (109)  (6D)
.db 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 		;   n    (110)  (6E)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 		;   o    (111)  (6F)
.db 0x00, 0x7C, 0x14, 0x14, 0x14, 0x08 		;   p    (112)  (70)
.db 0x00, 0x08, 0x14, 0x14, 0x18, 0x7C 		;   q    (113)  (71)
.db 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 		;   r    (114)  (72)
.db 0x00, 0x08, 0x54, 0x54, 0x54, 0x20 		;   s    (115)  (73)
.db 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 		;   t    (116)  (74)
.db 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C 		;   u    (117)  (75)
.db 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C 		;   v    (118)  (76)
.db 0x00, 0x3C, 0x40, 0x38, 0x40, 0x3C		;   w    (119)  (77)
.db 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 		;   x    (120)  (78)
.db 0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C 		;   y    (121)  (79)
.db 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 		;   z    (122)  (7A)
.db 0x00, 0x00, 0x08, 0x36, 0x41, 0x00 		;   {    (123)  (7B)
.db 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00 		;   |    (124)  (7C)
.db 0x00, 0x00, 0x41, 0x36, 0x08, 0x00 		;   }    (125)  (7D)
.db 0x00, 0x08, 0x04, 0x08, 0x04, 0x08 		;   ~    (126)  (7E)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		; black  (127)  (7F)
;--------------------------------------------
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E		;   �    (128)  (80)
.db 0x00, 0x7F, 0x45, 0x45, 0x45, 0x39 		;   �    (129)  (81)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 		;   �    (130)  (82)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x03 		;   �    (131)  (83)
.db 0x00, 0xC0, 0x7E, 0x41, 0x7F, 0xC0		;   �    (132)  (84)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 		;   �    (133)  (85)
.db 0x00, 0x77, 0x08, 0x7F, 0x08, 0x77 		;   �    (134)  (86)
.db 0x00, 0x41, 0x49, 0x49, 0x49, 0x36 		;   �    (135)  (87)
.db 0x00, 0x7F, 0x10, 0x08, 0x04, 0x7F 		;   �    (136)  (88)
.db 0x00, 0x7E, 0x20, 0x13, 0x08, 0x7E 		;   �    (137)  (89)
.db 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 		;   �    (138)  (8A)
.db 0x00, 0x40, 0x3C, 0x02, 0x01, 0x7F 		;   �    (139)  (8B)
.db 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F 		;   �    (140)  (8C)
.db 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F 		;   �    (141)  (8D)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E 		;   �    (142)  (8E)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x7F 		;   �    (143)  (8F)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 		;   �    (144)  (90)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 		;   �    (145)  (91)
.db 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 		;   �    (146)  (92)
.db 0x00, 0x07, 0x48, 0x48, 0x48, 0x3F 		;   �    (147)  (93)
.db 0x00, 0x1C, 0x22, 0x7F, 0x22, 0x1C 		;   �    (148)  (94)
.db 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 		;   �    (149)  (95)
.db 0x00, 0x3F, 0x20, 0x20, 0x3F, 0x60 		;   �    (150)  (96)
.db 0x00, 0x0F, 0x08, 0x08, 0x08, 0x7F 		;   �    (151)  (97)
.db 0x00, 0x7F, 0x40, 0x7E, 0x40, 0x7F 		;   �    (152)  (98)
.db 0x00, 0x3F, 0x20, 0x3C, 0x20, 0x7F 		;   �    (153)  (99)
.db 0x00, 0x01, 0x7F, 0x48, 0x48, 0x30 		;   �    (154)  (9A)
.db 0x00, 0x7F, 0x48, 0x30, 0x00, 0x7F 		;   �    (155)  (9B)
.db 0x00, 0x7F, 0x48, 0x48, 0x48, 0x30		;   �    (156)  (9C)
.db 0x00, 0x22, 0x49, 0x49, 0x49, 0x3E 		;   �    (157)  (9D)
.db 0x00, 0x7F, 0x08, 0x3E, 0x41, 0x3E 		;   �    (158)  (9E)
.db 0x00, 0x46, 0x29, 0x19, 0x09, 0x7F 		;   �    (159)  (9F)
.db 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 		;   �    (160)  (A0)
.db 0x00, 0x3C, 0x4A, 0x4A, 0x49, 0x30 		;   �    (161)  (A1)
.db 0x00, 0x7C, 0x54, 0x54, 0x28, 0x00   	;   �    (162)  (A2)
.db 0x00, 0x7C, 0x04, 0x04, 0x0C, 0x00 		;   �    (163)  (A3)
.db 0x00, 0xC0, 0x78, 0x44, 0x7C, 0xC0 		;   �    (164)  (A4)
.db 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 		;   �    (165)  (A5)
.db 0x00, 0x6C, 0x10, 0x7C, 0x10, 0x6C 		;   �    (166)  (A6)
.db 0x00, 0x44, 0x54, 0x54, 0x28, 0x00 		;   �    (167)  (A7)
.db 0x00, 0x7C, 0x20, 0x10, 0x08, 0x7C 		;   �    (168)  (A8)
.db 0x00, 0x7C, 0x21, 0x12, 0x09, 0x7C 		;   �    (169)  (A9)
.db 0x00, 0x7C, 0x10, 0x28, 0x44, 0x00 		;   �    (170)  (AA)
.db 0x00, 0x70, 0x08, 0x04, 0x7C, 0x00 		;   �    (171)  (AB)
.db 0x00, 0x7C, 0x08, 0x10, 0x08, 0x7C 		;   �    (172)  (AC)
.db 0x00, 0x7C, 0x10, 0x10, 0x10, 0x7C 		;   �    (173)  (AD)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 		;   �    (174)  (AE)
.db 0x00, 0x7C, 0x04, 0x04, 0x04, 0x7C 		;   �    (175)  (AF)
.db 0x00, 0x7C, 0x14, 0x14, 0x14, 0x08 		;   �    (176)  (B0)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 		;   �    (177)  (B1)
.db 0x00, 0x04, 0x04, 0x7C, 0x04, 0x04 		;   �    (178)  (B2)
.db 0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C 		;   �    (179)  (B3)
.db 0x00, 0x30, 0x48, 0xFC, 0x48, 0x30 		;   �    (180)  (B4)
.db 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 		;   �    (181)  (B5)
.db 0x00, 0x7C, 0x40, 0x40, 0x7C, 0xC0 		;   �    (182)  (B6)
.db 0x00, 0x0C, 0x10, 0x10, 0x10, 0x7C 		;   �    (183)  (B7)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0x7C 		;   �    (184)  (B8)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0xFC 		;   �    (185)  (B9)
.db 0x00, 0x04, 0x7C, 0x50, 0x50, 0x20 		;   �    (186)  (BA)
.db 0x00, 0x7C, 0x50, 0x20, 0x00, 0x7C 		;   �    (187)  (BB)
.db 0x00, 0x7C, 0x50, 0x50, 0x20, 0x00 		;   �    (188)  (BC)
.db 0x00, 0x28, 0x44, 0x54, 0x54, 0x38 		;   �    (189)  (BD)
.db 0x00, 0x7C, 0x10, 0x38, 0x44, 0x38 		;   �    (190)  (BE)
.db 0x00, 0x08, 0x54, 0x34, 0x14, 0x7C 		;   �    (191)  (BF)

;------------------------------------------------------------------------------
;	������� ������ ������
;------------------------------------------------------------------------------
.db 0x00, 0xFC, 0xFE, 0x07, 0x03, 0x83		;   0    (192)
.db 0x83, 0x63, 0x67, 0xFE, 0xFC, 0x00
.db 0x00, 0x3F, 0x7F, 0xE6, 0xC6, 0xC1
.db 0xC1, 0xC0, 0xE0, 0x7F, 0x3F, 0x00

.db 0x00, 0x00, 0x00, 0x18, 0x1C, 0xFF		;   1    (193)
.db 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xFF
.db 0xFF, 0xC0, 0xC0, 0x00, 0x00, 0x00

.db 0x00, 0x06, 0x07, 0x03, 0x03, 0x03		;   2    (194)
.db 0x03, 0x83, 0xC7, 0xFE, 0x7C, 0x00
.db 0x00, 0xE0, 0xF0, 0xF8, 0xDC, 0xCE
.db 0xC7, 0xC3, 0xC1, 0xC0, 0xC0, 0x00

.db 0x00, 0x0C, 0x0E, 0x87, 0x83, 0x83		;   3    (195)
.db 0x83, 0x83, 0xC7, 0xFE, 0x3C, 0x00
.db 0x00, 0x30, 0x70, 0xE1, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00

.db 0x00, 0x00, 0x00, 0xC0, 0xF0, 0x3C		;   4    (196)
.db 0x0E, 0x06, 0xFF, 0xFF, 0x00, 0x00
.db 0x00, 0x06, 0x07, 0x07, 0x06, 0x06
.db 0x06, 0x06, 0xFF, 0xFF, 0x06, 0x00

.db 0x00, 0x7E, 0xFF, 0xC3, 0xC3, 0xC3		;   5    (197)
.db 0xC3, 0xC3, 0xC3, 0x83, 0x03, 0x00
.db 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0
.db 0xC0, 0xC0, 0xE1, 0x7F, 0x3F, 0x00

.db 0x00, 0xFE, 0xFF, 0xC3, 0xC3, 0xC3		;   6    (198)
.db 0xC3, 0xC3, 0xC3, 0x87, 0x06, 0x00
.db 0x00, 0x7F, 0xFF, 0xE1, 0xC0, 0xC0
.db 0xC0, 0xC0, 0xE1, 0x7F, 0x3F, 0x00

.db 0x00, 0x03, 0x03, 0x03, 0x03, 0x03		;   7    (199)
.db 0x03, 0xC3, 0xF3, 0x3F, 0x0F, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFC
.db 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00

.db 0x00, 0x7C, 0xFE, 0xC7, 0x83, 0x83		;   8    (200)
.db 0x83, 0x83, 0xC7, 0xFE, 0x7C, 0x00
.db 0x00, 0x3E, 0x7F, 0xE3, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00

.db 0x00, 0x7C, 0xFE, 0xC7, 0x83, 0x83		;   9    (201)
.db 0x83, 0x83, 0xC7, 0xFE, 0xFC, 0x00
.db 0x00, 0x30, 0x70, 0xE1, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE1, 0x7F, 0x3F, 0x00

;------------------------------------------------------------------------------
;	Fonts 24*40
;------------------------------------------------------------------------------
FONT2_TAB:
.db 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF      ; 0
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0xEF, 0xEF, 0xEF, 0xC7, 0x83, 0x01
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x01, 0x83, 0xC7, 0xEF, 0xEF, 0xEF
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F
;
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00      ;1
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F
;
.db 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F      ;2
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0xE0, 0xE0, 0xE0, 0xD0, 0xB8, 0x7C
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7D, 0x3B, 0x17, 0x0F, 0x0F, 0x0F
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00
;
.db 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F      ;3
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F
;
.db 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0      ;4
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x0F, 0x0F, 0x0F, 0x17, 0x3B, 0x7D
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F
;
.db 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF      ;5
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x0F, 0x0F, 0x0F, 0x17, 0x3B, 0x7D
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7C, 0xB8, 0xD0, 0xE0, 0xE0, 0xE0
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F
;
.db 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF      ;6
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xEF, 0xEF, 0xEF, 0xD7, 0xBB, 0x7D
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7C, 0xB8, 0xD0, 0xE0, 0xE0, 0xE0
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F
;
.db 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F      ;7
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F
;
.db 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF      ;8
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0xEF, 0xEF, 0xEF, 0xD7, 0xBB, 0x7D
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F
;
.db 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF      ;9
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
.db 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x0F, 0x0F, 0x0F, 0x17, 0x3B, 0x7D
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE
.db 0x7D, 0xBB, 0xD7, 0xEF, 0xEF, 0xEF
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
.db 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
.db 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F




;******************************************************************************

.exit                                       ; ����� ���������


/*
	; ������� ������� ����� 1 �����
SPEEDOUT_100:
 ldi	temp,50								; ������ ���������� X
 rcall	GLCD_X								; ������������� ���������� X
 ldi	temp,3								; ������ ���������� Y
 rcall	GLCD_Y								; ������������� ���������� Y

 lds	temp,SECOND_H							; ��������� �������� ������� �������
 ldi	temp1,4
 mul	temp,temp1							; ������ ����� ����� � ������� (n=temp*4)
 ldi	temp1,192
 add	mulL,temp1							; ������ �������� �� ������ �������
 mov	temp,mulL                              
 mov	d1,temp								; ��������� � d1 ������� � ������� �������
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
 inc	d1									; ����������� ������� ������� �� �������
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
	; ������� ������ ����� 1 �����
 ldi	temp,50								; ������ ���������� X
 rcall	GLCD_X								; ������������� ���������� X
 ldi	temp,4								; ������ ���������� Y
 rcall	GLCD_Y								; ������������� ���������� Y
 inc	d1
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
 inc	d1									; ����������� ������� ������� �� �������
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����

	; ������� ������� ����� 2 �����
 ldi	temp,65								; ������ ���������� X
 rcall	GLCD_X								; ������������� ���������� X
 ldi	temp,3								; ������ ���������� Y
 rcall	GLCD_Y								; ������������� ���������� Y

 lds	temp,SECOND_L							; ��������� �������� ������� �������
 ldi	temp1,4
 mul	temp,temp1							; ������ ����� ����� � ������� (n=temp*4)
 ldi	temp1,192
 add	mulL,temp1							; ������ �������� �� ������ �������
 mov	temp,mulL                              
 mov	d1,temp								; ��������� � d1 ������� � ������� �������
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
 inc	d1									; ����������� ������� ������� �� �������
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
	; ������� ������ ����� 2 �����
 ldi	temp,65								; ������ ���������� X
 rcall	GLCD_X								; ������������� ���������� X
 ldi	temp,4								; ������ ���������� Y
 rcall	GLCD_Y								; ������������� ���������� Y
 inc	d1
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
 inc	d1									; ����������� ������� ������� �� �������
 mov	temp,d1
 rcall	GLCD_PUT_CHAR						; ������� ������� ������ ����� �����
*/
