; To compile: .\vasm6502_oldstyle.exe -Fbin -dotdir -c02 example.s

PORTB = $c000		; Write direction for port B
DD_PORTB = $c002	; Data direction register for port B	
	
	.org $e000
program_start:	
	ldx #$ff
	txs

	lda #%1111111	; Set bottom 7 pins as output
	sta DD_PORTB	; Initialize Data direction register B

	; Init the LCD Module
	lda #%0010000	; Start configuration for 4 bit mode.
	jsr lcd_instruction

	lda #%0010000	; 4 bit mode, display is 8x5 pixels
	jsr lcd_instruction

	lda #%1000000
	jsr lcd_instruction

	lda #%0000000	; Display ON, cursor ON and blinking cursor OFF
	jsr lcd_instruction

	lda #%1110000
	jsr lcd_instruction

	lda #%0000000	; Increment register by one and shift cursor to the right
	jsr lcd_instruction

	lda #%0110000
	jsr lcd_instruction

	ldy #%00110000	; Number 0 will be loaded

message: .asciiz "Hola Laura!"

	ldx #0
print:
	lda message,x
	beq loop		; If message[x] is 0 (end of asciiz), jump to loop
	jsr send_char
	inx
	jmp print

loop:
	;tya				; Transfer Y to A
	;jsr send_char
	;iny				; Increment Y for the next letter

	jmp loop

send_char:
	; Send the first 4 bits of the letter
	and #$F0		; Fetch the first 4 bits
	ror				; Rotate A one to the right

	jsr lcd_data

	; Send the next 4 bits of the letter
	tya
	and #$0F		; Fetch the last 4 bits
	rol				; Rotate A three to the left
	rol
	rol

	jsr lcd_data
	rts

lcd_data:
	ora #$01		; Add the 1 from RS signal
lcd_instruction:
	jsr lcd_wait
	sta PORTB
	ora #%00000100	; Set ENABLE high
	sta PORTB
	and #%11111011	; Set ENABLE low
	sta PORTB
	rts

lcd_wait:	;  Wait for busy flag = 0
	pha
	txa
	pha
	lda #%0000111	; Set 4 data pins as inputs, but maintain RS,RW,E as outputs
	sta DD_PORTB	; Initialize Data direction register B
lcd_read_flag:
	lda #%0000010	; Fetch the busy flag (ENABLE low)
	sta PORTB
	lda #%0000110	; Set ENABLE high
	sta PORTB
	ldx PORTB		; BF x x x x x x -> X
	lda #%0000010	;  Set ENABLE low
	sta PORTB

	lda #%0000010	; Second mandatory read (because 4 bit mode)
	jsr lcd_instruction

	txa
	and #%1000000
	bne lcd_read_flag ; BF is 1, so AND instruction makes Z flag = 0.

	lda #%1111111	; Set bottom 7 pins as output
	sta DD_PORTB	; Initialize Data direction register B
	
	pla
	tax
	pla
	rts

	.org $fffc			; Entry direction of the W65C02
	.word program_start	; Fetch this direction to the start of the program.
	.word 0