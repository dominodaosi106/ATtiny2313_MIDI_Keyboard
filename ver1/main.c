/*
 * �y�|�[�g�Ή��z ATtiny2313A
 * x: ���݂��Ȃ�, -: ��
 * bit       7     6     5     4     3     2     1     0
 * portA     x     x     x     x     x     x     UP    DOWN
 * portB     B     A#    A     G#    G     F#    F     E
 * portD     x     D#    D     C#    C     MODE  MIDI  CH
 *
 * �yATtiny2313A�s���}�b�s���O�z
 *   (RESET) PA2 |1       20| VCC
 *     (CH) PD0 |2       19| PB7 (B)
 *   (MIDI) PD1 |3       18| PB6 (A#)
 *     (UP) PA1 |4       17| PB5 (A)
 *   (DOWN) PA0 |5       16| PB4 (G#)
 *   (MODE) PD2 |6       15| PB3 (G)
 *      (C) PD3 |7       14| PB2 (F#)
 *     (C#) PD4 |8       13| PB1 (F)
 *      (D) PD5 |9       12| PB0 (E)
 *      GND     |10      11| PD6 (D#)
 * ��MODE, CH (�����v���A�b�v)
 *
 * �y�r���h���z
 * Atmel Studio 7 (Version: 7.0.129)
 * ATtiny2313A ����1MHz�N���b�N
 */



#define F_CPU 1000000UL // ATtiny2313�̃N���b�N���x�A1MHz

#include <avr/io.h>
#include <util/delay.h>

// �s���̏�Ԃ�ǐՂ��邽�߂̃O���[�o���ϐ�
uint8_t prev_state_portd = 0xFF; // PORTD�̑O�̏��
uint8_t prev_state_portb = 0xFF; // PORTB�̑O�̏��
uint8_t midi_channel = 0; // MIDI�`�����l���i0=CH1, ..., 15=CH16�j
uint8_t base_note = 60; // ��m�[�g�ԍ��i����: 60=C4, CH10=36=C2�j
int8_t octave_shift = 0; // �I�N�^�[�u�V�t�g�i�S�`�����l��: -5�`+3�j
uint8_t active_notes[12] = {0}; // �A�N�e�B�u�m�[�g�i0=�I�t, 1=�I���APD3-PD6=0-3, PB0-PB7=4-11�j
int8_t note_octave_shifts[12] = {0}; // �e�m�[�g�̔�������octave_shift�l

void uart_init(void) {
	UBRRH = 0;
	UBRRL = (F_CPU / (16UL * 31250)) - 1; // 31,250bps�iMIDI�W���j
	UCSRB = (1 << TXEN); // ���M����
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0); // 8�r�b�g�f�[�^�t�H�[�}�b�g
}

void uart_transmit(unsigned char data) {
	while (!(UCSRA & (1 << UDRE))); // �f�[�^���W�X�^���󂭂܂ő҂�
	UDR = data; // �f�[�^�𑗐M
}

void midi_note_on(unsigned char channel, unsigned char note, unsigned char velocity) {
	uart_transmit(0x90 | channel);
	uart_transmit(note);
	uart_transmit(velocity);
}

void midi_note_off(unsigned char channel, unsigned char note) {
	uart_transmit(0x80 | channel);
	uart_transmit(note);
	uart_transmit(0);
}

void midi_program_change(uint8_t channel, uint8_t program) {
	uart_transmit(0xC0 | channel);
	uart_transmit(program);
}

void init_settings(void) {
	// PD0��MIDI�`�����l���ݒ�iLOW=CH1, HIGH=CH10�j
	if (!(PIND & (1 << PD0))) {
		midi_channel = 0; // CH1
		base_note = 60; // C4
		} else {
		midi_channel = 9; // CH10
		base_note = 36; // C2
	}
}

void update_octave(void) {
	static uint8_t last_pa1 = 1, last_pa0 = 1; // PA1, PA0�̑O����
	static uint8_t pa1_pressed = 0, pa0_pressed = 0; // �{�^�������t���O
	static uint16_t pa1_timer = 0, pa0_timer = 0; // �������Ԍv��
	static int8_t original_shift = 0; // �����O�̃I�N�^�[�u�V�t�g�l
	static uint8_t function_mode = 0; // �t�@���N�V�����L�[���[�h�t���O
	uint8_t curr_pa1 = PINA & (1 << PA1) ? 1 : 0; // ���݂�PA1���
	uint8_t curr_pa0 = PINA & (1 << PA0) ? 1 : 0; // ���݂�PA0���
	uint8_t curr_state_portd = PIND;
	uint8_t curr_state_portb = PINB;

	// �`�����l��10����PD2=HIGH�̏ꍇ�A�I�N�^�[�u�V�t�g����
	if (midi_channel == 9 && (PIND & (1 << PD2))) {
		if (octave_shift != 0) {
			octave_shift = 0; // �V�t�g�����Z�b�g
		}
		last_pa1 = curr_pa1;
		last_pa0 = curr_pa0;
		pa1_pressed = 0;
		pa0_pressed = 0;
		pa1_timer = 0;
		pa0_timer = 0;
		function_mode = 0;
		return; // PA1/PA0�̓��͂𖳎�
	}

	// �t�@���N�V�����L�[�iPA1+PA0���������j�̌��o
	if (!curr_pa1 && !curr_pa0 && (last_pa1 || last_pa0)) { // ���������ꂽ
		function_mode = 1; // �t�@���N�V�����L�[���[�hON
		pa1_pressed = 0;
		pa0_pressed = 0;
		pa1_timer = 0;
		pa0_timer = 0;
		_delay_ms(10); // �f�o�E���X
		} else if (function_mode && (curr_pa1 || curr_pa0)) { // �ǂ��炩�������ꂽ
		function_mode = 0; // �t�@���N�V�����L�[���[�hOFF
		_delay_ms(10); // �f�o�E���X
	}

	if (function_mode) {
		// �t�@���N�V�����L�[+C#�iPD4�j�FMIDI CH1
		if (!(curr_state_portd & (1 << PD4)) && (prev_state_portd & (1 << PD4))) {
			midi_channel = 0;
			base_note = 60; // C4�i�����I�ɐݒ�j
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+D#�iPD6�j�FMIDI CH10
		if (!(curr_state_portd & (1 << PD6)) && (prev_state_portd & (1 << PD6))) {
			midi_channel = 9;
			// base_note�͕ێ�
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+F#�iPB2�j�FMIDI CH-
		if (!(curr_state_portb & (1 << PB2)) && (prev_state_portb & (1 << PB2))) {
			if (midi_channel > 0) {
				midi_channel--; // 1�`16�i0�`15�j
				// base_note�͕ێ�
			}
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+A#�iPB6�j�FMIDI CH+
		if (!(curr_state_portb & (1 << PB6)) && (prev_state_portb & (1 << PB6))) {
			if (midi_channel < 15) {
				midi_channel++; // 1�`16�i0�`15�j
				// base_note�͕ێ�
			}
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+C�iPD3�j�F�v���O�����`�F���W1�i�s�A�m�j
		if (!(curr_state_portd & (1 << PD3)) && (prev_state_portd & (1 << PD3))) {
			midi_program_change(midi_channel, 0); // �s�A�m
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+D�iPD5�j�F�v���O�����`�F���W10�i�I���S�[���j
		if (!(curr_state_portd & (1 << PD5)) && (prev_state_portd & (1 << PD5))) {
			midi_program_change(midi_channel, 10); // �I���S�[��
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+A�iPB5�j�F�v���O�����`�F���W88�i�x�[�X+���[�h�j
		if (!(curr_state_portb & (1 << PB5)) && (prev_state_portb & (1 << PB5))) {
			midi_program_change(midi_channel, 87); // �x�[�X+���[�h
			_delay_ms(10);
		}
		// �t�@���N�V�����L�[+B�iPB7�j�F�v���O�����`�F���W81�i��`�g�j
		if (!(curr_state_portb & (1 << PB7)) && (prev_state_portb & (1 << PB7))) {
			midi_program_change(midi_channel, 80); // ��`�g
			_delay_ms(10);
		}
		last_pa1 = curr_pa1;
		last_pa0 = curr_pa0;
		return; // �t�@���N�V�����L�[���[�h���͒ʏ�̃V�t�g�������X�L�b�v
	}

	// PA1�i�I�N�^�[�u�A�b�v�j�̏���
	if (!curr_pa1 && last_pa1) { // PA1�������ꂽ�i����������j
		pa1_pressed = 1;
		pa1_timer = 0; // �^�C�}�[���Z�b�g
		original_shift = octave_shift; // ���݂̃V�t�g�l��ۑ�
		if (octave_shift < 3) { // �S�`�����l����-5�`+3
			octave_shift++; // �����V�t�g�A�b�v
		}
		} else if (curr_pa1 && !last_pa1 && pa1_pressed) { // PA1�������ꂽ
		if (pa1_timer >= 200) { // �������i200ms�ȏ�j
			octave_shift = original_shift; // ���̃V�t�g�l�ɖ߂�
		} // �Z�����i200ms�����j�̏ꍇ�̓V�t�g���ێ�
		pa1_pressed = 0; // �����t���O�N���A
		} else if (pa1_pressed) { // PA1�������ꑱ���Ă���
		pa1_timer += 10; // 10ms���ƂɃ^�C�}�[����
	}

	// PA0�i�I�N�^�[�u�_�E���j�̏���
	if (!curr_pa0 && last_pa0) { // PA0�������ꂽ�i����������j
		pa0_pressed = 1;
		pa0_timer = 0; // �^�C�}�[���Z�b�g
		original_shift = octave_shift; // ���݂̃V�t�g�l��ۑ�
		if (octave_shift > -5) { // �S�`�����l����-5�`+3
			octave_shift--; // �����V�t�g�_�E��
		}
		} else if (curr_pa0 && !last_pa0 && pa0_pressed) { // PA0�������ꂽ
		if (pa0_timer >= 200) { // �������i200ms�ȏ�j
			octave_shift = original_shift; // ���̃V�t�g�l�ɖ߂�
		} // �Z�����i200ms�����j�̏ꍇ�̓V�t�g���ێ�
		pa0_pressed = 0; // �����t���O�N���A
		} else if (pa0_pressed) { // PA0�������ꑱ���Ă���
		pa0_timer += 10; // 10ms���ƂɃ^�C�}�[����
	}

	last_pa1 = curr_pa1;
	last_pa0 = curr_pa0;
}

void check_pin_changes(void) {
	uint8_t curr_state_portd = PIND;
	uint8_t curr_state_portb = PINB;

	// PORTD�iPD3-PD6�j
	for (uint8_t i = 3; i <= 6; i++) {
		uint8_t mask = (1 << i);
		uint8_t index = i - 3; // active_notes�̃C���f�b�N�X
		int16_t note = base_note + (i - 3) + (octave_shift * 12); // ���݂̃m�[�g�v�Z
		int16_t note_at_press = base_note + (i - 3) + (note_octave_shifts[index] * 12); // ���������_�̃m�[�g
		if (note >= 0 && note <= 127) { // MIDI�m�[�g�͈̓`�F�b�N
			if (!(curr_state_portd & mask) && (prev_state_portd & mask)) {
				midi_note_on(midi_channel, note, 127);
				active_notes[index] = 1; // �m�[�g�I����Ԃ��L�^
				note_octave_shifts[index] = octave_shift; // ��������octave_shift���L�^
				} else if ((curr_state_portd & mask) && !(prev_state_portd & mask) && active_notes[index]) {
				midi_note_off(midi_channel, note_at_press); // ���������_�̃m�[�g���I�t
				active_notes[index] = 0; // �m�[�g�I�t��Ԃ��N���A
			}
		}
	}

	// PORTB�iPB0-PB7�j
	for (uint8_t i = 0; i <= 7; i++) {
		uint8_t mask = (1 << i);
		uint8_t index = 4 + i; // active_notes�̃C���f�b�N�X
		int16_t note = base_note + 4 + i + (octave_shift * 12); // ���݂̃m�[�g�v�Z
		int16_t note_at_press = base_note + 4 + i + (note_octave_shifts[index] * 12); // ���������_�̃m�[�g
		if (note >= 0 && note <= 127) {
			if (!(curr_state_portb & mask) && (prev_state_portb & mask)) {
				midi_note_on(midi_channel, note, 127);
				active_notes[index] = 1; // �m�[�g�I����Ԃ��L�^
				note_octave_shifts[index] = octave_shift; // ��������octave_shift���L�^
				} else if ((curr_state_portb & mask) && !(prev_state_portb & mask) && active_notes[index]) {
				midi_note_off(midi_channel, note_at_press); // ���������_�̃m�[�g���I�t
				active_notes[index] = 0; // �m�[�g�I�t��Ԃ��N���A
			}
		}
	}

	prev_state_portd = curr_state_portd;
	prev_state_portb = curr_state_portb;
}

int main(void) {
	// �s������͂ɐݒ�
	DDRA &= ~((1 << PA0) | (1 << PA1)); // PA0, PA1�����
	DDRD &= ~((1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6)); // PD0, PD2-PD6
	DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) |
	(1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7)); // PB0-PB7

	// �v���A�b�v��R��L����
	PORTA |= ((1 << PA0) | (1 << PA1));
	PORTD |= ((1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6));
	PORTB |= ((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) |
	(1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));

	uart_init();
	init_settings(); // �N������PD0��ǂݍ���

	while (1) {
		update_octave(); // �I�N�^�[�u�V�t�g����уt�@���N�V�����L�[����
		check_pin_changes(); // �s���̏�ԃ`�F�b�N
		_delay_ms(10); // �f�o�E���X
	}
}