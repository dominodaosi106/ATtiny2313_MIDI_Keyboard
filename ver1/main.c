/*
 * 【ポート対応】 ATtiny2313A
 * x: 存在しない, -: 空き
 * bit       7     6     5     4     3     2     1     0
 * portA     x     x     x     x     x     x     UP    DOWN
 * portB     B     A#    A     G#    G     F#    F     E
 * portD     x     D#    D     C#    C     MODE  MIDI  CH
 *
 * 【ATtiny2313Aピンマッピング】
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
 * ※MODE, CH (内部プルアップ)
 *
 * 【ビルド環境】
 * Atmel Studio 7 (Version: 7.0.129)
 * ATtiny2313A 内蔵1MHzクロック
 */



#define F_CPU 1000000UL // ATtiny2313のクロック速度、1MHz

#include <avr/io.h>
#include <util/delay.h>

// ピンの状態を追跡するためのグローバル変数
uint8_t prev_state_portd = 0xFF; // PORTDの前の状態
uint8_t prev_state_portb = 0xFF; // PORTBの前の状態
uint8_t midi_channel = 0; // MIDIチャンネル（0=CH1, ..., 15=CH16）
uint8_t base_note = 60; // 基準ノート番号（初期: 60=C4, CH10=36=C2）
int8_t octave_shift = 0; // オクターブシフト（全チャンネル: -5～+3）
uint8_t active_notes[12] = {0}; // アクティブノート（0=オフ, 1=オン、PD3-PD6=0-3, PB0-PB7=4-11）
int8_t note_octave_shifts[12] = {0}; // 各ノートの発音時のoctave_shift値

void uart_init(void) {
	UBRRH = 0;
	UBRRL = (F_CPU / (16UL * 31250)) - 1; // 31,250bps（MIDI標準）
	UCSRB = (1 << TXEN); // 送信許可
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0); // 8ビットデータフォーマット
}

void uart_transmit(unsigned char data) {
	while (!(UCSRA & (1 << UDRE))); // データレジスタが空くまで待つ
	UDR = data; // データを送信
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
	// PD0でMIDIチャンネル設定（LOW=CH1, HIGH=CH10）
	if (!(PIND & (1 << PD0))) {
		midi_channel = 0; // CH1
		base_note = 60; // C4
		} else {
		midi_channel = 9; // CH10
		base_note = 36; // C2
	}
}

void update_octave(void) {
	static uint8_t last_pa1 = 1, last_pa0 = 1; // PA1, PA0の前回状態
	static uint8_t pa1_pressed = 0, pa0_pressed = 0; // ボタン押下フラグ
	static uint16_t pa1_timer = 0, pa0_timer = 0; // 押下時間計測
	static int8_t original_shift = 0; // 押す前のオクターブシフト値
	static uint8_t function_mode = 0; // ファンクションキーモードフラグ
	uint8_t curr_pa1 = PINA & (1 << PA1) ? 1 : 0; // 現在のPA1状態
	uint8_t curr_pa0 = PINA & (1 << PA0) ? 1 : 0; // 現在のPA0状態
	uint8_t curr_state_portd = PIND;
	uint8_t curr_state_portb = PINB;

	// チャンネル10かつPD2=HIGHの場合、オクターブシフト無効
	if (midi_channel == 9 && (PIND & (1 << PD2))) {
		if (octave_shift != 0) {
			octave_shift = 0; // シフトをリセット
		}
		last_pa1 = curr_pa1;
		last_pa0 = curr_pa0;
		pa1_pressed = 0;
		pa0_pressed = 0;
		pa1_timer = 0;
		pa0_timer = 0;
		function_mode = 0;
		return; // PA1/PA0の入力を無視
	}

	// ファンクションキー（PA1+PA0同時押し）の検出
	if (!curr_pa1 && !curr_pa0 && (last_pa1 || last_pa0)) { // 両方押された
		function_mode = 1; // ファンクションキーモードON
		pa1_pressed = 0;
		pa0_pressed = 0;
		pa1_timer = 0;
		pa0_timer = 0;
		_delay_ms(10); // デバウンス
		} else if (function_mode && (curr_pa1 || curr_pa0)) { // どちらかが離された
		function_mode = 0; // ファンクションキーモードOFF
		_delay_ms(10); // デバウンス
	}

	if (function_mode) {
		// ファンクションキー+C#（PD4）：MIDI CH1
		if (!(curr_state_portd & (1 << PD4)) && (prev_state_portd & (1 << PD4))) {
			midi_channel = 0;
			base_note = 60; // C4（明示的に設定）
			_delay_ms(10);
		}
		// ファンクションキー+D#（PD6）：MIDI CH10
		if (!(curr_state_portd & (1 << PD6)) && (prev_state_portd & (1 << PD6))) {
			midi_channel = 9;
			// base_noteは保持
			_delay_ms(10);
		}
		// ファンクションキー+F#（PB2）：MIDI CH-
		if (!(curr_state_portb & (1 << PB2)) && (prev_state_portb & (1 << PB2))) {
			if (midi_channel > 0) {
				midi_channel--; // 1～16（0～15）
				// base_noteは保持
			}
			_delay_ms(10);
		}
		// ファンクションキー+A#（PB6）：MIDI CH+
		if (!(curr_state_portb & (1 << PB6)) && (prev_state_portb & (1 << PB6))) {
			if (midi_channel < 15) {
				midi_channel++; // 1～16（0～15）
				// base_noteは保持
			}
			_delay_ms(10);
		}
		// ファンクションキー+C（PD3）：プログラムチェンジ1（ピアノ）
		if (!(curr_state_portd & (1 << PD3)) && (prev_state_portd & (1 << PD3))) {
			midi_program_change(midi_channel, 0); // ピアノ
			_delay_ms(10);
		}
		// ファンクションキー+D（PD5）：プログラムチェンジ10（オルゴール）
		if (!(curr_state_portd & (1 << PD5)) && (prev_state_portd & (1 << PD5))) {
			midi_program_change(midi_channel, 10); // オルゴール
			_delay_ms(10);
		}
		// ファンクションキー+A（PB5）：プログラムチェンジ88（ベース+リード）
		if (!(curr_state_portb & (1 << PB5)) && (prev_state_portb & (1 << PB5))) {
			midi_program_change(midi_channel, 87); // ベース+リード
			_delay_ms(10);
		}
		// ファンクションキー+B（PB7）：プログラムチェンジ81（矩形波）
		if (!(curr_state_portb & (1 << PB7)) && (prev_state_portb & (1 << PB7))) {
			midi_program_change(midi_channel, 80); // 矩形波
			_delay_ms(10);
		}
		last_pa1 = curr_pa1;
		last_pa0 = curr_pa0;
		return; // ファンクションキーモード中は通常のシフト処理をスキップ
	}

	// PA1（オクターブアップ）の処理
	if (!curr_pa1 && last_pa1) { // PA1が押された（立ち下がり）
		pa1_pressed = 1;
		pa1_timer = 0; // タイマーリセット
		original_shift = octave_shift; // 現在のシフト値を保存
		if (octave_shift < 3) { // 全チャンネルで-5～+3
			octave_shift++; // 即時シフトアップ
		}
		} else if (curr_pa1 && !last_pa1 && pa1_pressed) { // PA1が離された
		if (pa1_timer >= 200) { // 長押し（200ms以上）
			octave_shift = original_shift; // 元のシフト値に戻す
		} // 短押し（200ms未満）の場合はシフトを維持
		pa1_pressed = 0; // 押下フラグクリア
		} else if (pa1_pressed) { // PA1が押され続けている
		pa1_timer += 10; // 10msごとにタイマー増加
	}

	// PA0（オクターブダウン）の処理
	if (!curr_pa0 && last_pa0) { // PA0が押された（立ち下がり）
		pa0_pressed = 1;
		pa0_timer = 0; // タイマーリセット
		original_shift = octave_shift; // 現在のシフト値を保存
		if (octave_shift > -5) { // 全チャンネルで-5～+3
			octave_shift--; // 即時シフトダウン
		}
		} else if (curr_pa0 && !last_pa0 && pa0_pressed) { // PA0が離された
		if (pa0_timer >= 200) { // 長押し（200ms以上）
			octave_shift = original_shift; // 元のシフト値に戻す
		} // 短押し（200ms未満）の場合はシフトを維持
		pa0_pressed = 0; // 押下フラグクリア
		} else if (pa0_pressed) { // PA0が押され続けている
		pa0_timer += 10; // 10msごとにタイマー増加
	}

	last_pa1 = curr_pa1;
	last_pa0 = curr_pa0;
}

void check_pin_changes(void) {
	uint8_t curr_state_portd = PIND;
	uint8_t curr_state_portb = PINB;

	// PORTD（PD3-PD6）
	for (uint8_t i = 3; i <= 6; i++) {
		uint8_t mask = (1 << i);
		uint8_t index = i - 3; // active_notesのインデックス
		int16_t note = base_note + (i - 3) + (octave_shift * 12); // 現在のノート計算
		int16_t note_at_press = base_note + (i - 3) + (note_octave_shifts[index] * 12); // 押した時点のノート
		if (note >= 0 && note <= 127) { // MIDIノート範囲チェック
			if (!(curr_state_portd & mask) && (prev_state_portd & mask)) {
				midi_note_on(midi_channel, note, 127);
				active_notes[index] = 1; // ノートオン状態を記録
				note_octave_shifts[index] = octave_shift; // 発音時のoctave_shiftを記録
				} else if ((curr_state_portd & mask) && !(prev_state_portd & mask) && active_notes[index]) {
				midi_note_off(midi_channel, note_at_press); // 押した時点のノートをオフ
				active_notes[index] = 0; // ノートオフ状態をクリア
			}
		}
	}

	// PORTB（PB0-PB7）
	for (uint8_t i = 0; i <= 7; i++) {
		uint8_t mask = (1 << i);
		uint8_t index = 4 + i; // active_notesのインデックス
		int16_t note = base_note + 4 + i + (octave_shift * 12); // 現在のノート計算
		int16_t note_at_press = base_note + 4 + i + (note_octave_shifts[index] * 12); // 押した時点のノート
		if (note >= 0 && note <= 127) {
			if (!(curr_state_portb & mask) && (prev_state_portb & mask)) {
				midi_note_on(midi_channel, note, 127);
				active_notes[index] = 1; // ノートオン状態を記録
				note_octave_shifts[index] = octave_shift; // 発音時のoctave_shiftを記録
				} else if ((curr_state_portb & mask) && !(prev_state_portb & mask) && active_notes[index]) {
				midi_note_off(midi_channel, note_at_press); // 押した時点のノートをオフ
				active_notes[index] = 0; // ノートオフ状態をクリア
			}
		}
	}

	prev_state_portd = curr_state_portd;
	prev_state_portb = curr_state_portb;
}

int main(void) {
	// ピンを入力に設定
	DDRA &= ~((1 << PA0) | (1 << PA1)); // PA0, PA1を入力
	DDRD &= ~((1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6)); // PD0, PD2-PD6
	DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) |
	(1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7)); // PB0-PB7

	// プルアップ抵抗を有効化
	PORTA |= ((1 << PA0) | (1 << PA1));
	PORTD |= ((1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6));
	PORTB |= ((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) |
	(1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));

	uart_init();
	init_settings(); // 起動時にPD0を読み込み

	while (1) {
		update_octave(); // オクターブシフトおよびファンクションキー処理
		check_pin_changes(); // ピンの状態チェック
		_delay_ms(10); // デバウンス
	}
}