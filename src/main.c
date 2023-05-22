/*
 * This program is a driver for using a 1602A LCD with Arduino/Atmega328P in 4-bit mode.
 *
 * Author: Weilong Mao
 * Date: 2023/05/22
 */
#include <avr/io.h>
#include <util/delay.h>

#define RS_PIN PB4
#define RW_PIN PB3
#define E_PIN  PB2

// 4-bit mode, DB4-DB7
#define D4_PIN PD7
#define D5_PIN PD6
#define D6_PIN PD5
#define D7_PIN PD4

// Minimum is 10us, but 12us is stable.
#define DELAY_US 12

const short DB[] = {D7_PIN, D6_PIN, D5_PIN, D4_PIN};

unsigned char str1[] = "Hello, World!";
unsigned char str2[] = "--Weilong Mao";
unsigned char str3[] = "To Explore";
unsigned char str4[] = "and dream!";

void lcd_en_write() {
    _delay_us(DELAY_US);
    // Set E pin to HIGH
    PORTB |= (1 << E_PIN);
    _delay_us(DELAY_US);
    // Set E pin to LOW
    PORTB &= ~(1 << E_PIN);
    _delay_us(DELAY_US);
}

void lcd_command_write(uint8_t command) {
    uint8_t i, temp;
    // Set RS, RW , E pins to LOW
    PORTB &= ~((1 << RS_PIN) | (1 << RW_PIN) | (1 << E_PIN));

    // Send command for DB4-DB7
    temp = command & 0xF0;
    for (i = 0; i < 4; i++) {
        if (temp & 0x80) {
            PORTD |= (1 << DB[i]);
        } else {
            PORTD &= ~(1 << DB[i]);
        }
        temp <<= 1;
    }
    lcd_en_write();

    // Send command for DB0-DB3
    temp = (command & 0x0F) << 4;
    for (i = 0; i < 4; i++) {
        if (temp & 0x80) {
            PORTD |= (1 << DB[i]);
        } else {
            PORTD &= ~(1 << DB[i]);
        }
        temp <<= 1;
    }
    lcd_en_write();
}

void lcd_data_write(uint8_t data) {
    uint8_t i, temp;
    // Set RW, E pins to LOW
    PORTB &= ~((1 << RW_PIN) | (1 << E_PIN));
    // Set RS pin to HIGH
    PORTB |= (1 << RS_PIN);

    // Send data for DB4-DB7
    temp = data & 0xF0;
    for (i = 0; i < 4; i++) {
        if (temp & 0x80) {
            PORTD |= (1 << DB[i]);
        } else {
            PORTD &= ~(1 << DB[i]);
        }
        temp <<= 1;
    }
    lcd_en_write();

    // Send data for DB0-DB3
    temp = (data & 0x0F) << 4;
    for (i = 0; i < 4; i++) {
        if (temp & 0x80) {
            PORTD |= (1 << DB[i]);
        } else {
            PORTD &= ~(1 << DB[i]);
        }
        temp <<= 1;
    }
    lcd_en_write();
}

void lcd_set_xy(uint8_t x, uint8_t y) {
    uint8_t address;
    if (y == 0) {
        address = 0x80 + x;
    } else {
        address = 0xC0 + x;
    }
    lcd_command_write(address);
}

void lcd_write_char(uint8_t x, uint8_t y, unsigned char c) {
    lcd_set_xy(x, y);
    lcd_data_write(c);
}

void lcd_write_string(uint8_t x, uint8_t y, unsigned char *str) {
    lcd_set_xy(x, y);
    while (*str) {
        lcd_data_write(*str++);
    }
}

void lcd_clear() {
    lcd_command_write(0x01);
    _delay_ms(2);
}

void setup() {
    // Set all pins as output
    DDRB |= (1 << RS_PIN) | (1 << RW_PIN) | (1 << E_PIN);
    DDRD |= (1 << D4_PIN) | (1 << D5_PIN) | (1 << D6_PIN) | (1 << D7_PIN);
    // Set all pins to LOW
    PORTB &= ~((1 << RS_PIN) | (1 << RW_PIN) | (1 << E_PIN));
    PORTD &= ~((1 << D4_PIN) | (1 << D5_PIN) | (1 << D6_PIN) | (1 << D7_PIN));

    _delay_ms(150);
    lcd_command_write(0x28);  // 4-bit mode, 2-line, 5x8 font
    _delay_ms(15);
    lcd_command_write(0x28);
    _delay_ms(15);
    lcd_command_write(0x28);
    _delay_ms(15);
    /*
     * After reset, the 1602 LCD defaults to 8-bit input mode. lcd_write_command() is designed to write commands using
     * 4-bit mode.When sending 0x28, it will send 0x02 first then 0x08. After receiving 0x02, the LCD will be in 4-bit
     * mode. It will always wait for the second nibble, so we need to send 0x28 three times to make sure it is in 4-bit
     * mode. And use lcd_en_write() to skip the nibble. Nor we will meet the gibberish issue.
     */
    lcd_en_write();
    _delay_ms(15);
    lcd_command_write(0x28);
    _delay_ms(15);
    lcd_command_write(0x0C);  // Display on, no cursor, no blink
    _delay_ms(5);
    lcd_command_write(0x06);  // Entry mode: increment, no shift
    _delay_ms(5);
    lcd_command_write(0x01);  // Clear display
    _delay_ms(5);
    lcd_command_write(0x02);  // Return home
    _delay_ms(5);
    lcd_command_write(0x80);  // Set DDRAM address to 0x00
    _delay_ms(5);

}

int main() {
    // Initialize
    setup();
    while (1) {
        lcd_clear();
        lcd_write_string(0, 0, str1);
        lcd_write_string(3, 1, str2);
        _delay_ms(2000);
        lcd_clear();
        lcd_write_string(0, 0, str3);
        lcd_write_string(6, 1, str4);
        _delay_ms(2000);
    }
}
