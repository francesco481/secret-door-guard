/*
 * Smart Lock - AVR-C pentru ATmega2560
 * Versiune finala cu PWM hardware pentru servo
 *
 * MODIFICARE HARDWARE NECESARA:
 *   Servo mutat de pe D10 (PB4) pe D12 (PB6 = OC1B)
 *   Relay ramane pe D11 (PB5) - GPIO pur, neatins de timer
 *
 * Motivatie:
 *   PWM software via ISR pe D10 are race condition intre COMPA/COMPB
 *   si resetul manual TCNT1=0, ceea ce blocheaza pinul in stare nedefinita.
 *   Timer1 FastPWM (mode 14, ICR1=TOP) genereaza semnal 50Hz hardware
 *   pe OC1B (PB6/D12) fara nicio interventie software in bucla principala.
 *
 * Mapare pini fizici ATmega2560:
 *  - Servo       : PB6  (Arduino D12, OC1B) -> PWM hardware Timer1
 *  - Relay/Bec   : PB5  (Arduino D11)       -> GPIO simplu
 *  - Trig HC-SR04: PL3  (Arduino D46)
 *  - Echo HC-SR04: PL2  (Arduino D47)
 *  - LCD I2C     : SDA=PD1 (D20), SCL=PD0 (D21)
 *  - Keypad ROWS : PC7..PC4 (D30..D33)
 *  - Keypad COLS : PC3..PC0 (D34..D37)
 *
 * Timere:
 *  - Timer0: millis() via OVF ISR (prescaler 64)
 *  - Timer1: FastPWM mode 14 (ICR1=TOP), prescaler 8, 50Hz pe OC1B (PB6)
 *            COM1A=00 -> PB5 (relay) neatins de timer
 *            COM1B=10 -> OC1B non-inverting (PB6 = semnal servo)
 *  - Timer3: masurare distanta HC-SR04 (pornit/oprit per masurare)
 *
 * PWM servo Timer1 FastPWM mode 14:
 *   TOP = ICR1 = 40000 (prescaler 8, 16MHz -> 0.5us/tick -> 20ms perioada)
 *   OCR1B = 2000 -> 1ms  impuls -> 0   grade
 *   OCR1B = 5000 -> 2.5ms impuls -> 185 grade
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* =========================================================
 * CONSTANTE
 * ========================================================= */
#define F_CPU_HZ  16000000UL
#define BAUD      9600
#define UBRR_VAL  ((F_CPU_HZ / (16UL * BAUD)) - 1)

#define LCD_ADDR  0x27
#define LCD_RS    (1 << 0)
#define LCD_RW    (1 << 1)
#define LCD_EN    (1 << 2)
#define LCD_BL    (1 << 3)

/* =========================================================
 * PINI FIZICI
 * ========================================================= */
/* Servo: D12 = PB6 = OC1B. PWM hardware Timer1 FastPWM mode 14. */
#define SERVO_DDR   DDRB
#define SERVO_PORT  PORTB
#define SERVO_BIT   PB6

/* Relay/Bec: D11 = PB5. GPIO pur.
 * COM1A=00 in TCCR1A => OC1A (PB5) neatins de Timer1. */
#define RELAY_DDR   DDRB
#define RELAY_PORT  PORTB
#define RELAY_BIT   PB5

/* HC-SR04 pe PORTL */
#define TRIG_DDR   DDRL
#define TRIG_PORT  PORTL
#define TRIG_BIT   PL3
#define ECHO_DDR   DDRL
#define ECHO_PINR  PINL
#define ECHO_BIT   PL2

/* Keypad pe PORTC */
#define KP_DDR    DDRC
#define KP_PORT   PORTC
#define KP_PINR   PINC

/* =========================================================
 * VARIABILE GLOBALE
 * ========================================================= */
static char    input_pin[5]  = "";
static char    master_pin[5] = "";
static uint8_t mod_schimbare = 0;

/* =========================================================
 * UART
 * ========================================================= */
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

/* =========================================================
 * TWI (I2C) hardware
 * ========================================================= */
#define TWI_FREQ  100000UL
#define TWBR_VAL  ((F_CPU_HZ / TWI_FREQ - 16) / 2)

static void twi_init(void)
{
    TWSR = 0x00;
    TWBR = (uint8_t)TWBR_VAL;
    TWCR = (1 << TWEN);
}

static void twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

static void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static void twi_send_byte(uint8_t addr, uint8_t data)
{
    twi_start();
    twi_write(addr << 1);
    twi_write(data);
    twi_stop();
}

/* =========================================================
 * LCD via PCF8574 (I2C, 4-bit)
 * ========================================================= */
static void lcd_pulse_en(uint8_t data)
{
    twi_send_byte(LCD_ADDR, data | LCD_EN);
    _delay_us(1);
    twi_send_byte(LCD_ADDR, data & ~LCD_EN);
    _delay_us(50);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t d = LCD_BL | ((nibble & 0x0F) << 4);
    if (rs) d |= LCD_RS;
    lcd_pulse_en(d);
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble(byte >> 4,   rs);
    lcd_send_nibble(byte & 0x0F, rs);
}

static void lcd_cmd(uint8_t cmd)    { lcd_send_byte(cmd,  0); }
static void lcd_data(uint8_t data)  { lcd_send_byte(data, 1); }

static void lcd_init(void)
{
    _delay_ms(50);
    lcd_send_nibble(0x03, 0); _delay_ms(5);
    lcd_send_nibble(0x03, 0); _delay_us(150);
    lcd_send_nibble(0x03, 0); _delay_us(150);
    lcd_send_nibble(0x02, 0);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    _delay_ms(2);
}

static void lcd_clear(void) { lcd_cmd(0x01); _delay_ms(2); }

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t off[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + off[row & 1]));
}

static void lcd_print(const char *s) { while (*s) lcd_data((uint8_t)*s++); }
static void lcd_print_char(char c)   { lcd_data((uint8_t)c); }

/* =========================================================
 * SERVO - PWM hardware pe PB6 (D12, OC1B) via Timer1
 *
 * Timer1 FastPWM mode 14 (WGM13:10 = 1110):
 *   TOP = ICR1 = 40000
 *   Prescaler 8 -> tick = 0.5us -> perioada = 40000 * 0.5us = 20ms (50Hz)
 *
 * COM1B = 10 (non-inverting):
 *   OC1B (PB6) = HIGH la BOTTOM, LOW la OCR1B match
 *   => impuls pozitiv cu latimea OCR1B * 0.5us
 *
 * COM1A = 00:
 *   OC1A (PB5) ignorat de timer -> relay pe PB5 functioneaza ca GPIO pur
 *
 * Conversie unghi -> OCR1B:
 *   0   grade -> OCR1B = 2000 (1000us = 1ms)
 *   185 grade -> OCR1B = 5000 (2500us = 2.5ms)
 *   Formula: OCR1B = 2000 + angle * 3000 / 185
 *
 * Nu sunt necesare ISR-uri - semnalul PWM e generat 100% hardware.
 * ========================================================= */
static void servo_init(void)
{
    // 1. Setăm pinul ca ieșire și ÎN ACELAȘI TIMP îl forțăm în LOW
    // pentru a evita semnale parazite până pornește PWM-ul
    SERVO_PORT &= ~(1 << SERVO_BIT);
    SERVO_DDR  |= (1 << SERVO_BIT);

    TCCR1A = (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1  = 40000;   // 20ms perioadă
    
    // IMPORTANTE: Această valoare trebuie să fie fix 1000 pentru 0 grade
    OCR1B = 1000;    
}

static void servo_write(uint16_t angle)
{
    if (angle > 180) angle = 180;

    // Formula precisă pentru intervalul 1000-5000
    // (5000 - 1000) / 180 = 4000 / 180
    uint32_t ocr = 1000UL + ((uint32_t)angle * 4000UL) / 180UL;
    
    OCR1B = (uint16_t)ocr;
}

/* =========================================================
 * RELAY
 * ========================================================= */
static void relay_init(void)
{
    RELAY_DDR  |=  (1 << RELAY_BIT);
    RELAY_PORT |=  (1 << RELAY_BIT);   /* HIGH = bec stins initial */
}

static void relay_on(void)  { RELAY_PORT &= ~(1 << RELAY_BIT); }   /* LOW  = aprins */
static void relay_off(void) { RELAY_PORT |=  (1 << RELAY_BIT); }   /* HIGH = stins  */

/* =========================================================
 * HC-SR04 - masurare cu Timer3 hardware
 *
 * Timer3: prescaler 8 -> 0.5us/tick, independent de Timer1.
 * Pornit la inceputul masurarii, oprit la sfarsit.
 *
 * Formula: distanta [cm] = ticks * 17 / 2000
 *   (echivalent: ticks * 0.5us * 0.034cm/us / 2)
 *
 * Timeout: 60000 ticks = 30ms (identic cu pulseIn(..., 30000) Arduino)
 * ========================================================= */
static void ultrasonic_init(void)
{
    TRIG_DDR  |=  (1 << TRIG_BIT);
    ECHO_DDR  &= ~(1 << ECHO_BIT);
    TRIG_PORT &= ~(1 << TRIG_BIT);
}

static uint16_t citeste_distanta(void)
{
    /* Timer3: Normal mode, prescaler 8 -> 0.5us/tick */
    TCCR3A = 0x00;
    TCCR3B = (1 << CS31);
    TCNT3  = 0;

    /* Trigger 10us */
    TRIG_PORT |=  (1 << TRIG_BIT);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_BIT);

    /* Asteapta flanc RISING pe Echo */
    uint32_t to = 480000UL;
    while (!(ECHO_PINR & (1 << ECHO_BIT))) {
        if (!--to) { TCCR3B = 0x00; return 999; }
    }

    /* Masoara durata pulsului Echo */
    TCNT3 = 0;
    while (ECHO_PINR & (1 << ECHO_BIT)) {
        if (TCNT3 > 60000U) { TCCR3B = 0x00; return 999; }
    }

    uint16_t ticks = TCNT3;
    TCCR3B = 0x00;

    return (uint16_t)((uint32_t)ticks * 17UL / 2000UL);
}

/* =========================================================
 * KEYPAD 4x4
 *
 * ROWS: biti 7..4 pe PORTC (D30..D33) -> iesiri
 * COLS: biti 3..0 pe PORTC (D34..D37) -> intrari cu pull-up
 * ========================================================= */
static const char key_map[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static void keypad_init(void)
{
    KP_DDR  |=  (1<<7)|(1<<6)|(1<<5)|(1<<4);
    KP_PORT |=  (1<<7)|(1<<6)|(1<<5)|(1<<4);
    KP_DDR  &= ~((1<<3)|(1<<2)|(1<<1)|(1<<0));
    KP_PORT |=   (1<<3)|(1<<2)|(1<<1)|(1<<0);
}

static char keypad_get_key(void)
{
    for (uint8_t r = 0; r < 4; r++) {
        KP_PORT |=  (1<<7)|(1<<6)|(1<<5)|(1<<4);
        KP_PORT &= ~(1 << (7 - r));
        _delay_us(10);

        uint8_t cols = KP_PINR & 0x0F;
        if (cols != 0x0F) {
            _delay_ms(20);
            cols = KP_PINR & 0x0F;
            for (uint8_t c = 0; c < 4; c++) {
                if (!(cols & (1 << (3 - c)))) {
                    while (!(KP_PINR & (1 << (3 - c))));
                    KP_PORT |= (1<<7)|(1<<6)|(1<<5)|(1<<4);
                    return key_map[r][c];
                }
            }
        }
    }
    KP_PORT |= (1<<7)|(1<<6)|(1<<5)|(1<<4);
    return 0;
}

static char keypad_wait_for_key(void)
{
    char k;
    do { k = keypad_get_key(); } while (!k);
    return k;
}

/* =========================================================
 * EEPROM
 * ========================================================= */
static void salveaza_pin(const char *pin)
{
    for (uint8_t i = 0; i < 4; i++)
        eeprom_write_byte((uint8_t *)(uintptr_t)i, (uint8_t)pin[i]);
}

static void citeste_pin(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t c = eeprom_read_byte((uint8_t *)(uintptr_t)i);
        if (c == 0xFF || c == 0x00) { strcpy(master_pin, "1234"); return; }
        master_pin[i] = (char)c;
    }
    master_pin[4] = '\0';
}

/* =========================================================
 * TIMER0 -> millis()
 * Prescaler 64 -> ~1.024ms/OVF (identic cu Arduino)
 * ========================================================= */
static volatile uint32_t t0_ms = 0;

ISR(TIMER0_OVF_vect) { t0_ms++; }

static void timer0_init(void)
{
    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << TOIE0);
}

static uint32_t millis(void)
{
    uint32_t m; cli(); m = t0_ms; sei(); return m;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t = millis();
    while ((millis() - t) < ms);
}

/* =========================================================
 * LOGICA APLICATIE
 * ========================================================= */
static void reset_interfata(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Introduceti Cod:");
    lcd_set_cursor(0, 1);
    input_pin[0]  = '\0';
    mod_schimbare = 0;
}

static void gestioneaza_acces(void)
{
    char buf[32];

    relay_on();        
    servo_write(180);  // Deschidere completă la 180 grade
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("BINE ATI VENIT!");
    uart_puts("Acces Permis. Usa deschisa, bec aprins.\r\n");

    delay_ms(5000);

    uint32_t ultima_detectie = millis();

    while (1) {
        uint16_t dist = citeste_distanta();

        snprintf(buf, sizeof(buf), "Senzor: %u cm | ", dist);
        uart_puts(buf);

        if (dist <= 10) {
            uart_puts("PREZENTA DETECTATA\r\n");
            ultima_detectie = millis();
        } else {
            uart_puts("ZONA LIBERA\r\n");
        }

        if ((millis() - ultima_detectie) > 5000UL) {
            uart_puts("Timp expirat. Se declanseaza inchiderea.\r\n");
            break;
        }

        delay_ms(500);
    }

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Inchidere...");
    uart_puts("Se inchide usa si becul simultan.\r\n");

    servo_write(0);
    relay_off();
    
    delay_ms(2000);
    reset_interfata();
}

static void verifica_cod(void)
{
    lcd_clear();

    if (mod_schimbare) {
        if (strcmp(input_pin, master_pin) == 0) {
            lcd_set_cursor(0, 0);
            lcd_print("Cod Nou:");
            input_pin[0]  = '\0';
            mod_schimbare = 0;

            uint8_t len = 0;
            while (len < 4) {
                char t = keypad_wait_for_key();
                if (t >= '0' && t <= '9') {
                    input_pin[len++] = t;
                    input_pin[len]   = '\0';
                    lcd_set_cursor(len - 1, 1);
                    lcd_print_char('*');
                }
            }
            strcpy(master_pin, input_pin);
            salveaza_pin(master_pin);
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Pin Salvat!");
            delay_ms(2000);
            reset_interfata();
        } else {
            lcd_set_cursor(0, 0);
            lcd_print("Eroare Auth!");
            delay_ms(2000);
            reset_interfata();
        }
    } else {
        if (strcmp(input_pin, master_pin) == 0) {
            gestioneaza_acces();
        } else {
            lcd_set_cursor(0, 0);
            lcd_print("Cod Incorect!");
            delay_ms(2000);
            reset_interfata();
        }
    }
}

/* =========================================================
 * MAIN
 * ========================================================= */
int main(void)
{
    uart_init();
    twi_init();
    lcd_init();
    relay_init();
    ultrasonic_init();
    keypad_init();
    timer0_init();
    servo_init();   /* Timer1 FastPWM hardware, nu necesita sei() pentru PWM */
    sei();          /* activare ISR Timer0 pentru millis() */

    citeste_pin();
    reset_interfata();
    uart_puts("Sistem pornit. Asteptare cod...\r\n");

    while (1) {
        char tasta = keypad_get_key();
        if (!tasta) continue;

        if (tasta == '#') {
            verifica_cod();
        } else if (tasta == '*') {
            mod_schimbare = 0;
            reset_interfata();
        } else if (tasta == 'A') {
            input_pin[0]  = '\0';
            mod_schimbare = 1;
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Cod Actual:");
        } else {
            uint8_t len = (uint8_t)strlen(input_pin);
            if (len < 4) {
                input_pin[len]     = tasta;
                input_pin[len + 1] = '\0';
                lcd_set_cursor(len, 1);
                lcd_print_char('*');
            }
        }
    }

    return 0;
}