#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_rtc.h"
#include "lpc17xx_dac.h"
#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"
#include "joystick.h"
#include "pca9532.h"

#define SBIT_TIMER0  1
#define SBIT_TIMER1  2

#define SBIT_MR0I    0
#define SBIT_MR0R    1

#define SBIT_CNTEN   0

#define PCLK_TIMER0  2
#define PCLK_TIMER1  4

#define LED1         0 // P2_0
#define LED2         1 // P2_1

#define MiliToMicroSec(x)(x*1000)  /* ms is multiplied by 1000 to get us*/

#define NOTE_PIN_HIGH() GPIO_SetValue(0, (uint32_t) 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, (uint32_t) 1<<26);

extern const unsigned char sound_8k[];
extern int sound_sz;

static uint32_t msTicks = 0;

uint32_t ctr = 0;
uint32_t off = 0;
uint32_t sampleRate = 0;
uint32_t delay = 0;


void SysTick_Handler(void) {
    msTicks++;
}

static uint32_t getTicks(void) {
    return msTicks;
}

/*!
 *  @brief    Inicjalizacja przetwornika DAC
 *  @returns  0 - jesli inicjaliazja sie powiedzie,
 *  1 - jesli inicjaliazja sie nie powiedzie
 */
static int init_dac(void) {
	PINSEL_CFG_Type PinCfg;

    GPIO_SetDir(2, 1 << 0, 1);
    GPIO_SetDir(2, 1 << 1, 1);

    GPIO_SetDir(0, (uint32_t) 1 << 27, 1);
    GPIO_SetDir(0, (uint32_t) 1 << 28, 1);
    GPIO_SetDir(2, (uint32_t) 1 << 13, 1);
    GPIO_SetDir(0, (uint32_t) 1 << 26, 1);

    GPIO_ClearValue(0, (uint32_t) 1 << 27); //LM4811-clk
    GPIO_ClearValue(0, (uint32_t) 1 << 28); //LM4811-up/dn
    GPIO_ClearValue(2, (uint32_t) 1 << 13); //LM4811-shutdn

    /*
     * Init DAC pin connect
    * AOUT on P0.26
    */
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);

    /* init DAC structure to default
     * Maximum	current is 700 uA
     * First value to AOUT is 0
     */
    DAC_Init(LPC_DAC);


    /* ChunkID */
    if (sound_8k[ctr] != 'R' && sound_8k[ctr + 1] != 'I' &&
        sound_8k[ctr + 2] != 'F' && sound_8k[ctr + 3] != 'F') {
        return 0;
    }
    ctr += 4;

    /* skip chunk size*/
    ctr += 4;

    /* Format */
    if (sound_8k[ctr] != 'W' && sound_8k[ctr + 1] != 'A' &&
        sound_8k[ctr + 2] != 'V' && sound_8k[ctr + 3] != 'E') {
        return 0;
    }
    ctr += 4;

    /* SubChunk1ID */
    if (sound_8k[ctr] != 'f' && sound_8k[ctr + 1] != 'm' &&
        sound_8k[ctr + 2] != 't' && sound_8k[ctr + 3] != ' ') {
        return 0;
    }
    ctr += 4;

    /* skip chunk size, audio format, num channels */
    ctr += 8;

    sampleRate = (sound_8k[ctr] | (sound_8k[ctr + 1] << 8) |
                  (sound_8k[ctr + 2] << 16) | (sound_8k[ctr + 3] << 24));

    if (sampleRate != 8000) {
        return 0;
    }

    delay = 1000000 / sampleRate;

    ctr += 4;

    /* skip byte rate, align, bits per sample */
    ctr += 8;

    /* SubChunk2ID */
    if (sound_8k[ctr] != 'd' && sound_8k[ctr + 1] != 'a' &&
        sound_8k[ctr + 2] != 't' && sound_8k[ctr + 3] != 'a') {
        return 0;
    }
    ctr += 4;

    /* skip chunk size */
    ctr += 4;

    off = ctr;
}

/*!
 *  @brief    inicjalizacja SSP.
 */
static void init_ssp(void) {
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize SPI pin connect
     * P0.7 - SCK;
     * P0.8 - MISO
     * P0.9 - MOSI
     * P2.2 - SSEL - used as GPIO
     */
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 7;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);

    // Initialize SSP peripheral with parameter given in structure above
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

    // Enable SSP peripheral
    SSP_Cmd(LPC_SSP1, ENABLE);

}

/*!
 *  @brief    Inicjalizacja I2C
 */
static void init_i2c(void) {
    PINSEL_CFG_Type PinCfg;

    /* Initialize I2C2 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize I2C peripheral
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C1 operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}

/*!
 *  @brief    Wyswietlenie domyslego widoku na ekranie OLED
 */
static void print_start_screen_content() {
    oled_clearScreen(OLED_COLOR_WHITE);
    oled_putString(1, 1, (uint8_t *) "Temp(C): ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 10, (uint8_t *) "Time  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 19, (uint8_t *) "Stoper: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 28, (uint8_t *) "Budz  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

/*!
 *  @brief       Procedura odpowiedzialna za odpowiednia
 *               interakcje po ruchu joystickiem
 *  @param joy
 *               zmienna, w ktorej znajduje sie wartosc
 *               odczytana po ruchu joysticka
 *  @param *AlarmHour
 *               Wskaznik na zmienna przechowujaca informacje
 *               o godzine alarmu
 *  @param *AlarmMin
 *               Wskaznik na zmienna przechowujaca informacje
 *               o godzine alarmu
 */
static void joystick_movement(uint8_t joy, uint32_t *AlarmHour, uint32_t *AlarmMin) {
    if ((joy & JOYSTICK_UP) != 0) {
        LPC_TIM1->TCR |= (1<<1);
        LPC_TIM1->TCR = 1;
    }
    if ((joy & JOYSTICK_DOWN) != 0) {
        LPC_TIM1->TCR = 0;
    }
    if ((joy & JOYSTICK_LEFT) != 0) {
        if (*AlarmMin == 59) {
            *AlarmMin = 0;
            (*AlarmHour)++;
        } else {
            (*AlarmMin)++;
        }
    }
    if ((joy & JOYSTICK_RIGHT) != 0) {
        if (*AlarmHour >= 23) {
            *AlarmHour = 0;
        } else {
            (*AlarmHour)++;
        }
    }
}

/*!
 *  @brief       Procedura odpowiedzialna za sekwencje
 *               swiecenia diod
 *  @param *ledOn
 *               wskaznik na zmienna, okreslajaca czy led sie swieci
 *  @param *ledOff
 *               wskaznik na zmienna, okreslajaca czy led sie nie swieci
 *  @param *cnt
 *               wskaznik na zmienna, okreslajaca ktory led ma sie swiecic
 */
static void leds_dancing(uint32_t *ledOn, uint32_t *ledOff, uint32_t *cnt) {
    uint8_t dir = 0;

    if (LPC_TIM1->TCR == 1) {
        if (*cnt < (uint32_t) 16) {
            *ledOn |= ((uint32_t) 1 << *cnt);
        }
        if (*cnt > (uint32_t) 15) {
            *ledOn &= ~((uint32_t) 1 << (*cnt - (uint32_t) 16));
        }
        if (*cnt > (uint32_t) 15) {
            *ledOff |= ((uint32_t) 1 << (*cnt - (uint32_t) 16));
        }
        if (*cnt < (uint32_t) 16) {
            *ledOff &= ~((uint32_t) 1 << *cnt);
        }

        pca9532_setLeds(*ledOn, *ledOff);

        if (dir == 0) {
            if (*cnt == (uint32_t) 0) {
                *cnt = 31;
            } else {
                (*cnt)--;
            }
        } else {
            (*cnt)++;
            if (*cnt >= (uint32_t) 32) {
                *cnt = 0;
            }
        }
    }
}

/*!
 *  @brief     Wyswietla na ekranie informacje o alarmie
 *             oraz uruchamia glosnik kiedy jest alarm.
 *             Gdy odczytane natezenie swiatla z czujnika
 *             bedzie mniejsze niz 100 luksow to wylaczamy
 *             budzik
 *  @param *AlarmHour
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine alarmu
 *  @param *AlarmMin
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine alarmu
 *  @param *RTCHour
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine zegara RTC
 *  @param *RTCmin
 *            Wskaznik na zmienna przechowujaca informacje
 *            o minutach zegara RTC
 *  @param *RTCSec
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine zegara RTC
 */
static void display_alarm_message(uint32_t *AlarmHour, uint32_t *AlarmMin, uint32_t *RTCHour,
                                  uint32_t *RTCMin, uint32_t *RTCSec) {
    if ((*AlarmMin == *RTCMin) &&
        (*AlarmHour == *RTCHour) && (*RTCSec == 0)) {
        oled_clearScreen(OLED_COLOR_WHITE);
        oled_putString(32, 27, (uint8_t *) "ALARM!!!",
                       OLED_COLOR_BLACK, OLED_COLOR_WHITE);

        int tmp = 0;

        while (tmp == 0) {
            ctr = off;
            while (ctr++ < sound_sz) {
               uint32_t	wart = sound_8k[ctr];
			   wart = wart << 2;
               DAC_UpdateValue(LPC_DAC, wart);
               Timer0_us_Wait(delay);
            }
            if (light_read() < 100) {
                tmp = 1;
                print_start_screen_content();
                break;
            }
        }
    }
}

/*!
 *  @brief    Aktualizuje zawartosc ekranu
 *  @param temp
 *            Wartosc odczytanej temperatury z termometru
 *  @param *RTCHour
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine zegara RTC
 *  @param *RTCmin
 *            Wskaznik na zmienna przechowujaca informacje
 *            o minutach zegara RTC
 *  @param *RTCSec
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine zegara RTC
 *  @param *AlarmHour
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine alarmu
 *  @param *AlarmMin
 *            Wskaznik na zmienna przechowujaca informacje
 *            o godzine alarmu
 *  @param *buf
 *            Wskaznik na bufor przechowujacy tekst
 */
static void update_screen_content(uint32_t temp, uint32_t *RTCHour, uint32_t *RTCMin,
                                  uint32_t *RTCSec, uint32_t *stoper, uint32_t *AlarmHour, uint32_t *AlarmMin,
                                  uint8_t *buf) {
    sprintf(buf, "Temp(C):%2d,%d", temp / 10, temp % 10);
    oled_putString(1, 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    sprintf(buf, "Czas :%02d:%02d:%02d", *RTCHour, *RTCMin, *RTCSec);
    oled_putString(1, 10, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    sprintf(buf, "Stoper :%4d", *stoper);
    oled_putString(1, 19, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    sprintf(buf, "Budzik : %02d:%02d", *AlarmHour, *AlarmMin);
    oled_putString(1, 28, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

int main(void) {
 	init_i2c();
	init_ssp();
    pca9532_init();
    joystick_init();
    oled_init();

    if (SysTick_Config(SystemCoreClock / 1000) == 1) {
        while (1);  // Capture error
    }

    temp_init(&getTicks);
    light_enable();
    light_setRange(LIGHT_RANGE_1000);
    init_dac();


    LPC_RTC->CCR = 1;
    LPC_RTC->HOUR = 12;
    LPC_RTC->MIN = 40;
    LPC_RTC->SEC = 50;

    print_start_screen_content();
    LPC_TIM1->TCR |= (1<<1); //Reset Counter
    LPC_TIM1->TCR &= ~(1<<1); //release reset
    LPC_TIM1->TCR = 0;
    LPC_TIM1->PR = 25000000 - 1;

    uint32_t AlarmHour = LPC_RTC->HOUR;
    uint32_t AlarmMin = (LPC_RTC->MIN);
    uint32_t cnt = 0;
    uint32_t ledOn = 0;
    uint32_t ledOff = 0;
    uint8_t buf[10];

    AlarmMin++;

    while (1) {
        uint32_t RTCSec = LPC_RTC->SEC;
        uint32_t RTCMin = LPC_RTC->MIN;
        uint32_t RTCHour = LPC_RTC->HOUR;
        uint32_t stoper = LPC_TIM1->TC;

        update_screen_content(temp_read(), &RTCHour, &RTCMin,
                              &RTCSec, &stoper, &AlarmHour, &AlarmMin, &buf[0]);
        joystick_movement(joystick_read(), &AlarmHour, &AlarmMin);
        leds_dancing(&ledOn, &ledOff, &cnt);
        display_alarm_message(&AlarmHour, &AlarmMin,
                              &RTCHour, &RTCMin, &RTCSec);
    }
}
