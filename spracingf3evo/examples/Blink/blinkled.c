
void serialEvent1(void) __attribute__((weak));

#include "platform.h"

#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"

// Arduino
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

// Board-specific
GPIO_TypeDef * gpio_type_from_pin(uint8_t pin);
uint16_t gpio_pin_from_pin(uint8_t pin);
serialPort_t * serial0_open(void);

void SetSysClock(void);

static serialPort_t * serial0;

void pinMode(uint8_t pin, uint8_t mode)
{
    // XXX currently support output mode only
    if (mode != OUTPUT) return;

    pin = 1<<pin;

    GPIO_TypeDef * gpio = gpio_type_from_pin(pin);

    gpio_config_t cfg;

    cfg.pin = gpio_pin_from_pin(pin);
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;

    gpioInit(gpio, &cfg);
}

void digitalWrite(uint8_t pin, uint8_t level)
{
    pin = 1<<pin;

    GPIO_TypeDef * gpio = gpio_type_from_pin(pin);

    uint16_t gpio_pin = gpio_pin_from_pin(pin);

    switch (level) {
        case HIGH:
            digitalLo(gpio, gpio_pin);
            break;
        case LOW:
            digitalHi(gpio, gpio_pin);
            break;
    }
}

void reset(void)
{
    systemReset();
}

void resetToBootloader(void)
{
    systemResetToBootloader();
}






int led = 4;

// the setup routine runs once when you press reset:
void setup() {                

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {

    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);               // wait for a second
}







int main(void) {

    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));

    SetSysClock();

    systemInit();

    timerInit();  // timer must be initialized before any channel is allocated

    serial0 = serial0_open();

    dmaInit();

    setup();

    while(1)
        loop();

//     while (true) {

// #ifndef EXTERNAL_DEBUG
//         static uint32_t dbg_start_msec;
//         // support reboot from host computer
//         if (millis()-dbg_start_msec > 100) {
//             dbg_start_msec = millis();
//             while (serialRxBytesWaiting(serial0)) {
//                 uint8_t c = serialRead(serial0);
//                 if (c == 'R') 
//                     systemResetToBootloader();
//             }
//         }
// #endif
//         loop();
//     }
} // main