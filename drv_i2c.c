/*
   drv_i2c.c :  I^2C support for STM32F103

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_i2c.c

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#define I2C_DEVICE (I2CDEV_2)

#include <stdbool.h>
#include <stdio.h>
#include <string.h> // memset

#include "stm32f10x_conf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"
#include "breezystm32.h"

#include "drv_i2c.h"

#ifndef SOFT_I2C

#define I2C_struct_init_args false, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, NULL, NULL, NULL, 0, 0, 0, NULL

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7

// I2C Interrupt Handlers
static void i2c_er_handler(uint8_t index);
static void i2c_ev_handler(uint8_t index);
static void i2cUnstick(uint8_t index);

// I2C Circular Buffer Variables
static i2cJob_t i2c_buffer[3][I2C_BUFFER_SIZE];
static void i2c_init_buffer(uint8_t dev_index);
static void i2c_job_handler(uint8_t index);

typedef struct i2cDevice_t {
    I2C_TypeDef *dev;
    GPIO_TypeDef *gpio;
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;

    bool error;
    bool busy;
    bool initialized;

    uint8_t subaddress_sent;
    uint8_t final_stop;                         // flag to indicate if subaddess sent, flag to indicate final bus condition
    int8_t index;                               // index is signed -1 == send the subaddress
    uint8_t error_count;
    uint8_t addr;
    uint8_t reg;
    uint8_t bytes;
    uint8_t writing;
    uint8_t reading;
    uint8_t *write_p;
    uint8_t *read_p;
    volatile uint8_t *status;

    volatile uint8_t i2c_buffer_head;
    volatile uint8_t i2c_buffer_tail;
    volatile uint8_t i2c_buffer_count;
    void (*complete_CB)(void);
} i2cDevice_t;

static volatile i2cDevice_t i2cHardwareMap[3] = {
    { I2C1, I2CDEV1_GPIO_PORT, I2CDEV1_SCL_PIN, I2CDEV1_SDA_PIN,
      I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_APB1Periph_I2C1, I2C_struct_init_args },
    { I2C2, I2CDEV2_GPIO_PORT, I2CDEV2_SCL_PIN, I2CDEV2_SDA_PIN,
      I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_APB1Periph_I2C2, I2C_struct_init_args },
    { I2C2, I2CDEV3_GPIO_PORT, I2CDEV3_SCL_PIN, I2CDEV3_SDA_PIN,
      I2C3_EV_IRQn, I2C3_ER_IRQn, RCC_APB1Periph_I2C3, I2C_struct_init_args }
};

void I2C1_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_2);
}

void I2C3_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_3);
}


#define I2C_DEFAULT_TIMEOUT 30000

static bool i2cHandleHardwareFailure(uint8_t index)
{
    i2cHardwareMap[index].error_count++;
    // reinit peripheral + clock out garbage
    i2cInit(index);
    return false;
}

bool i2cWriteBuffer(uint8_t index, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if(index >= I2CDEV_COUNT)
        return false;

    volatile i2cDevice_t* device = & i2cHardwareMap[index];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    device->addr = addr_ << 1;
    device->reg = reg_;
    device->writing = 1;
    device->reading = 0;
    device->write_p = data;
    device->read_p = data;
    device->bytes = len_;
    device->busy = 1;
    device->error = false;

    if (!device->initialized)
        return false;

    if(!device->dev)
        return false;

    if (!(device->dev->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(device->dev->CR1 & 0x0100)) {                                    // ensure sending a start
            while (device->dev->CR1 & 0x0200 && --timeout > 0) {
                ;    // wait for any stop to finish sending
            }
            if (timeout == 0)
                return i2cHandleHardwareFailure(index);
            I2C_GenerateSTART(device->dev, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (device->busy && --timeout > 0) {
        ;
    }
    if (timeout == 0)
        return i2cHandleHardwareFailure(index);

    return !device->error;
}

bool i2cWrite(uint8_t index, uint8_t addr_, uint8_t reg_, uint8_t data)
{

    return i2cWriteBuffer(index, addr_, reg_, 1, &data);
}

bool i2cRead(uint8_t index, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    if(index >= I2CDEV_COUNT)
        return false;

    volatile i2cDevice_t* device = &i2cHardwareMap[index];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    device->addr = addr_ << 1;
    device->reg = reg_;
    device->writing = 0;
    device->reading = 1;
    device->read_p = buf;
    device->write_p = buf;
    device->bytes = len;
    device->busy = 1;
    device->error = false;

    if(!device->initialized)
        return false;

    if (!device->dev)
        return false;

    if (!(device->dev->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(device->dev->CR1 & 0x0100)) {                                    // ensure sending a start
            while (device->dev->CR1 & 0x0200 && --timeout > 0) {
                ;    // wait for any stop to finish sending
            }
            if (timeout == 0)
                return i2cHandleHardwareFailure(index);
            I2C_GenerateSTART(device->dev, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (device->busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure(index);

    return !device->error;
}

bool i2cReadAsync(uint8_t index, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf, volatile uint8_t* status_, void (*CB)(void))
{
    if(index >= I2CDEV_COUNT)
        return false;

    volatile i2cDevice_t* device = &i2cHardwareMap[index];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    device->addr = addr_ << 1;
    device->reg = reg_;
    device->writing = 0;
    device->reading = 1;
    device->read_p = buf;
    device->write_p = buf;
    device->bytes = len;
    device->busy = 1;
    device->error = false;
    device->status = status_;
    device->complete_CB = CB;

    if(!device->dev)
        return false;

    if (!(device->dev->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(device->dev->CR1 & 0x0100)) {                                    // ensure sending a start
            while (device->dev->CR1 & 0x0200 && --timeout > 0) {               // This is blocking, but happens only
                ;    // wait for any stop to finish sending             // if we are stomping on the port (try to avoid)
            }
            if (timeout == 0)
                return i2cHandleHardwareFailure(index);
            I2C_GenerateSTART(device->dev, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }
    return true;
}

bool i2cWriteAsync(uint8_t index, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *buf_, volatile uint8_t* status_, void (*CB)(void))
{
    if(index >= I2CDEV_COUNT)
        return false;

    volatile i2cDevice_t* device = &i2cHardwareMap[index];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    device->addr = addr_ << 1;
    device->reg = reg_;
    device->writing = 1;
    device->reading = 0;
    device->write_p = buf_;
    device->read_p = buf_;
    device->bytes = len_;
    device->busy = 1;
    device->error = false;
    device->status = status_;
    device->complete_CB = CB;

    if (!device->dev)
        return false;

    if (!(device->dev->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(device->dev->CR1 & 0x0100)) {                                    // ensure sending a start
            while (device->dev->CR1 & 0x0200 && --timeout > 0) {
                ;    // wait for any stop to finish sending
            }
            if (timeout == 0)
                return i2cHandleHardwareFailure(index);
            I2C_GenerateSTART(device->dev, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }
    return true;
}

static void i2c_er_handler(uint8_t index)
{
    volatile i2cDevice_t* device = &i2cHardwareMap[index];

    // Read the I2C1 status register
    volatile uint32_t SR1Register = device->dev->SR1;

    if (SR1Register & 0x0F00)                                           // an error
        device->error = true;

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & 0x0700) {
        (void)device->dev->SR2;                                                        // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(device->dev, I2C_IT_BUF, DISABLE);                                // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & I2C_SR1_ARLO) && !(device->dev->CR1 & I2C_CR1_STOP)) {     // if we dont have an ARLO error, ensure sending of a stop
            if (device->dev->CR1 & I2C_CR1_START) {                                    // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (device->dev->CR1 & I2C_CR1_START) { ; }                         // wait for any start to finish sending
                I2C_GenerateSTOP(device->dev, ENABLE);                                 // send stop to finalise bus transaction
                while (device->dev->CR1 & I2C_CR1_STOP) { ; }                          // wait for stop to finish sending
                i2cInit(index);                                                // reset and configure the hardware
            } else {
                I2C_GenerateSTOP(device->dev, ENABLE);                                 // stop to free up the bus
                I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    device->dev->SR1 &= ~0x0F00;                                                       // reset all the error bits to clear the interrupt
    device->busy = 0;
}

void i2c_ev_handler(uint8_t dev_index)
{
    volatile i2cDevice_t* dev = &i2cHardwareMap[dev_index];
    uint8_t SReg_1 = dev->dev->SR1;                                              // read the status register here

    if (SReg_1 & 0x0001) {                                                       // we just sent a start - EV5 in ref manual
        dev->dev->CR1 &= ~0x0800;                                                // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(dev->dev, ENABLE);                                 // make sure ACK is on
        dev->index = 0;                                                          // reset the index
        if (dev->reading && (dev->subaddress_sent || 0xFF == dev->reg)) {        // we have sent the subaddr
            dev->subaddress_sent = 1;                                            // make sure this is set in case of no subaddress, so following code runs correctly
            if (dev->bytes == 2)
                dev->dev->CR1 |= 0x0800;                                         // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(dev->dev, dev->addr, I2C_Direction_Receiver);    // send the address and set hardware mode
        } else {                                                                 // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(dev->dev, dev->addr, I2C_Direction_Transmitter); // send the address and set hardware mode
            if (dev->reg != 0xFF)                                                // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                dev->index = -1;                                                 // send a subaddress
        }

    } else if (SReg_1 & 0x0002) {                                                // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                                 // memory fence to control hardware
        if (dev->bytes == 1 && dev->reading && dev->subaddress_sent) {           // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(dev->dev, DISABLE);                            // turn off ACK
            __DMB();
            (void)dev->dev->SR2;                                                 // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(dev->dev, ENABLE);                                  // program the stop
            dev->final_stop = 1;
            I2C_ITConfig(dev->dev, I2C_IT_BUF, ENABLE);                          // allow us to have an EV7
        } else {                                                                 // EV6 and EV6_1
            (void)dev->dev->SR2;                                                 // clear the ADDR here
            __DMB();
            if (dev->bytes == 2 && dev->reading && dev->subaddress_sent)         // rx 2 bytes - EV6_1
            {
                I2C_AcknowledgeConfig(dev->dev, DISABLE);                        // turn off ACK
                I2C_ITConfig(dev->dev, I2C_IT_BUF, DISABLE);                     // disable TXE to allow the buffer to fill
            } else if (dev->bytes == 3 && dev->reading && dev->subaddress_sent)  // rx 3 bytes
                I2C_ITConfig(dev->dev, I2C_IT_BUF, DISABLE);                     // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                                 // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(dev->dev, I2C_IT_BUF, ENABLE);
        }

    } else if (SReg_1 & 0x004) {                                                 // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        dev->final_stop = 1;
        if (dev->reading && dev->subaddress_sent) {                              // EV7_2, EV7_3
            if (dev->bytes > 2) {                                                // EV7_2
                I2C_AcknowledgeConfig(dev->dev, DISABLE);                        // turn off ACK
                dev->read_p[dev->index++] = (uint8_t)dev->dev->DR;               // read data N-2
                I2C_GenerateSTOP(dev->dev, ENABLE);                              // program the Stop
                dev->final_stop = 1;                                             // required to fix hardware
                dev->read_p[dev->index++] = (uint8_t)dev->dev->DR;               // read data N - 1
                I2C_ITConfig(dev->dev, I2C_IT_BUF, ENABLE);                      // enable TXE to allow the final EV7
            } else {                                                             // EV7_3
                if (dev->final_stop){
                    I2C_GenerateSTOP(dev->dev, ENABLE);                          // program the Stop
                }
                else
                    I2C_GenerateSTART(dev->dev, ENABLE);                         // program a rep start
                dev->read_p[dev->index++] = (uint8_t)dev->dev->DR;               // read data N - 1
                dev->read_p[dev->index++] = (uint8_t)dev->dev->DR;               // read data N
                dev->index++;                                                    // to show job completed
            }
        } else {                                                                 // EV8_2, which may be due to a subaddress sent or a write completion
            if (dev->subaddress_sent || (dev->writing)) {
                if (dev->final_stop)
                    I2C_GenerateSTOP(dev->dev, ENABLE);                          // program the Stop
                else
                    I2C_GenerateSTART(dev->dev, ENABLE);                         // program a rep start
                dev->index++;                                                    // to show that the job is complete
            } else {                                                             // We need to send a subaddress
                I2C_GenerateSTART(dev->dev, ENABLE);                             // program the repeated Start
                dev->subaddress_sent = 1;                                        // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (dev->dev->CR1 & 0x0100) {
            ;
        }

    } else if (SReg_1 & 0x0040) {                                 // Byte received - EV7
        dev->read_p[dev->index++] = (uint8_t)dev->dev->DR;
        if (dev->bytes == (dev->index + 3))
            I2C_ITConfig(dev->dev, I2C_IT_BUF, DISABLE);                     // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (dev->bytes == dev->index)                                        // We have completed a final EV7
            dev->index++;                                                    // to show job is complete
    } else if (SReg_1 & 0x0080) {                                            // Byte transmitted EV8 / EV8_1
        if (dev->index != -1) {                                              // we dont have a subaddress to send
            dev->dev->DR = dev->write_p[dev->index++];
            if (dev->bytes == dev->index)                                    // we have sent all the data
                I2C_ITConfig(dev->dev, I2C_IT_BUF, DISABLE);                 // disable TXE to allow the buffer to flush
        } else {
            dev->index++;
            dev->dev->DR = dev->reg;                                         // send the subaddress
            if (dev->reading || !dev->bytes)                                 // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(dev->dev, I2C_IT_BUF, DISABLE);                 // disable TXE to allow the buffer to flush
        }
    }

    if (dev->index == dev->bytes + 1) {                                      // we have completed the current job
        dev->subaddress_sent = 0;                                            // reset this here
        if (dev->final_stop) {                                               // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(dev->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);        // Disable EVT and ERR interrupts while bus inactive
        }
        if (dev->status != NULL){
            (*dev->status) = I2C_JOB_COMPLETE;                               // Update status
        }
        if (dev->complete_CB != NULL){
            dev->complete_CB();                                              // Call the custom callback (we are finished)
        }
        dev->busy = 0;
        i2c_job_handler(dev_index);                                          // Start the next job (if there is one on the queue)
    }
}

bool i2cInit(uint8_t index)
{
    if (index >= I2CDEV_COUNT)
        return false;

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

    volatile i2cDevice_t* device = &i2cHardwareMap[index];

    if (device->gpio == I2CDEV_DISABLED) // this device is disabled
        return false;

    // Turn on peripheral clock, save device and index
//    I2C_TypeDef* I2Cx = i2cHardwareMap[index].dev;
    RCC_APB1PeriphClockCmd(device->peripheral, ENABLE);

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    i2cUnstick(index);

    // Init I2C peripheral
    I2C_DeInit(device->dev);
    I2C_StructInit(&i2c);

    I2C_ITConfig(device->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Enable EVT and ERR interrupts - they are enabled by the first request
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Cmd(device->dev, ENABLE);
    I2C_Init(device->dev, &i2c);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = device->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = device->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&nvic);

    // Initialize buffer
    i2c_init_buffer(index);
    device->initialized = true;
    return true;
}

uint16_t i2cGetErrorCounter(uint8_t index)
{
    if(index <= I2CDEV_COUNT)
        return 0;
    else
        return i2cHardwareMap[index].error_count;
}

static void i2cUnstick(uint8_t index)
{
    volatile i2cDevice_t* device = &i2cHardwareMap[index];

    GPIO_TypeDef *gpio;
    gpio_config_t cfg;
    uint16_t scl, sda;
    int i;

    // prepare pins
    gpio = device->gpio;
    scl = device->scl;
    sda = device->sda;

    digitalHi(gpio, scl | sda);

    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_Out_OD;
    gpioInit(gpio, &cfg);

    for (i = 0; i < 8; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpio, scl))
            delayMicroseconds(10);

        // Pull low
        digitalLo(gpio, scl); // Set bus low
        delayMicroseconds(10);
        // Release high again
        digitalHi(gpio, scl); // Set bus high
        delayMicroseconds(10);
    }

    // Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpio, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpio, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_AF_OD;
    gpioInit(gpio, &cfg);
}
//=============================================================
// Asynchronous (Interrupt Driven I2C)
//=============================================================
void i2c_job_handler(uint8_t index)
{
    volatile i2cDevice_t* dev = &i2cHardwareMap[index];
    if(dev->i2c_buffer_count == 0)
    {
        // the queue is empty, stop performing i2c until
        // a new job is enqueued
        return;
    }

    if(dev->busy)
    {
        // wait for the current job to finish.  This function
        // will get called again when the job is done
        return;
    }

    // Perform the job on the front of the queue
    i2cJob_t* job = i2c_buffer[index] + dev->i2c_buffer_tail;

    // First, change status to BUSY
    (*job->status) = I2C_JOB_BUSY;

    // perform the appropriate job
    if(job->type == READ)
    {
        i2cReadAsync(job->index,
                     job->addr,
                     job->reg,
                     job->length,
                     job->data,
                     job->status,
                     job->CB);
    }
    else
    {
        i2cWriteAsync(job->index,
                      job->addr,
                      job->reg,
                      job->length,
                      job->data,
                      job->status,
                      job->CB);
    }

    // Increment the tail
    dev->i2c_buffer_tail = (dev->i2c_buffer_tail +1)%I2C_BUFFER_SIZE;

    // Decrement the number of jobs on the buffer
    dev->i2c_buffer_count--;
    return;
}

void i2c_queue_job(uint8_t index, i2cJobType_t type, uint8_t addr_, uint8_t reg_, uint8_t *data, uint8_t length, volatile uint8_t* status_, void (*CB)(void))
{
    volatile i2cDevice_t* dev = &i2cHardwareMap[index];

    // Get a pointer to the head
    i2cJob_t* job = i2c_buffer[index] + dev->i2c_buffer_head;

    // save the data about the job
    job->type = type;
    job->data = data;
    job->addr = addr_;
    job->reg = reg_;
    job->length = length;
    job->index = index;
    job->status = status_;
    job->CB = CB;

    // change job status
    (*job->status) = I2C_JOB_QUEUED;

    // Increment the buffer size
    dev->i2c_buffer_count++;

    // Increment the buffer head for next call
    dev->i2c_buffer_head = (dev->i2c_buffer_head + 1)%I2C_BUFFER_SIZE;

    if(dev->i2c_buffer_count == 1)
    {
        // if the buffer queue was empty, restart i2c job handling
        i2c_job_handler(index);
    }

    return;
}

void i2c_init_buffer(uint8_t index)
{
    volatile i2cDevice_t* dev = &i2cHardwareMap[index];
    // write zeros to the buffer, and set all the indexes to zero
    memset(i2c_buffer[index], 0, I2C_BUFFER_SIZE*sizeof(i2cJob_t));
    dev->i2c_buffer_count = 0;
    dev->i2c_buffer_head = 0;
    dev->i2c_buffer_tail = 0;
}





#endif
