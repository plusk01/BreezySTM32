/*
   i2sniff.c : sniff and report I^2C devices 

   Copyright (C) 2016 Simon D. Levy 

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_adc.c

   Don't forget to supply external power for external sensors (like MB1242 sonar)!

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

#include <breezystm32f1.h>


bool i2c1_avail = false;
bool i2c2_avail = false;
void setup(void)
{
    i2c1_avail = i2cInit(I2CDEV_1);
    i2c2_avail = i2cInit(I2CDEV_2);
} 

void loop(void)
{
    uint8_t addr;

    printf("I2C1: %s, ----- I2C2: %s, ---\n", (i2c1_avail)? "on": "off", (i2c2_avail)?"on":"off" );
    for (addr=0; addr<128; ++addr)
    {
        if(i2c1_avail)
        {
            if (i2cWrite(I2CDEV_1, addr, 0xFF, 0x00))
            {
                printf("I2C1 Found device at address 0X%02X\n", addr);
            }
        }

        if(i2c2_avail)
        {
            if (i2cWrite(I2CDEV_2, addr, 0xFF, 0x00))
            {
                printf("I2C2 Found device at address 0X%02X\n", addr);
            }
        }
    }

    printf("--------------------------\n\n");

    delay(1000);
}
