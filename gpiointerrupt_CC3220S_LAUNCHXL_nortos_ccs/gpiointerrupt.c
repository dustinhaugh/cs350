/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver Configuration */
#include "ti_drivers_config.h"

/* State Machine Variables below */
volatile int word = 4;          //   4 = SOS, 5 = LED_OK
volatile int intword = 4;       //   4 = SOS, 5 = LED_OK
volatile int i = 0;             // iterator
volatile int state;

/* Flags */
volatile unsigned char TimerFlag = 0;          // 100ms timer period
volatile unsigned char RightButtonFlag = 0;    // 200ms check the button flags
volatile unsigned char LeftButtonFlag = 0;     // 200ms check the button flags

/* Timer Configuration */
int timer_period = 100000;


/* Thermostat Variables */
int16_t temperature;            // current room temperature read from sensor
int16_t setpoint = 20;          // my desired temperature in Celsius
int heat;
int seconds;


/*
 *  ======== UART BEGINS ========
 */

#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;
void initUART(void) {
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/*
 *  ======== I2C BEGINS ========
 */

/* I2C Global Variables */
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] =
{
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

/* Driver Handles - Global variables */
I2C_Handle i2c;

/* Must call initUART() before calling this function. */
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    };

    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    }

    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,"
                         "contact professor\n\r"));
    }
}

int16_t readTemp(void) {
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r"
                                     ,i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging "
                                     "USB and plugging back in.\n\r"));
    }
    return temperature;
}


/*
 *  ======== TIMER  ========
 */

/* timerCallback  */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
    /* TEST timercallback */
    //printf("timerCallback\n");
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    params.period =  timer_period;  /* setting to 100 ms which is the lowest common denominator*/
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while(1) {}
    }

    if(Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while(1) {}
    }

}

void gpioInit() {
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
}

/*
 *  ======== gpioButtonFxn0 Callback ========
 */
void gpioButtonFxn0(uint_least8_t index) {

    /* set necessary flags */
    // to toggle values use variable = x1 + x2 - variable, will toggle between x1 and x2
    RightButtonFlag = 1;
    printf("Set Temp Down 1 Degree Celsius\n");
}

/*
 *  ======== gpioButtonFxn1 Callback ========
 */
void gpioButtonFxn1(uint_least8_t index) {

    /* set necessary flags */
    // to toggle values use variable = x1 + x2 - variable, will toggle between x1 and x2
    LeftButtonFlag = 1;
    printf("Set Temp Up 1 Degree Celsius\n");
}

/*
 *  ======== Configure GPIOs ========
 */
void configureGPIO(void) {

    /* Call driver init functions */
    GPIO_init();
    /* Configure the RED LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    /* If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
}


/*
 * ======== Thermostat ========
 */
void thermostat(void) {

     /* periods */
     int check_button_period = 200000;
     int read_temp_period = 500000;
     int display_uart_period = 1000000;

     /* counters */
     int button_counter = 0;
     int temp_counter = 0;
     int display_counter = 0;

     /* loop */

     while(1){

         /* loop will proceed after timercallback */
         while (!TimerFlag) {}

         /* check buttons every 200 ms */
         if (button_counter >= check_button_period) {
             if(LeftButtonFlag == 1){
                setpoint += 1;
                printf("Temp is set to %d degrees\n", setpoint);
                printf("Current temp is %d degrees \n", temperature);
                printf("\n");
                LeftButtonFlag = 0;
                 }
             if(RightButtonFlag == 1){
                 setpoint -= 1;
                 printf("Temp is set to %d degrees \n", setpoint);
                 printf("Current temp is %d degrees\n", temperature);
                 printf("\n");
                 RightButtonFlag = 0;
             }
                 /* TEST BELOW to check setpoint */
                 // printf("temperature is set to %d\n", setpoint);
                 button_counter = 0;
         }

         /* read sensor temperature every 500 ms */
         if (temp_counter >= read_temp_period) {
             temperature = readTemp();

             if (setpoint >= temperature ){
                 /* Turn on user LED */
                 GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                 heat = 1;
             }
             else {
                 GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                 heat = 0;
             }

             /* TEST BELOW to check sensor temperature */
             // printf("current temp is %d \n", temperature);
             temp_counter = 0;
         }

         /* display every 1000 ms */
         if (display_counter >= display_uart_period) {

             DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds));
             seconds++;
             /* TEST display call below */
             //printf("Current Temp is %d, Set Temp is %d\n", temperature, setpoint);
             display_counter = 0;
         }

     button_counter += timer_period;
     temp_counter += timer_period;
     display_counter += timer_period;
     TimerFlag = 0;

     }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* configuration of LED GPIO */
    configureGPIO();

    /* initialize APIs */
    initUART();
    initI2C();
    initTimer();

    /* calls the thermostat */
    thermostat();

    return (NULL);
}

/* NOTES
 *
 * My UART PORT:  COM 6
 *
 */





















































