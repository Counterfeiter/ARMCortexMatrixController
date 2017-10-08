/*      Author: Sebastian Förster
 */

#include <stdint.h>
#include <stdbool.h>
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/epi.h"
#include "utils/uartstdio.h"
#include "utils/cpu_usage.h"
#include "LED_Matrix.h"
#include "driverlib/ssi.h"



int g_ui32uDMAErrCount;
uint32_t ui32Status;

void
uDMAErrorHandler(void)
{


    //
    // Check for uDMA error bit
    //
    ui32Status = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
        //setup_dma();
    }
}

//*****************************************************************************
//
// Blink the on-board LED and a hole bunch of RGB LEDs ;)
//
//*****************************************************************************
int main(void)
{

   //
	// Set the clocking to run directly from the crystal at 120MHz.
	//
	MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
											 SYSCTL_OSC_MAIN |
											 SYSCTL_USE_PLL |
											 SYSCTL_CFG_VCO_480), 120000000);

    volatile uint32_t ui32Loop;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xFF);


    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R12;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ui32Loop = SYSCTL_RCGCGPIO_R;

    //
    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTN_DIR_R = 0x01 | 0x02 | 0x04 | 0x08;
    GPIO_PORTN_DEN_R = 0x01 | 0x02 | 0x04 | 0x08;

    //
    // Enable processor interrupts.
    //

    setup_epi();
    setup_ssi2();

    setup_dma_ssi2();
    setup_timer_cap();

    setup_dma_epi_ping_pong();


    //

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        //GPIO_PORTN_DATA_R |= 0x01;

        //GPIO_PORTN_DATA_R ^= 0x01;


        //
        // Turn off the LED.
        //
        //GPIO_PORTN_DATA_R &= ~(0x01);

        //
        // Delay for a bit.
        //

    	while(g_timer_ready != -1) ;

    	g_timer_ready = 0;


    	if(actuell_outgoing_image == &image_A[0][0]) {
    		GPIO_PORTN_DATA_R |= 0x02;
    		if(actuell_income_image == incom_image_A) {
    			make_pwm(incom_image_B, &image_B[0][0]);
    		} else {
    			make_pwm(incom_image_A, &image_B[0][0]);
    		}

    		actuell_outgoing_image = &image_B[0][0];
    		GPIO_PORTN_DATA_R &= ~0x02;
    	} else {
    		if(actuell_income_image == incom_image_A) {
    			make_pwm(incom_image_B, &image_A[0][0]);
    		} else {
    			make_pwm(incom_image_A, &image_A[0][0]);
    		}
    		actuell_outgoing_image = &image_A[0][0];
    	}


    	//cause flicker :-/
    	//dma_counter = 0;

    }
}
