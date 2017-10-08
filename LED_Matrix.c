/*
 * LED_Matrix.c
 *
 *  Created on: 02.04.2014
 *      Author: Sebastian Förster
 */

#include "LED_Matrix.h"
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c1294ncpdt.h"
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
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
#include "driverlib/epi.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"
#include "utils/cpu_usage.h"
#include "driverlib/ssi.h"
#include "gamma.h"


//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************


//*****************************************************************************
//
// Use the following to specify the GPIO pins used by the SDRAM EPI bus.
//
//*****************************************************************************
#define EPI_PORTA_PINS (GPIO_PIN_7 | GPIO_PIN_6)
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTK_PINS (GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTG_PINS (GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTQ_PINS (GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTN_PINS (GPIO_PIN_3)
#define EPI_PORTM_PINS (GPIO_PIN_3 | GPIO_PIN_2)
#define EPI_PORTB_PINS (GPIO_PIN_3)


//8 Bit orginal image
uint8_t incom_image_A[IMAGE_SIZE];
uint8_t incom_image_B[IMAGE_SIZE];

//8 Bit Image A and B
#if PANELS == 2
	uint16_t image_A[256][MAX_IN_BUF];
	uint16_t image_B[256][MAX_IN_BUF];
	volatile uint16_t *actuell_outgoing_image = &image_A[0][0];
#else
	uint8_t image_A[256][256];
	uint8_t image_B[256][256];
	volatile uint8_t *actuell_outgoing_image = &image_A[0][0];
#endif

volatile uint8_t *actuell_income_image = incom_image_A;

volatile int actuell_income_pos = 0;

volatile int g_timer_ready = 0;


#define SEND_DMA_LEN	1024/PANELS




volatile uint16_t dma_counter = SEND_DMA_LEN;

#if PANELS == 2
	volatile uint16_t *matrix_epi_buf;
#else
	volatile uint8_t *matrix_epi_buf;
#endif

void setup_ssi3(void);


void setup_epi(void)
{
    //
    // The EPI0 peripheral must be enabled for use.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    uint32_t ui32Val;

    //
    // EPI0S8 ~ EPI0S9: A6 ~ 7
    //
   ui32Val = HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL);
    ui32Val &= 0x00FFFFFF;
    ui32Val |= 0xFF000000;
    HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) = ui32Val;

    //
    // EPI0S10 ~ EPI0S11: G0 ~ 1
    //
    ui32Val = HWREG(GPIO_PORTG_BASE + GPIO_O_PCTL);
    ui32Val &= 0xFFFFFF00;
    ui32Val |= 0x000000FF;
    HWREG(GPIO_PORTG_BASE + GPIO_O_PCTL) = ui32Val;

    //
    // EPI0S12 ~ EPI0S13: M2 ~ 3
    //
    ui32Val = HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL);
    ui32Val &= 0xFFFF00FF;
    ui32Val |= 0x0000FF00;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) = ui32Val;

    //
    // EPI0S4 ~ EPI0S7: C4 ~ 7
    //
    ui32Val = HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL);
    ui32Val &= 0x0000FFFF;
    ui32Val |= 0xFFFF0000;
    HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = ui32Val;

    ui32Val = HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL);
    ui32Val &= 0xFFFF0FFF;
    ui32Val |= 0x0000F000;
    HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) = ui32Val;

    //
     // EPI0S00 ~ EPI0S03, EPI0S31 : K0 ~ 3, K5
     //
     ui32Val = HWREG(GPIO_PORTK_BASE + GPIO_O_PCTL);
     ui32Val &= 0xFF0F0000;
     ui32Val |= 0x00F0FFFF;
     HWREG(GPIO_PORTK_BASE + GPIO_O_PCTL) = ui32Val;

     //
     // Q0 ~ 3
     //
     ui32Val = HWREG(GPIO_PORTQ_BASE + GPIO_O_PCTL);
     ui32Val &= 0xFFFF0000;
     ui32Val |= 0x0000FFFF;
     HWREG(GPIO_PORTQ_BASE + GPIO_O_PCTL) = ui32Val;

     GPIOPinTypeEPI(GPIO_PORTA_BASE, EPI_PORTA_PINS);
     GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
     GPIOPinTypeEPI(GPIO_PORTM_BASE, EPI_PORTM_PINS);
    GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
    GPIOPinTypeEPI(GPIO_PORTK_BASE, EPI_PORTK_PINS);
    GPIOPinTypeEPI(GPIO_PORTQ_BASE, EPI_PORTQ_PINS);
    //GPIOPinTypeEPI(GPIO_PORTN_BASE, EPI_PORTN_PINS);
    GPIOPinTypeEPI(GPIO_PORTB_BASE, EPI_PORTB_PINS);

    EPIDividerSet(EPI0_BASE, 6);

    //
    // Select GPIO mode.
    //
    EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);

    EPIAddressMapSet(EPI0_BASE,EPI_ADDR_RAM_SIZE_64KB | EPI_ADDR_RAM_BASE_6 | EPI_ADDR_CODE_SIZE_64KB);

#if PANELS == 2
    EPIConfigGPModeSet(EPI0_BASE,EPI_GPMODE_CLKPIN | EPI_GPMODE_CLKGATE | EPI_GPMODE_ASIZE_12 | EPI_GPMODE_DSIZE_16,0,0);
    matrix_epi_buf = (uint16_t *)0x60000000;
#else
    EPIConfigGPModeSet(EPI0_BASE,EPI_GPMODE_CLKPIN | EPI_GPMODE_CLKGATE | EPI_GPMODE_ASIZE_12 | EPI_GPMODE_DSIZE_8,0,0);
    matrix_epi_buf = (uint8_t *)0x60000000;
#endif


}



void setup_dma_epi_ping_pong()
{


	uDMAChannelAttributeDisable(UDMA_CH21_EPI0TX,
	                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
	                                    UDMA_ATTR_HIGH_PRIORITY |
	                                    UDMA_ATTR_REQMASK);

	uDMAChannelAttributeEnable(UDMA_CH21_EPI0TX, UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_USEBURST);
	uDMAChannelAssign(UDMA_CH21_EPI0TX);

#if PANELS == 2
	uDMAChannelControlSet(UDMA_CH21_EPI0TX | UDMA_PRI_SELECT,
	                              UDMA_SIZE_16 | UDMA_SRC_INC_16 |
	                              UDMA_DST_INC_NONE |
	                              UDMA_ARB_2);
#else
	uDMAChannelControlSet(UDMA_CH21_EPI0TX | UDMA_PRI_SELECT,
	                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
	                              UDMA_DST_INC_NONE |
	                              UDMA_ARB_2);
#endif

	uDMAChannelTransferSet(UDMA_CH21_EPI0TX | UDMA_PRI_SELECT,
									UDMA_MODE_PINGPONG, (void*)&actuell_outgoing_image[0],
	                               (void *)(matrix_epi_buf),
	                               SEND_DMA_LEN);
#if PANELS == 2
	uDMAChannelControlSet(UDMA_CH21_EPI0TX | UDMA_ALT_SELECT,
	                              UDMA_SIZE_16 | UDMA_SRC_INC_16 |
	                              UDMA_DST_INC_NONE |
	                              UDMA_ARB_2);
#else
	uDMAChannelControlSet(UDMA_CH21_EPI0TX | UDMA_ALT_SELECT,
	                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
	                              UDMA_DST_INC_NONE |
	                              UDMA_ARB_2);
#endif
	uDMAChannelTransferSet(UDMA_CH21_EPI0TX | UDMA_ALT_SELECT,
									UDMA_MODE_PINGPONG, (void*)&actuell_outgoing_image[SEND_DMA_LEN],
	                               (void *)(matrix_epi_buf),
	                               SEND_DMA_LEN);



    EPIFIFOConfig(EPI0_BASE,EPI_FIFO_CONFIG_TX_1_2);

    EPIDMATxCount(EPI0_BASE,SEND_DMA_LEN);

    EPIIntEnable(EPI0_BASE,EPI_INT_DMA_TX_DONE);

	dma_counter = SEND_DMA_LEN;
	uDMAChannelEnable(UDMA_CH21_EPI0TX);
	EPI0_EISC_R = 0xFFFFFFFF;
	IntEnable(INT_EPI0);

}


#define USE_BITBAND	0

#if USE_BITBAND
//bitbanding variables
static volatile uint16_t oe_lat;
static volatile uint16_t bit_m;
#endif

uint16_t pwm_c;
//two panal
#if PANELS == 2
void make_pwm(uint8_t *image, uint16_t *result_img)
{
	uint16_t pos2=IMAGE_SIZE/4;
	uint16_t pos3=IMAGE_SIZE/2;
	uint16_t pos4=(IMAGE_SIZE/4)*3;
#else
void make_pwm(uint8_t *image, uint8_t *result_img)
{
	uint16_t pos2=IMAGE_SIZE/2;
#endif

	uint8_t a;
	uint16_t pos1=0;
	uint8_t i;

#if USE_BITBAND
#else
	uint16_t oe_lat;
#endif



	for(a = 0;a<8;a++) {

		for(i=0;i<32;i++) {
			uint8_t r1;
			uint8_t g1;
			uint8_t b1;
			uint8_t r2;
			uint8_t g2;
			uint8_t b2;




#if PANELS == 2
			uint8_t r3;
			uint8_t g3;
			uint8_t b3;
			uint8_t r4;
			uint8_t g4;
			uint8_t b4;

			r1=gamma(image[pos1]);
			pos1++;
			g1=gamma(image[pos1]);
			pos1++;
			b1=gamma(image[pos1]);
			pos1++;
			r2=gamma(image[pos2]);
			pos2++;
			g2=gamma(image[pos2]);
			pos2++;
			b2=gamma(image[pos2]);
			pos2++;

			r3=gamma(image[pos3]);
			pos3++;
			g3=gamma(image[pos3]);
			pos3++;
			b3=gamma(image[pos3]);
			pos3++;
			r4=gamma(image[pos4]);
			pos4++;
			g4=gamma(image[pos4]);
			pos4++;
			b4=gamma(image[pos4]);
			pos4++;
#else
			//TODO: add gamme ?
			r1=image[pos1];
			pos1++;
			g1=image[pos1];
			pos1++;
			b1=image[pos1];
			pos1++;
			r2=image[pos2];
			pos2++;
			g2=image[pos2];
			pos2++;
			b2=image[pos2];
			pos2++;
#endif

			if(i==31 || i<=GLOBAL_LINEAR_DIM)
			{
				//oe_lat |= 0x0040;

				//bitband
				HWREGBITW(&oe_lat, 6) = 1;
			}
			else {
				//oe_lat &= ~0x0040;

				//bitband
				HWREGBITW(&oe_lat, 6) = 0;
			}

			if(i==31)
			{
				//oe_lat |= 0x0080;

				//bitband
				HWREGBITW(&oe_lat, 7) = 1;
			}
			else {
				//oe_lat &= ~0x0080;

				//bitband
				HWREGBITW(&oe_lat, 7) = 0;
			}

			uint16_t  xxx = (uint16_t)i + (uint16_t)a*32*MAX_IN_BUF;



#if USE_BITBAND
			bit_m = (uint16_t)0x3F3F | oe_lat;
#else
			uint16_t bit_m = (uint16_t)0x3F3F | oe_lat;
#endif


			for(pwm_c=0;pwm_c<MAX_IN_BUF;pwm_c++)
			{
				//first panel
				if(!(pwm_c ^ r1)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 0) = 0;
#else
					bit_m &= ~0x0001;
#endif

				}
				if(!(pwm_c ^ g1)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 1) = 0;
#else
					bit_m &= ~0x0002;
#endif
				}
				if(!(pwm_c ^ b1)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 2) = 0;
#else
					bit_m &= ~0x0004;
#endif
				}
				if(!(pwm_c ^ r2)){
#if USE_BITBAND
					HWREGBITW(&bit_m, 3) = 0;
#else
					bit_m &= ~0x0008;
#endif
				}
				if(!(pwm_c ^ g2)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 4) = 0;
#else
					bit_m &= ~0x0010;
#endif
				}
				if(!(pwm_c ^ b2)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 5) = 0;
#else
					bit_m &= ~0x0020;
#endif
				}

//#if PANELS == 2
				//second panel
				if(!(pwm_c ^ r3)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 8) = 0;
#else
					bit_m &= ~0x0100;
#endif
				}
				if(!(pwm_c ^ g3)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 9) = 0;
#else
					bit_m &= ~0x0200;
#endif
				}
				if(!(pwm_c ^ b3)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 10) = 0;
#else
					bit_m &= ~0x0400;
#endif
				}
				if(!(pwm_c ^ r4)){
#if USE_BITBAND
					HWREGBITW(&bit_m, 11) = 0;
#else
					bit_m &= ~0x0800;
#endif
				}
				if(!(pwm_c ^ g4)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 12) = 0;
#else
					bit_m &= ~0x1000;
#endif
				}
				if(!(pwm_c ^ b4)) {
#if USE_BITBAND
					HWREGBITW(&bit_m, 13) = 0;
#else
					bit_m &= ~0x2000;
#endif
				}
//#endif


				result_img[pwm_c*32 + xxx] =  bit_m;

			}

		}
	}
}


void ExternalBusInterface(void)
{
	static int address_counter = 0;

	uint32_t mode = uDMAChannelModeGet(UDMA_CH21_EPI0TX | UDMA_PRI_SELECT);

	EPIDMATxCount(EPI0_BASE,SEND_DMA_LEN);

	if(mode == UDMA_MODE_STOP)
	{



		uDMAChannelTransferSet(UDMA_CH21_EPI0TX | UDMA_PRI_SELECT,
											   UDMA_MODE_PINGPONG, (void*)&actuell_outgoing_image[dma_counter],
											   (void *)(&matrix_epi_buf[address_counter]),
											   SEND_DMA_LEN);


		if(EPI0_DMATXCNT_R== 0)
			EPIDMATxCount(EPI0_BASE,SEND_DMA_LEN);


		dma_counter += SEND_DMA_LEN;


#if PANELS != 2
		address_counter = (dma_counter / 256);

		if(dma_counter == 0) {
			GPIO_PORTN_DATA_R ^= 0x01;
		}
#else

		if(dma_counter >= (256*MAX_IN_BUF)) {
			GPIO_PORTN_DATA_R ^= 0x01;
			dma_counter = 0;
		}

		if(dma_counter == 0)
			address_counter = 0;
		else
			address_counter = (dma_counter / MAX_IN_BUF);
#endif

	}

	mode = uDMAChannelModeGet(UDMA_CH21_EPI0TX | UDMA_ALT_SELECT);

	if(mode == UDMA_MODE_STOP)
	{


		uDMAChannelTransferSet(UDMA_CH21_EPI0TX | UDMA_ALT_SELECT,

												UDMA_MODE_PINGPONG, (void*)&actuell_outgoing_image[dma_counter],
											   (void *)(&matrix_epi_buf[address_counter]),
											   SEND_DMA_LEN);
		//EPIDMATxCount(EPI0_BASE,SEND_DMA_LEN);
		if(EPI0_DMATXCNT_R==0)
			EPIDMATxCount(EPI0_BASE,SEND_DMA_LEN);

		dma_counter += SEND_DMA_LEN;



#if PANELS != 2
		address_counter = (dma_counter / 256);

		if(dma_counter == 0) {
			GPIO_PORTN_DATA_R ^= 0x01;
		}
#else


		if(dma_counter >= (256*MAX_IN_BUF)) {
			GPIO_PORTN_DATA_R ^= 0x01;
			dma_counter = 0;
		}

		if(dma_counter == 0)
			address_counter = 0;
		else
			address_counter = (dma_counter / MAX_IN_BUF);
#endif
	}

    // Clear the requested interrupt sources.
    //
	EPI0_EISC_R = 0x00000010;
	//EPI0_EISC_R = 0xFFFFFFFF;

}

volatile uint32_t mode;



void MyTimerInterrupt1B(void)
{

	static int timer1a_count = 0;
	static int flipflop = 1;

	int temp_count = TimerValueGet(TIMER1_BASE,TIMER_A);

	if(temp_count == timer1a_count)
	{
		if(flipflop) {
			g_timer_ready = -1;
			flipflop = 0;
			//GPIO_PORTN_DATA_R ^= 0x01;

			SSIDisable(SSI2_BASE);

			uDMAChannelDisable(UDMA_CH12_SSI2RX);

			uDMAChannelControlSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
													  UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
													  UDMA_DST_INC_8 |
													  UDMA_ARB_1);

			if(actuell_income_image == incom_image_A)
			{
				actuell_income_image = incom_image_B;
			} else {
				actuell_income_image = incom_image_A;
			}

			uDMAChannelTransferSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
					UDMA_MODE_BASIC, (void*)(SSI2_BASE + 0x00000008),
											(void*)&actuell_income_image[0],
										   768);

			actuell_income_pos = 768;

			SSIEnable(SSI2_BASE);

			uDMAChannelEnable(UDMA_CH12_SSI2RX);
		}
	} else {
		flipflop = 1;
	}

	timer1a_count = temp_count;


    TimerIntClear(TIMER1_BASE,TIMER_TIMB_TIMEOUT);
}

//*****************************************************************************
//
// Interrupt handler for SSI2 peripheral in slave mode.  It reads the interrupt
// status and if the interrupt is fired by a RX time out interrupt it reads the
// SSI2 RX FIFO and increments a counter to tell the main loop that RX timeout
// interrupt was fired.
//
//*****************************************************************************
void
SSI2IntHandler(void)
{
	uint32_t ulStatus;

	//
	// Read interrupt status.
	//
	ulStatus = SSIIntStatus(SSI2_BASE, 1);

	//
	// Check the reason for the interrupt.
	//
	if(ulStatus & 0x00000010)
	{

#if PANELS == 2
		if(actuell_income_pos == 768 || actuell_income_pos == 1536 || actuell_income_pos == 2304)
#else
		if(actuell_income_pos == 768)
#endif
		{

			uDMAChannelTransferSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
					UDMA_MODE_BASIC, (void*)(SSI2_BASE + 0x00000008),
			                               (void*)&actuell_income_image[actuell_income_pos],
			                               768);
			actuell_income_pos += 768;
			uDMAChannelEnable(UDMA_CH12_SSI2RX);
		}
		else if(actuell_income_pos == 0) {
			//do this stuff in the timer interrupt

			actuell_income_pos = 768;

			uDMAChannelEnable(UDMA_CH12_SSI2RX);
		}

		else {

			uDMAChannelControlSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
			                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
			                              UDMA_DST_INC_NONE |
			                              UDMA_ARB_1);

			uDMAChannelTransferSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
								UDMA_MODE_BASIC, (void*)(SSI2_BASE + 0x00000008),
												(void*)(SSI3_BASE + 0x00000008),
						                               768);
			uDMAChannelEnable(UDMA_CH12_SSI2RX);
		}


	}

	//
	// Clear interrupts.
	//
	SSIIntClear(SSI2_BASE, ulStatus);
}

void setup_ssi2(void)
{
    //
    // The SSI2 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    //GPIOPinConfigure(GPIO_PD2_SSI2FSS);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);///TX
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);//RX


    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_1 |
                   GPIO_PIN_0);

    SSIClockSourceSet(SSI2_BASE,SSI_CLOCK_SYSTEM);

    //
    // Configure and enable the SSI2 port for SPI slave mode.
    //
    SSIConfigSetExpClk(SSI2_BASE, 120000000, SSI_FRF_MOTO_MODE_0,
    				   SSI_MODE_SLAVE, 5000000*12, 8);

    //
    // Enable the SSI2 module.
    //
    SSIEnable(SSI2_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    uint32_t dataaa;
    while(SSIDataGetNonBlocking(SSI2_BASE, &dataaa))
    {
    }

    //
    // Clear any pending interrupt
    //
    SSIIntClear(SSI2_BASE, SSI_RXTO);

    actuell_income_image = &incom_image_A[0];

    setup_ssi3();

}

void setup_ssi3(void)
{
    //
    // The SSI3 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);


    GPIOPinConfigure(GPIO_PF3_SSI3CLK);
    //GPIOPinConfigure(GPIO_PD2_SSI2FSS);
    GPIOPinConfigure(GPIO_PF1_SSI3XDAT0);///TX
    GPIOPinConfigure(GPIO_PF0_SSI3XDAT1);//RX

    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1 |
                   GPIO_PIN_0);

    SSIClockSourceSet(SSI3_BASE,SSI_CLOCK_SYSTEM);

    //
    // Configure and enable the SSI3 port for SPI master mode.
    //
    SSIConfigSetExpClk(SSI3_BASE, 120000000, SSI_FRF_MOTO_MODE_0,
    				   SSI_MODE_MASTER, 5000000, 8);

    //
    // Enable the SSI3 module.
    //
    SSIEnable(SSI3_BASE);

    //
    // Enable RX timeout interrupt.
    //
    //SSIIntEnable(SSI2_BASE, SSI_RXTO);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    uint32_t dataaa;
    while(SSIDataGetNonBlocking(SSI3_BASE, &dataaa))
    {
    }

}

void setup_dma_ssi2()
{


	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);


	IntEnable(INT_UDMAERR);

	uDMAEnable();

    // Point at the control table to use for channel control structures.
    //
    memset(pui8ControlTable,0x00,sizeof(pui8ControlTable));

	uDMAControlBaseSet(pui8ControlTable);

///////////////////A

	uDMAChannelAttributeDisable(UDMA_CH12_SSI2RX,
	                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
	                                    UDMA_ATTR_HIGH_PRIORITY |
	                                    UDMA_ATTR_REQMASK);

	//uDMAChannelAttributeEnable(UDMA_CH12_SSI2RX, UDMA_ATTR_USEBURST);
	uDMAChannelAssign(UDMA_CH12_SSI2RX);

	uDMAChannelControlSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
	                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
	                              UDMA_DST_INC_8 |
	                              UDMA_ARB_1);

	uDMAChannelTransferSet(UDMA_CH12_SSI2RX | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC, (void*)(SSI2_BASE + 0x00000008),
	                               incom_image_A,
	                               768);

	actuell_income_pos = 768;

	//enable DMA RX interrupt
	SSI2_IM_R |= 0x00000010;


    //
    // Clear any pending interrupt
    //
    SSIIntClear(SSI2_BASE, 0x00000010);

	SSIDMAEnable(SSI2_BASE,SSI_DMA_RX);

	IntEnable(INT_SSI2);

	uDMAChannelEnable(UDMA_CH12_SSI2RX);

}

void setup_timer_cap()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinConfigure(GPIO_PD2_T1CCP0);

	GPIOPinTypeTimer(GPIO_PORTD_BASE,GPIO_PIN_2);

	//
	// Configure the two 32-bit periodic timers.
	//
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP | TIMER_CFG_B_PERIODIC_UP);

	TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
	TimerLoadSet(TIMER1_BASE, TIMER_B, 30000);


	TimerIntEnable(TIMER1_BASE,TIMER_TIMB_TIMEOUT);
	// Setup the interrupts for the timer timeouts.
	//
	IntEnable(INT_TIMER1B);

	//
	// Enable the timers.
	//
	TimerEnable(TIMER1_BASE, TIMER_BOTH);

}
