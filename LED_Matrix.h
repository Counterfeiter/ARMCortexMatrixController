/*
 * LED_Matrix.h
 *
 *  Created on: 02.04.2014
 *      Author: Sebastian Förster
 */

#ifndef LED_MATRIX_H_
#define LED_MATRIX_H_

#include <stdint.h>


//1 or 2
#define PANELS					2
//default 0 => No brigthnes control 30 -> BLACK
#define GLOBAL_LINEAR_DIM		0


#define IMAGE_SIZE				512*3*PANELS




//8 Bit Image A and B
#if PANELS == 2
	#define MAX_IN_BUF			(256-32)
	extern uint16_t image_A[256][MAX_IN_BUF];
	extern uint16_t image_B[256][MAX_IN_BUF];

	extern void make_pwm(uint8_t *image, uint16_t *result_img);
	extern volatile uint16_t *actuell_outgoing_image;
#else
	#define MAX_IN_BUF			256
	extern uint8_t image_A[256][MAX_IN_BUF];
	extern uint8_t image_B[256][MAX_IN_BUF];

	extern void make_pwm(uint8_t *image, uint8_t *result_img);
	extern volatile uint8_t *actuell_outgoing_image;
#endif

//8 Bit orginal image
extern uint8_t incom_image_A[IMAGE_SIZE];
extern uint8_t incom_image_B[IMAGE_SIZE];

extern volatile uint8_t *actuell_income_image;
extern volatile int actuell_income_pos;
extern volatile int g_timer_ready;
extern volatile uint16_t dma_counter;

extern void setup_epi(void);
extern void setup_ssi2();
extern void setup_timer_cap();
extern void setup_dma_ssi2();
extern void setup_dma_epi_ping_pong();

//extern volatile uint8_t *matrix_epi_buf;




#endif /* LED_MATRIX_H_ */
