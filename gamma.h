/*
 * gamma.h
 *
 *  Created on: 23.04.2014
 *      Author: Sebastian Förster
 */

#ifndef GAMMA_H_
#define GAMMA_H_

#include <stdint.h>

#define gamma(x) 	(gamma_256_to_224_g22[x])

extern uint8_t gamma_256_to_224[256];
extern uint8_t gamma_256_to_224_g22[256];

#endif /* GAMMA_H_ */
