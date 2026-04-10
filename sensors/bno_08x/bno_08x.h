/*
    bno_08x.h
    Author: Diego
    Created on: 10, April 2026
 */

#ifndef BNO_08X_H_
#define BNO_08X_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#endif

status_t bno_08x_init(bool dma_enable, void* callback);

#endif /* BNO_08X_H_ */