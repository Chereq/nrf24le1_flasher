#include "main.h"

#ifndef SPI_H
#define SPI_H

#ifdef FTD2XX_LIB
int spi_begin(uint8_t ftdevice);
#else  // FTD2XX_LIB
int spi_begin(uint8_t bus, uint8_t port);
#endif // FTD2XX_LIB
void spi_end();
int spi_transfer(uint8_t *bytes, size_t size);


#endif // SPI_H
