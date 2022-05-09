#include "LPC17xx.h"
#include "core_cm3.h"
#include <stdlib.h>

#define SSP_RNE (1<<2)
#define SSP_BSY (1<<4)

void spi_init();
void spi_txrx(uint8_t* tx, uint8_t* rx, uint16_t len);

int main(void)
{
	int data_in = 0;
	spi_init();
	LPC_SSP0->CR0 |= 0xF; //16 bit transfer, SPI format, clock is low between frames. 
	LPC_SSP0->CR1 |= (1<<1); //SSP enabled in master mode.
	LPC_SSP0->CPSR = 4; //Clock is prescaled by 4 (96MHz/4 = 24MHz)

	LPC_SPI->SPCR |= (1 << 2); // 1 on bit 2, allows to send 16 bits data / 0 is to send 8 bits data
	LPC_SPI->SPCR &= ~(15 << 8); // 16 bits transfer (0000 to bit 11 to 8)
	LPC_SPI->SPCR |= (1 << 5); // 1 on bit 5, enable master mode for LPC17
	
	

    while(1){

        LPC_SSP0->DR = 0x0505;
        //while ( (LPC_SSP0->SR & ((SSP_SR_BSY)|SSP_SR_RNE)) != SSP_SR_RNE );
        data_in = LPC_SSP0->DR;

    }
	
	return 0;
}


void spi_init() {
	//notre test en SPI
	// Power SPI
	LPC_SC->PCONP |= (1 << 8);
	//clk config
	LPC_SC->PCLKSEL0 &= ~(3 << 16);
	LPC_SC->PCLKSEL0 |= (1 << 16);
	
	
	//config des pins 
	//P0.15 SCK0 -> clock
	//P0.16 SSEL0 ou SS -> slave select
	//P0.17 MISO0 -> master imput slave output
	//P0.18 MOSI0 -> master output slave input
	LPC_PINCON->PINSEL0 &= ~(3 << 30); //P0.15
	LPC_PINCON->PINSEL0 |= (1 << 31); //P0.15 function SCK0
	
	LPC_PINCON->PINSEL1 &= ~(3 | (3 << 2) | (3 << 4));
  LPC_PINCON->PINSEL1 |= 2 | (2 << 2) | (2 << 4);
	LPC_PINCON->PINMODE0 |= (3<<30);//P0.15
	LPC_PINCON->PINMODE1 |= (3<<0)|(3<<2)|(3<<4); //Turn on pull-downs on respective pins.

	
	//interruption pour le SPI
	//NVIC_EnableIRQ(SPI_IRQn);
	
	
//  // Power SSP0
//  LPC_SC->PCONP |= (1 << 21);

//  // Peripheral clock - select undivided clock for SSP0
//  LPC_SC->PCLKSEL1 &= ~(1 << 10); // essayer avec 10
//	LPC_SC->PCLKSEL1 &= ~(1 << 11); 
//	//LPC_SC->PCLKSEL1 |= (1 << 10); 

//  // Select pin functions
//  //   P0.15 as SCK0 (2 at 31:30)
//  LPC_PINCON->PINSEL0 &= ~(3 << 30);
//  LPC_PINCON->PINSEL0 |= (2 << 30);

//  //   P0.16 as SSEL0 (2 at 1:0)
//  //   P0.17 as MISO0 (2 at 3:2)
//  //   P0.18 as MOSI0 (2 at 5:4)
//  LPC_PINCON->PINSEL1 &= ~(3 | (3 << 2) | (3 << 4));
//  LPC_PINCON->PINSEL1 |= 2 | (2 << 2) | (2 << 4);

//  // SSP0 Control Register 0
//  //   8-bit transfers (7 at 3:0)
//  //   SPI (0 at 5:4)
//  //   Polarity and Phase default to Mode 0
//  LPC_SSP0->CR0 = 7;

//  // SSP0 Prescaler
//  // The SD spec requires a slow start at 200khz
//  LPC_SSP0->CPSR = SystemCoreClock / 200000;

//  // SPI Control Register 1
//  //   Defaults to Master
//  //   Start serial communications (bit 1)
//  LPC_SSP0->CR1 |= (1 << 1);
}

void spi_txrx(uint8_t* tx, uint8_t* rx, uint16_t len)
{
	/* Embed: transmit and receive len bytes
	 * Remember:
	 *   SPI transmits and receives at the same time
	 *   If tx == NULL and you are only receiving then transmit all 0xFF
	 *   If rx == NULL and you are only transmitting then dump all recieved bytes
	 */
  volatile uint_fast8_t dummy;
  while (len--) {
    if (tx == NULL) {
      LPC_SSP0->DR = 0xff;
    } else {
      LPC_SSP0->DR = *tx++;
    }

    while ( (LPC_SSP0->SR & (SSP_BSY | SSP_RNE)) != SSP_RNE );

    if (rx == NULL) {
      dummy = LPC_SSP0->DR;
    } else {
      *rx++ = LPC_SSP0->DR;
    }
  }
}