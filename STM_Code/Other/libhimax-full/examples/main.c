#include <msp430.h>
#include <stdlib.h>
#include <libio/console.h>
#include <libmsp/mem.h>
#include <libmsp/periph.h>
#include <libmsp/clock.h>
#include <libmsp/watchdog.h>
#include <libmsp/gpio.h>

#include "i2c.h"
#include "hm01b0.h"
#include "hm01b0_reg.h"

extern uint8_t frame[];

void mcu_init() {
  WDTCTL = WDTPW + WDTHOLD;   // Disable watchdog
  
	FRCTL0 = FRCTLPW + NWAITS_1;

	CSCTL0_H = CSKEY_H; 										// Unlock CS registers

	CSCTL1 = DCOFSEL_0; 										// Initial Clock Frequency Reset 

	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; 		// Set Clock Sources ACLK = SMCLK = MCLK = DCO

	// As per datasheet, div/4 for preventing out 
	// of spec operation. Refer to example codes.
	CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4; 
	CSCTL1 = DCOFSEL_4 | DCORSEL; 								// Frequency = 16MHz

	__delay_cycles(60);

	// Set Dividers to 1
	CSCTL3 = DIVA__1 | DIVS__4 | DIVM__1;						// ACLK = MCLK / 4 -- MCLK = DCO

	// Turn on VLO
	CSCTL4 &= ~VLOOFF;
	
	CSCTL0_H = 0;
	
}
   
int main(){
	
	msp_watchdog_disable();
	msp_gpio_unlock();
	//msp_clock_setup();
	mcu_init();
	INIT_CONSOLE();

	uint16_t id = 0, j;
	uint8_t i;

	P8OUT &= ~(BIT1 | BIT2); 
	P8DIR |= BIT1 + BIT2;
	
	PRINTF("Starting....\n\r");

	hm01b0_init();

	id = hm01b0_reg_default_init();

	PRINTF("Camera ID 0x%04x\n\r", id);	

	HM01B0 cam = {0};

	cam.mode = Streaming3;		// External trigger
	cam.state = sleep;			// Camera inited but to be configured
	cam.resolution = QQVGA;		// QQVGA resolution
	cam.dataDepth = EightB;		// 8-bit per pixel
	cam.dataIo = EightL;		// 8 Data lines
	//cam.tPattern = 1;			// Test Pattern

	hm01b0_capture(&cam);

	for(j = 0; j < cam.pixels; j++){

		PRINTF("%i ", frame[j]);

	}
}

