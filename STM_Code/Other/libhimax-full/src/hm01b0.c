#include <msp430.h>
#include <stdint.h>
#include <string.h>

#include "hm01b0.h"

#define enable_debug

#ifdef enable_debug
	#include <libio/console.h>
#endif

uint8_t __attribute__((section(".upper.rodata"))) frame[19200] = {0};
uint8_t __attribute__((section(".lower.data"))) RAMBuffer[160];

uint8_t frame_numeber = 0;

void hm01b0_enable(){

	CAM_VCC_OUT |= CAM_VCC_EN;
}

void hm01b0_disable(){

	CAM_VCC_OUT &= ~CAM_VCC_EN;
}

void hm01b0_reg_write(uint16_t addr, uint8_t val) {

	I2C_Master_WriteReg(HIMAX_I2C_ADDR, addr, &val, sizeof(val));
	__delay_cycles(1300);
}


uint8_t hm01b0_reg_read(uint16_t addr) {

	I2C_Master_ReadReg(HIMAX_I2C_ADDR, addr, 1);
	__delay_cycles(1100);

	return ReceiveBuffer[0];
}

uint8_t hm01b0_init(void) {

#ifdef enable_debug
	PRINTF("Camera: GPIO configuration\n\r");
#endif

	//Data pins
	DATA_SEL0 = 0;										//Camera data input
	DATA_SEL1 = 0;										//GPIO Mode
	DATA_DIR  = 0;										//Input

	CAM_VCC_SEL0 &= ~CAM_VCC_EN;
	CAM_VCC_SEL1 &= ~CAM_VCC_EN;
	CAM_VCC_OUT &= ~CAM_VCC_EN;
	CAM_VCC_DIR |= CAM_VCC_EN;

	//Camera Clock
	M_CLK_SEL0 |= M_CLK;								//Camera clock
	M_CLK_SEL1 &= ~(M_CLK);								//GPIO Mode
	M_CLK_DIR  |= M_CLK;								//Output

	//Sync signals
	SYNC_SEL0 &= ~(PCLKO | LVLD | FVLD);				//Camera pixel clock | Frame valid | Line Valid
	SYNC_SEL1 &= ~(PCLKO | LVLD | FVLD);				//GPIO Mode
	SYNC_DIR  &= ~(PCLKO | LVLD | FVLD);				//Input

	//Triggers
	TRG_OUT &= ~(TRIG);									//Low trigger signal
	TRG_SEL0 &= ~(INTR | TRIG);							//Interrupt from the camera and trigger to the camera
	TRG_SEL1 &= ~(INTR | TRIG);							//GPIO Mode
	TRG_DIR &= ~(INTR);									//Input
	TRG_DIR |= TRIG;									//Output


#ifdef enable_debug
	PRINTF("I2C: init\n\r");
#endif

	//I2C init
	initI2C(HIMAX_I2C_ADDR);

#ifdef enable_debug
	PRINTF("Camera: interface initialized\n\r");
#endif

	return 1;
}

uint8_t hm01b0_deinit() {

	//Deactivate clock
	M_CLK_SEL0 &= ~M_CLK;								//Camera clock
	M_CLK_SEL1 &= ~M_CLK;								//GPIO Mode
	M_CLK_OUT &= ~M_CLK;								//Output Low

	//Set sync pin as output
	SYNC_DIR  |= (PCLKO | LVLD | FVLD);					//Output
	SYNC_OUT  &= ~(PCLKO | LVLD | FVLD);				//Low

	//INTR pin as output
	TRG_DIR |= (INTR);									//Output
	TRG_OUT	&= ~(INTR);									//Low

	return 0;
}


uint16_t hm01b0_reg_default_init() {

	uint8_t PID_L = 0, PID_H = 0;
	uint16_t PID = 0;

	while (PID != 0x01B0) {

		hm01b0_rst();
		__delay_cycles(160000);

		PID_H = hm01b0_reg_read(MODEL_ID_H);
		PID_L = hm01b0_reg_read(MODEL_ID_L);

		PID = ((PID_H << 8) | PID_L);
	}

#ifdef enable_debug
	PRINTF("Camera: Register default config\n\r");
#endif

	// Clock configuration
	hm01b0_reg_write(OSC_CLK_DIV, 0x38); 						// Gated pclock, MSB first, main clock 8x div, reg clock 1x div

	// Motion detection
	hm01b0_reg_write(MD_CTRL, 0x30);                    		// Motion detection control -- Disable

	// QVGA windows
	hm01b0_reg_write(QVGA_WIN_EN, 0x01);   						// Enable qvga timing

	//Sync signal advance
	//hm01b0_reg_write(VSYNC_HSYNC_PIXEL_SHIFT_EN, 0x00); 		// Sync shift disable
	//hm01b0_reg_write(ADVANCE_HSYNC, 0x25);					// Advance LVLD by 2 Pixel Time
	hm01b0_reg_write(ADVANCE_VSYNC, 0x01);						// Advance LVLD by 2 Pixel Time

	// Exposure integration time start value 					// Optimized for bright outdoor 
	hm01b0_reg_write(INTEGRATION_H, 0x00);						// Pixels line lenght High bits [Def: 0x01]
	hm01b0_reg_write(INTEGRATION_L, 0x20);						// Pixels line lenght Low bits  [Def: 0x01]

	// Flicker step control
	hm01b0_reg_write(FS_CTRL, 0x00);                    		// Flicker Control -- Disable

	hm01b0_reg_write(AE_TARGET_MEAN, 0x39);						// Auto exp target value [Def: 0x3C]
	hm01b0_reg_write(AE_MIN_MEAN, 0x07);						// Auto exp target value [Def: 0x0A]

	hm01b0_reg_write(DIGITAL_GAIN_H, 0x03);						// Digital Gain High [Def: 0x01]
	hm01b0_reg_write(DIGITAL_GAIN_L, 0x96);						// Digital Gain Low [Def: 0x00]

	hm01b0_reg_write(MAX_DGAIN,	0xF0);							//	Max DGAIN [Def: 0xC0]
	hm01b0_reg_write(MIN_DGAIN, 0x30);							//	Min DGAIN [Def: 0x40]

	// Black target level
	//hm01b0_reg_write(BLC_TGT, 0x02);                    		//  BLC target :2  at 8 bit mode -- Black level target
	//hm01b0_reg_write(BLC2_TGT, 0x02);                   		//  BLI target :2  at 8 bit mode -- Black level target
	//hm01b0_reg_write(BLI_EN, 0x01);                     		//  BLI enable

	// Streaming mode 2 frames number
	hm01b0_reg_write(PMU_AUTOSLEEP_FRAMECNT, 0x01);				// Provides only 1 frame

#ifdef enable_debug
	PRINTF("Camera: configured\n\r");
#endif

	return PID;
}

void hm01b0_rst() {
#ifdef enable_debug
	PRINTF("Camera: reset\n\r");
#endif
	hm01b0_reg_write(SW_RESET, HIMAX_RESET);
}

uint8_t hm01b0_set_mode(uint8_t mode) {

	switch (mode) {
 
	case 0://Standby
#ifdef enable_debug
	PRINTF("Camera: Standby\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x00);
		break;

	case 1://Streaming mode 1
#ifdef enable_debug
	PRINTF("Camera: Streaming mode 1\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x01);
		break;

	case 2://Streaming mode 2
#ifdef enable_debug
	PRINTF("Camera: Streaming mode 2\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x03);
		break;

	case 3://Streaming mode 3
#ifdef enable_debug
	PRINTF("Camera: Streaming mode 3\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x05);
		break;

	default://Standby
#ifdef enable_debug
	PRINTF("Camera: Standby\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x00);

	}

	return mode;
}

uint8_t hm01b0_set_output(uint8_t outM) {

	switch (outM) {

	case 0://8 bit
#ifdef enable_debug
	PRINTF("Camera: 8bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x70);
		break;

	case 1://6 bit
#ifdef enable_debug
	PRINTF("Camera: 6bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x71);
		break;

	default://8 bit
#ifdef enable_debug
	PRINTF("Camera: 8bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x70);

	}

	return outM;
}

uint8_t hm01b0_set_interface(uint8_t io) {

	switch (io) {

	case 0://8-bit IO
#ifdef enable_debug
	PRINTF("Camera: 8bit IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x02);
		break;

	case 1://4-bit IO
#ifdef enable_debug
	PRINTF("Camera: 4bit IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x12);
		break;

	case 2://Serial IO
#ifdef enable_debug
	PRINTF("Camera: Serial IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x22);
		break;

	default: //8-bit IO
#ifdef enable_debug
	PRINTF("Camera: 8bit IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x02);

	}

	return io;
}

uint8_t hm01b0_set_resolution(HM01B0 *cam) {

	switch (cam->resolution) {

	case 0:
		//QVGA binning
#ifdef enable_debug
	PRINTF("Camera: QVGA\n\r");
#endif
		hm01b0_reg_write(READOUT_X, 0x01);   					// disable horizontal binning timing
		hm01b0_reg_write(READOUT_Y, 0x01);   					// disable vertical binning timing
		hm01b0_reg_write(BINNING_MODE, 0x00);   				// disable v and h binning

		cam->raw_W = 326;										// Raw camera orizontal pixel number
		cam->raw_H = 244;										// Raw camera vertical pixel number

		cam->W = 320;											// Picture orizontal pixel number
		cam->H = 240;											// Picture vertical pixel number

		break;

	case 1:
		//QQVGA binning
#ifdef enable_debug
	PRINTF("Camera: QQVGA\n\r");
#endif
		hm01b0_reg_write(READOUT_X, 0x03);   				// horizontal binning timing
		hm01b0_reg_write(READOUT_Y,	0x03);   				// vertical binning timing
		hm01b0_reg_write(BINNING_MODE, 0x03);   			// enable v and h binning

		// Frame rate control
		hm01b0_reg_write(LINE_LEN_PCK_H, 0x00);				// Pixels line lenght High bits [Def: 0x01]
		hm01b0_reg_write(LINE_LEN_PCK_L, 0xD7);				// Pixels line lenght Low bits  [Def: 0x01]
		hm01b0_reg_write(FRAME_LEN_LINES_H, 0x00);			// Frame line lenght High bits  [Def: 0x02]
		hm01b0_reg_write(FRAME_LEN_LINES_L, 0x80);			// Frame line lenght Low bits   [Def: 0x32]

		cam->raw_W = 164;									// Raw camera orizontal pixel number
		cam->raw_H = 122;									// Raw camera vertical pixel number

		cam->W = 160;										// Picture orizontal pixel number
		cam->H = 120;										// Picture vertical pixel number

		break;

	default:
		//QQVGA binning
#ifdef enable_debug
	PRINTF("Camera: QQVGA\n\r");
#endif
		hm01b0_reg_write(READOUT_X, 0x03);   				// horizontal binning timing
		hm01b0_reg_write(READOUT_Y,	0x03);   				// vertical binning timing
		hm01b0_reg_write(BINNING_MODE, 0x03);   			// enable v and h binning

		// Frame rate control
		hm01b0_reg_write(LINE_LEN_PCK_H, 0x00);				// Pixels line lenght High bits [Def: 0x01]
		hm01b0_reg_write(LINE_LEN_PCK_L, 0xD7);				// Pixels line lenght Low bits  [Def: 0x01]
		hm01b0_reg_write(FRAME_LEN_LINES_H, 0x00);			// Frame line lenght High bits  [Def: 0x02]
		hm01b0_reg_write(FRAME_LEN_LINES_L, 0x80);			// Frame line lenght Low bits   [Def: 0x32]

		cam->raw_W = 164;									// Raw camera orizontal pixel number
		cam->raw_H = 122;									// Raw camera vertical pixel number

		cam->W = 160;										// Picture orizontal pixel number
		cam->H = 120;										// Picture vertical pixel number
	}

	return cam->resolution;
}

uint8_t hm01b0_test_pattern(uint8_t pattern) {

	switch (pattern) {

	case 0://Disabled
#ifdef enable_debug
	PRINTF("Camera: Test pattern Disabled\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x00);
		break;

	case 1: // walking 1's
#ifdef enable_debug
	PRINTF("Camera: Test pattern walking 1's\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x11);
		break;

	case 2://color bars
#ifdef enable_debug
	PRINTF("Camera: Test pattern color bars\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x01);
		break;

	default://Disabled
#ifdef enable_debug
	PRINTF("Camera: Test pattern Disabled\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x00);
	}

	return pattern;
}

/*
 *	Currently implemented:
 *		QQVGA
 *		8-Bit data depth -- 8 data line
 *		Streaming mode 1, 2 and 3
 *		Single frame
 *
 */
void hm01b0_capture(HM01B0 *cam) {

	P8OUT &= ~BIT1;
	//P8OUT |= BIT1;

	uint16_t raw_W = 0, raw_H = 0, W = 0, H = 0, i = 0, j = 0;
	uint32_t tot = 0, max_pixels = 0;

	frame_numeber = 0;

	SYNC_IES &= ~FVLD;							// FVLD Lo/Hi edge
	SYNC_IFG &= ~FVLD;                         	// Clear FVLD interrupt flags

#ifdef enable_debug
  	PRINTF("Camera: Index address %u\n\r", &i);
  	PRINTF("Camera: Frame address %u\n\r", &frame[0]);
#endif

	if (cam->state == sleep) {

		hm01b0_set_resolution(cam);
		hm01b0_set_interface(cam->dataIo);
		hm01b0_set_output(cam->dataDepth);

		hm01b0_reg_write(GRP_PARAM_HOLD, 0x01);						//Group parameter hold -- Command Update

		cam->state = idle;
	}

	if (cam->tPattern != 0)
		hm01b0_test_pattern(cam->tPattern);

	//Store variables locally to reduce latency
	raw_W = cam->raw_W;
	raw_H = cam->raw_H;
	W = cam->W;
	H = cam->H;
	max_pixels = (uint32_t)(raw_W * raw_H);

	hm01b0_set_mode(cam->mode);

#ifdef enable_debug
    PRINTF("Camera: AEC...\n\r");
#endif

	if (cam->mode == 3)
		TRG_OUT |= TRIG;

	SYNC_IE |= FVLD;                      							// FVLD interrupt enabled
	__bis_SR_register(LPM0_bits + GIE);  							// Enter sleep mode for AEC calibration (~9 Cyles)

#ifdef enable_debug
    PRINTF("Camera: Capturing...\n\r");
#endif

	while(tot != (raw_W * raw_H)) {

		while (ST_F_SYNC);

		while (!ST_F_SYNC);

		while (ST_F_SYNC && tot < max_pixels) {

			while (i < raw_W){

				while (!ST_P_SYNC);

				RAMBuffer[i++] = DATA_IN;
			}

			P8OUT |= BIT2;

			j++;

			if(j > ((raw_H - H) / 2) && j <= (raw_H - ((raw_H - H) / 2))){

				//DMA Src add
				__data16_write_addr((unsigned short) &DMA0SA, (unsigned long) &(RAMBuffer[(raw_W - W) / 2]));

				//DMA Dst add
				__data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &frame[tot - (raw_W - W) * j - (raw_W - 2 * (raw_W - W))]);

				// Block size
	    		DMA0SZ = i - (raw_W - W);

	    		// DMA Block repeat + Src inc + Dst incr + Src byte + Dst Byte + DMA int
	    		DMA0CTL = DMADT_5 | DMASRCINCR_3 | DMADSTINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAIE;	

				// DMA enable															
				DMA0CTL |= DMAEN;		

				//PRINTF("FRAM array address incremented: %u\n\r", DMA0DA);

				// Manual transfer trigger
				DMA0CTL |= DMAREQ; 											

				// Enable interrupts, enter LPM0 and wait DMA transfer
				__bis_SR_register(LPM0_bits + GIE);     	

			}

			tot = tot + i;
			i = 0;		

			P8OUT &= ~BIT2;
		}
	}

	TRG_OUT &= ~TRIG;

#ifdef enable_debug
	PRINTF("Camera: Total %u pixels\n\r", tot);
#endif

	cam->pixels = tot - ((raw_W - W)  * j) - (W * 2);

	if (cam->mode == 1)
		hm01b0_set_mode(0);

	cam->mode = hm01b0_reg_read(MODE_SELECT);

	cam->captured = 1;

#ifdef enable_debug
	PRINTF("Camera: Captured %u pixels\n\r", cam->pixels);
#endif

	P8OUT |= BIT1;
	//P8OUT &= ~BIT1;

}

void capture(HM01B0 *cam){

	uint16_t id = 0;

	P8OUT |= BIT3;

	hm01b0_init();

	hm01b0_enable();

	id = hm01b0_reg_default_init();

	P8OUT &= ~BIT3;

#ifdef enable_debug
	PRINTF("Camera: ID 0x%04x\n\r", id);	
#endif

	P8OUT |= BIT3;

	hm01b0_capture(cam);

	hm01b0_disable();

	hm01b0_deinit();
	
	P8OUT &= ~BIT3;
}

//------------------------------------------------------------------------------
// DMA interrupt handler
//------------------------------------------------------------------------------
void __attribute__ ((interrupt(DMA_VECTOR))) DMA_ISR (void){

  DMA0CTL &= ~DMAIFG;                       // Clear DMA0 interrupt flag
  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0

}
//------------------------------------------------------------------------------

#ifdef old_pins
	//------------------------------------------------------------------------------
	// Port 4 interrupt handler
	//------------------------------------------------------------------------------
	#if defined(__GNUC__)
	void __attribute__ ((interrupt(PORT4_VECTOR))) port4_isr_handler (void)
	#else
	#error Compiler not supported!
	#endif
	{
	    switch(__even_in_range(P4IV, P4IV__P4IFG7))
	    {
	        case P4IV__P4IFG1:  							// Vector  4:  P4.1 interrupt flag
	        	frame_numeber++;                     		// Increment frame counter
	        	
				if(frame_numeber >= 9){						// After 9 frames, disable frame Interrupt
					SYNC_IE	&= ~FVLD;                     	// VSYNC interrupt disable
					__bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0
				}
	        break;                		
	        default: break;
	    }
	}
	//------------------------------------------------------------------------------
#else
	//------------------------------------------------------------------------------
	// Port 1 interrupt handler
	//------------------------------------------------------------------------------
	#if defined(__GNUC__)
	void __attribute__ ((interrupt(PORT1_VECTOR))) port1_isr_handler (void)
	#else
	#error Compiler not supported!
	#endif
	{
	    switch(__even_in_range(P1IV, P1IV__P1IFG7))
	    {            		
	        case P1IV__P1IFG2:            					// Vector  6:  P1.2 interrupt flag
	        	frame_numeber++;                     		// Increment frame counter
	        	
				if(frame_numeber >= 9){						// After 9 frames, disable frame Interrupt
					SYNC_IE	&= ~FVLD;                     	// VSYNC interrupt disable
					__bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0
				}
			break;
	        default: break;
	    }
	}
	//------------------------------------------------------------------------------
#endif

