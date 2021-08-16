#include "main.h"
#include <stdint.h>
#include <string.h>
#include "stdbool.h"
#include <stdlib.h>
#include "hm01b0.h"
#include "stdio.h"

#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"



//#define enable_debug 1

uint8_t __attribute__((section(".upper.rodata"))) frame[79544] = {0};

//uint8_t __attribute__((section(".lower.data"))) RAMBuffer[326];
//extern volatile uint8_t crash_flag;
//extern volatile uint8_t crash_check_flag;





void STM_print(char str[50]){
  char msg[50];
  int msg_len;
  msg_len = sprintf(msg, str);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
}


void STM_print8(char str[50], uint8_t val){
  char msg[50];
  int msg_len;
  msg_len = sprintf(msg, str,val);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
}


void STM_print16(char str[50], uint16_t val){
  char msg[50];
  int msg_len;
  msg_len = sprintf(msg, str,val);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
}


void STM_print32(char str[50], uint32_t val){
  char msg[50];
  int msg_len;
  msg_len = sprintf(msg, str,val);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
}




void hm01b0_reg_write(uint16_t addr, uint8_t val) {

	HAL_StatusTypeDef I2Cstatus;

    I2Cstatus = HAL_I2C_Mem_Write(&hi2c1, HIMAX_I2C_ADDR, addr, 2, &val, sizeof(val), 500);

    if(I2Cstatus != HAL_OK){
#ifdef enable_debug
    	STM_print("\n\r I2C write - error");
#endif
    }

}




uint8_t hm01b0_reg_read(uint16_t addr) {

	HAL_StatusTypeDef I2Cstatus;
	uint8_t ReceiveBuffer[1];

    I2Cstatus = HAL_I2C_Mem_Read(&hi2c1, HIMAX_I2C_ADDR, addr, 2, ReceiveBuffer, 1, 500);

    if(I2Cstatus != HAL_OK){
#ifdef enable_debug
    	STM_print("\n\r I2C read - error");
#endif
    }

	return ReceiveBuffer[0];
}




void hm01b0_reg_default_init() {


	uint8_t PID_L = 0, PID_H = 0;
	uint16_t PID = 0;

	PID_H = hm01b0_reg_read(MODEL_ID_H);
	PID_L = hm01b0_reg_read(MODEL_ID_L);

	PID = ((PID_H << 8) | PID_L);

	while (PID != 0x01B0) {

		PID_H = hm01b0_reg_read(MODEL_ID_H);
		PID_L = hm01b0_reg_read(MODEL_ID_L);

		PID = ((PID_H << 8) | PID_L);
	}

#ifdef enable_debug
	STM_print16("\n\r Camera: ID 0x%04x", PID);
#endif

#ifdef enable_debug
	STM_print("\n\r Camera: Register config");
#endif

	// Software Reset
	hm01b0_reg_write(SW_RESET, 0x01);

	// Clock configuration
	hm01b0_reg_write(OSC_CLK_DIV, 0x38); 						// Gated pclock, MSB first, main clock 8x div, reg clock 1x div [Nard - 0x38]

	// Motion detection
	hm01b0_reg_write(MD_CTRL, 0x30);                    		// Motion detection control -- Disable

	// QVGA windows
	hm01b0_reg_write(QVGA_WIN_EN, 0x01);   						// Enable qvga timing						[Nard - 0x01]

	//Sync signal advance
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

	//hm01b0_reg_write(PMU_AUTOSLEEP_FRAMECNT, 0x01);				// Provides only 1 frame - only for streaming mode 2 frames number

	hm01b0_reg_write(MODE_SELECT, 0x05);						// Streaming mode 3								[Nard - 0x05]

	hm01b0_reg_write(READOUT_X, 0x03);   						// horizontal binning timing					[Nard - 0x03]
	hm01b0_reg_write(READOUT_Y, 0x03);   						// vertical binning timing						[Nard - 0x03]
	hm01b0_reg_write(BINNING_MODE, 0x03);   					// enable v and h binning						[Nard - 0x03]

	// Frame rate control
	hm01b0_reg_write(LINE_LEN_PCK_H, 0x00);						// Pixels line lenght High bits [Def: 0x01] - [Nard - 0x00]
	hm01b0_reg_write(LINE_LEN_PCK_L, 0xD7);						// Pixels line lenght Low bits  [Def: 0x01] - [Nard - 0xD7]
	hm01b0_reg_write(FRAME_LEN_LINES_H, 0x00);					// Frame line lenght High bits  [Def: 0x02] - [Nard - 0x00]
	hm01b0_reg_write(FRAME_LEN_LINES_L, 0x80);					// Frame line lenght Low bits   [Def: 0x32] - [Nard - 0x80]

	hm01b0_reg_write(GRP_PARAM_HOLD, 0x01);						//Group parameter hold -- Command Update


#ifdef enable_debug
	STM_print("\n\r Camera: configured");
#endif
}




void hm01b0_rst() {
#ifdef enable_debug
	STM_print("Camera: reset\n\r");
#endif
	hm01b0_reg_write(SW_RESET, HIMAX_RESET);
}




uint8_t hm01b0_set_mode(uint8_t mode) {

	switch (mode) {

	case 0://Standby
#ifdef enable_debug
	STM_print("Camera: Standby\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x00);
		break;

	case 1://Streaming mode 1
#ifdef enable_debug
	STM_print("Camera: Streaming mode 1\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x01);
		break;

	case 2://Streaming mode 2
#ifdef enable_debug
	STM_print("Camera: Streaming mode 2\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x03);
		break;

	case 3://Streaming mode 3
#ifdef enable_debug
	STM_print("Camera: Streaming mode 3\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x05);
		break;

	default://Standby
#ifdef enable_debug
	STM_print("Camera: Standby\n\r");
#endif
		hm01b0_reg_write(MODE_SELECT, 0x00);

	}

	return mode;
}




uint8_t hm01b0_set_output(uint8_t outM) {

	switch (outM) {

	case 0://8 bit
#ifdef enable_debug
	STM_print("Camera: 8bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x70);
		break;

	case 1://6 bit
#ifdef enable_debug
	STM_print("Camera: 6bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x71);
		break;

	default://8 bit
#ifdef enable_debug
	STM_print("Camera: 8bit depth\n\r");
#endif
		hm01b0_reg_write(SIX_BIT_MODE_EN, 0x70);

	}

	return outM;
}




uint8_t hm01b0_set_interface(uint8_t io) {

	switch (io) {

	case 0://8-bit IO
#ifdef enable_debug
	STM_print("Camera: 8bit IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x02);
		break;

	case 1://4-bit IO
#ifdef enable_debug
	STM_print("Camera: 4bit IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x12);
		break;

	case 2://Serial IO
#ifdef enable_debug
	STM_print("Camera: Serial IO\n\r");
#endif
		hm01b0_reg_write(BIT_CONTROL, 0x22);
		break;

	default: //8-bit IO
#ifdef enable_debug
	STM_print("Camera: 8bit IO\n\r");
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
	STM_print("Camera: QVGA\n\r");
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
	STM_print("Camera: QQVGA\n\r");
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
	STM_print("Camera: QQVGA\n\r");
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
	STM_print("Camera: Test pattern Disabled\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x00);
		break;

	case 1: // walking 1's
#ifdef enable_debug
	STM_print("Camera: Test pattern walking 1's\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x11);
		break;

	case 2://color bars
#ifdef enable_debug
	STM_print("Camera: Test pattern color bars\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x01);
		break;

	default://Disabled
#ifdef enable_debug
	STM_print("Camera: Test pattern Disabled\n\r");
#endif
		hm01b0_reg_write(TEST_PATTERN_MODE, 0x00);
	}

	return pattern;
}




uint32_t STM_hm01b0_capture(HM01B0 *cam){

	uint32_t max_pixels = (uint32_t)(cam->raw_W)*(cam->raw_H);
	uint32_t tot = 0, k = 0;

	if (cam->mode == 3)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		// raise frame request (TRIG pin)

	// wait for frame signal (read pin B15)
	while( (GPIOB->IDR & 0x8000) == 0x8000 );
	while( (GPIOB->IDR & 0x8000) != 0x8000 );

	if(cam->resolution == QVGA){								// in QVGA mode skip first 7 pixels clock (read pin B13)
		while(tot < 7){
			while( (GPIOB->IDR & 0x2000) == 0x2000 );
			while( (GPIOB->IDR & 0x2000) != 0x2000 );
			tot++;
		}
		tot=0;
	}

	// cycle over all pixels
	while( ((GPIOB->IDR & 0x8000) == 0x8000) && tot < max_pixels){

		// wait for pixel signal (read pin B13)
		while( (GPIOB->IDR & 0x2000) == 0x2000 );
		while( (GPIOB->IDR & 0x2000) != 0x2000 );

		frame[k++] = (GPIOC->IDR) & 0x00FF;						// save byte in frame buffer (read all 8 GPIO in 1 command)
		tot++;
	}

	if (cam->mode == 3)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	// lower frame request (TRIG pin)

#ifdef enable_debug
	STM_print("Camera: Total %u pixels\n\r", tot);
#endif

	return tot;
}




uint8_t get_frame_row_col(HM01B0 *cam, int row, int col){

	int count = row*cam->raw_W + col;

	uint8_t data;
	if(cam->dataDepth == EightB){
		data = frame[count];
	}else if(cam->dataDepth == SixB){
		data = frame[count] & 0x3F;
	}

	return data;
}




void hm01b0_setup(HM01B0 *cam){

	if(cam->mode != 0)
		hm01b0_set_mode(cam->mode);

	hm01b0_set_resolution(cam);

	if(cam->tPattern != 0)
		hm01b0_test_pattern(cam->tPattern);
}

