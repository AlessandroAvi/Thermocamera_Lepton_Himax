#ifndef HM01B0_H
#define HM01B0_H

#include "hm01b0_reg.h"
#include "stdbool.h"

//I2C receiving buffer
extern uint8_t ReceiveBuffer[];
extern uint8_t RXByteCtr;

enum mode {Standby, Streaming, Streaming2, Streaming3};
enum state {sleep, idle, active};
enum resolution{QVGA, QQVGA};
enum dataDepth{EightB, SixB};
enum dataIo{EightL, FourL, Serial};
enum tPattern{Disabled, Walking, ColorBar};


typedef struct HM01B0 {

	uint8_t mode;
	uint8_t state;
	uint8_t resolution;
	uint8_t dataDepth;
	uint8_t dataIo;
	uint8_t tPattern;

	uint16_t raw_W;
	uint16_t raw_H;
	uint16_t W;
	uint16_t H;

	uint32_t pixels;

	uint8_t captured;

}HM01B0;




/* Sending debug message through UART
 * Write str inside uart buffer
 *
 * str:	Message to be displayed
 *
 * val: Value to be displayed in the message
 *  */
void STM_print(char str[50]);
void STM_print8(char str[50],  uint8_t val);
void STM_print16(char str[50], uint16_t val);
void STM_print32(char str[50], uint32_t val);



/* Setting Camera register using I2C
 * Set val into addr camera register
 *
 * addr:	Camera addres were to write val.
 *
 * val:		The value to write at addr register
 *
 *  */
void hm01b0_reg_write(uint16_t addr, uint8_t val);



/* Read register from camera using I2C
 * Read the value store into addr camera register
 *
 * addr:	Camera addres that we want to read
 *
 * Return the value at addr
 *  */
uint8_t hm01b0_reg_read(uint16_t addr);



/* Camera register initialization
 * Configure the camera encoding format
 * TO DO: Move from static initialization format to dynamic
 *  */
void hm01b0_reg_default_init(void);



/* Reset camera to default configuration
 * Reset all camera register with the default values
 *
 *  */
void hm01b0_rst(void);



/* This function selects HM01B0 camera mode.
 *
 * mode: HM01B0 streaming mode
 * 		0 - Standby
 *		1 - Streaming mode 1
 *		2 - Streaming mode 2
 *		3 - Streaming mode 3
 *
 * return: selected streaming mode
 *
 *  */
uint8_t hm01b0_set_mode(uint8_t mode);



/* This function selects HM01B0 data output mode.
 *
 * outM: HM01B0 output mode
 * 		0 - RAW 8 bit
 *		1 - RAW 6 bit
 *
 * return: selected output mode
 *
 *  */
uint8_t hm01b0_set_output(uint8_t outM);



/* This function selects HM01B0 IO interface mode.
 *
 * io: HM01B0 interface mode
 * 		0 - 8-bit IO interface mode
 *		1 - 4-bit IO interface mode
 *		2 - serial IO interface mode
 *
 * return: selected interface mode
 *
 *  */
uint8_t hm01b0_set_interface(uint8_t io);



/* This function selects HM01B0 frame resolution.
 *
 * res: HM01B0 frame resolution
 * 		0 - QVGA  -- 326x244
 *		1 - QQVGA -- 164x122
 *
 * return: selected frame resolution
 *
 *  */
uint8_t hm01b0_set_resolution(HM01B0 *cam);



/* This function activates HM01B0 test pattern.
 *
 * pattern: HM01B0 test pattern
 * 		0 - Disable
 *		1 - Walking 1's
 *		2 - Color bars
 *
 * return: selected test pattern
 *
 *  */
uint8_t hm01b0_test_pattern(uint8_t pattern);



/* Capture and save a frame from the himax
 *
 * cam:	Struct used for knowing the setup of the camera
 *
 * Return the number of pixel caucht by the function
 *  */
uint32_t STM_hm01b0_capture(HM01B0 *cam);



/* Return the pixel value representing pixel(row, col)
 *
 * cam:	Struct used for knowing the size of the allocated array
 *
 * row: Value that represents the vertical position of the pixel
 *
 * col: Value that represents the horizontal position of the pixel
 *
 * Return the 8 bit value of the pixel
 *  */
uint8_t get_frame_row_col(HM01B0 *cam, int row, int col);



/* Setup the camera depending on the struct
 *
 * cam:	Struct used for knowing the setup of the camera
 *
 *  */
void hm01b0_setup(HM01B0 *cam);



#endif

