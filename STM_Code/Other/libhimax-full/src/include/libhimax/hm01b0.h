#ifndef HM01B0_H
#define HM01B0_H

#include "hm01b0_reg.h"
#include "hm01b0_pins.h"
#include "i2c.h"

//I2C receiving buffer
extern uint8_t ReceiveBuffer[];
extern uint8_t RXByteCtr;

enum mode {Standby, Streaming, Streaming2, Streaming3};
enum state {sleep, idle, active};
enum resolution{QVGA, QQVGA};
enum dataDepth{EightB, SixB};
enum dataIo{EightL, FourL, Serial};

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

#define HIMAX_I2C_ADDR					0x24

/* Enable camera VCC 
 * 
 * */
void hm01b0_enable();

/* Disable camera VCC 
 * 
 * */
void hm01b0_disable();

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


/* Camera GPIO configuiration 
 * 
 * */
uint8_t hm01b0_init(void);


/* Camera GPIO deinitialization 
 * 
 * */
uint8_t hm01b0_deinit();

/* Camera register initialization
 * Configure the camera encoding format
 * TO DO: Move from static initialization format to dynamic
 *  */
uint16_t hm01b0_reg_default_init(void);


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


/* Capturing an img from camera
 * Capture an image from the camera and store inside a buffer
 *
 *  */
void hm01b0_capture(HM01B0 *cam);


/* Initialize, Configure and caputre an image from the cmaera
 * 
 *
 *  */
void capture(HM01B0 *cam);

#endif

