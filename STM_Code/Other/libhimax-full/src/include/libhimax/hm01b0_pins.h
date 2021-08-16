#ifndef HM01B0_PINS_H
#define HM01B0_PINS_H

#include <msp430.h>

//#define old_pins

#ifdef old_pins
	//Camera VCC enable pin
	#define	CAM_VCC_SEL0		P6SEL0
	#define CAM_VCC_SEL1		P6SEL1
	#define CAM_VCC_DIR			P6DIR
	#define CAM_VCC_OUT			P6OUT
	#define CAM_VCC_EN			BIT1

	//Data PINS
	#define	DATA_SEL0		P3SEL0
	#define DATA_SEL1		P3SEL1
	#define DATA_DIR		P3DIR
	#define DATA_IN			P3IN		/* Data Port P3.0-7 */

	//Camera External clock input
	#define M_CLK_SEL0		PJSEL0		
	#define M_CLK_SEL1		PJSEL1
	#define M_CLK_DIR		PJDIR
	#define M_CLK_OUT		PJOUT
	#define M_CLK			BIT0

	//Frame Sync Pins
	#define SYNC_SEL0		P4SEL0
	#define SYNC_SEL1		P4SEL1
	#define SYNC_DIR 		P4DIR
	#define SYNC_OUT 		P4OUT
	#define SYNC_IN			P4IN		// Sync signals input register 
	#define SYNC_IE			P4IE 		// Interrupt Enable 
	#define SYNC_IES		P4IES 		// Interrupt Edge 
	#define SYNC_IFG		P4IFG		// Interrupt Flag 
	#define PCLKO			BIT3		// Pixel Clock 
	#define LVLD 			BIT2		// Line valid 
	#define FVLD			BIT1		// Frame valid

	//Trigger pins
	#define TRG_DIR			P1DIR		
	#define TRG_OUT			P1OUT
	#define TRG_SEL0		P1SEL0
	#define TRG_SEL1		P1SEL1
	#define INTR			BIT5		// Frame Interrupt 
	#define TRIG			BIT3		// Frame Trigger 

	//Sync Pins input controll 
	#define ST_F_SYNC 	(SYNC_IN & FVLD)	// Check F clock input state/
	#define ST_L_SYNC 	(SYNC_IN & LVLD)	// Check L clock input state
	#define ST_P_SYNC 	(SYNC_IN & PCLKO)	// Check P clock input state

#else

	//Camera VCC enable pin
	#define	CAM_VCC_SEL0		P4SEL0
	#define CAM_VCC_SEL1		P4SEL1
	#define CAM_VCC_DIR			P4DIR
	#define CAM_VCC_OUT			P4OUT
	#define CAM_VCC_EN			BIT0

	//Data PINS
	#define	DATA_SEL0		P3SEL0
	#define DATA_SEL1		P3SEL1
	#define DATA_DIR		P3DIR
	#define DATA_IN			P3IN		/* Data Port P3.0-7 */

	//Camera External clock input
	#define M_CLK_SEL0		PJSEL0		
	#define M_CLK_SEL1		PJSEL1
	#define M_CLK_DIR		PJDIR
	#define M_CLK_OUT		PJOUT
	#define M_CLK			BIT0

	//Frame Sync Pins
	#define SYNC_SEL0		P1SEL0
	#define SYNC_SEL1		P1SEL1
	#define SYNC_DIR 		P1DIR
	#define SYNC_OUT 		P1OUT
	#define SYNC_IN			P1IN		// Sync signals input register 
	#define SYNC_IE			P1IE 		// Interrupt Enable 
	#define SYNC_IES		P1IES 		// Interrupt Edge 
	#define SYNC_IFG		P1IFG		// Interrupt Flag 
	#define PCLKO			BIT0		// Pixel Clock 
	#define LVLD 			BIT1		// Line valid 
	#define FVLD			BIT2		// Frame valid

	//Trigger pins
	#define TRG_DIR			P1DIR		
	#define TRG_OUT			P1OUT
	#define TRG_SEL0		P1SEL0
	#define TRG_SEL1		P1SEL1
	#define INTR			BIT3		// Frame Interrupt 
	#define TRIG			BIT4		// Frame Trigger 


	//Sync Pins input controll 
	#define ST_F_SYNC 	(SYNC_IN & FVLD)	// Check F clock input state/
	#define ST_L_SYNC 	(SYNC_IN & LVLD)	// Check L clock input state
	#define ST_P_SYNC 	(SYNC_IN & PCLKO)	// Check P clock input state

#endif
#endif
