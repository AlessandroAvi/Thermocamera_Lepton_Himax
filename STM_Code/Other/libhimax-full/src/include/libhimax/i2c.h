#ifndef I2C_H
#define I2C_H

//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define I2C_SEL0	P7SEL0	//Port 7SEL0 
#define I2C_SEL1	P7SEL1	//Port 7SEL1
#define SCL     	BIT1 	// P7.1
#define SDA     	BIT0 	// P7.0

//******************************************************************************
// Rx/Tx maximum buffer size ***************************************************
//******************************************************************************

#define MAX_BUFFER_SIZE     20

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    TX_16REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode MasterMode;

/* The Register Address/Command to use*/
uint16_t TransmitRegAddr;

/* ReceiveBuffer:   Buffer used to receive data in the ISR
 * RXByteCtr:       Number of bytes left to receive
 * ReceiveIndex:    The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer:  Buffer used to transmit data in the ISR
 * TXByteCtr:       Number of bytes left to transfer
 * TransmitIndex:   The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE];
uint8_t RXByteCtr;
uint8_t ReceiveIndex;

uint8_t TransmitBuffer[MAX_BUFFER_SIZE];
uint8_t TXByteCtr;
uint8_t TransmitIndex;

/* I2C initialization */
void initI2C(uint8_t SLAVE_ADDR);

/* I2C Write and Read Functions */

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: SLAVE_REGISTER
 * *reg_data: The buffer to write
 *           Example: Resiter_Data
 * count: The length of *reg_data
 *           Example: DATA_LENGTH
 *  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: SLAVE_REGISTER
 * count: The length of data to read
 *           Example: DATA_LENGTH
 *  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t count);

/* Copy data from source array to destination array 
 * 
 * source: The source address to by copied
 *
 * dest: The destination address to which copy data
 *
 * count: The lenght of the data to be copied
 *
 */
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);


#endif
