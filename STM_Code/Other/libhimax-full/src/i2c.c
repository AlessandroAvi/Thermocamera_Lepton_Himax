#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "i2c.h"

void initI2C(uint8_t SLAVE_ADDR){

  // Init variables
  //ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
  memset(ReceiveBuffer, 0, MAX_BUFFER_SIZE*sizeof(uint8_t));
  RXByteCtr = 0;
  ReceiveIndex = 0;

  //TransmitBuffer[MAX_BUFFER_SIZE] = {0};
  memset(TransmitBuffer, 0, MAX_BUFFER_SIZE*sizeof(uint8_t));
  TXByteCtr = 0;
  TransmitIndex = 0;
  TransmitRegAddr = 0;

  __enable_interrupt();

  // Init software state machine
  MasterMode = IDLE_MODE;

  // I2C pins
  I2C_SEL0 |= (SDA | SCL);
  I2C_SEL1 &= ~(SDA | SCL);

  UCB2CTLW0 = UCSWRST;                                          // Enable SW reset
  UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC;       // I2C master mode, SMCLK
  UCB2BRW = 10;                                                 // SMCLK = 4Mhz - fSCL = SMCLK/10 = ~400kHz
  UCB2I2CSA = SLAVE_ADDR;                                       // Slave Address
  UCB2CTLW0 &= ~UCSWRST;                                        // Clear SW reset, resume operation
  UCB2IE |= UCNACKIE;                                           // Activate NACK interrupt
}

I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t count){

    /* Initialize state machine */
    MasterMode = TX_16REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB2I2CSA = dev_addr;
    UCB2IFG &= ~(UCTXIFG + UCRXIFG);                // Clear any pending interrupts
    UCB2IE &= ~UCRXIE;                              // Disable RX interrupt
    UCB2IE |= UCTXIE;                               // Enable TX interrupt

    UCB2CTLW0 |= UCTR + UCTXSTT;                    // I2C TX, start condition

    //__bis_SR_register(LPM1_bits | GIE);                     // Enter LPM3, enable interrupts

    return MasterMode;
}


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t count){

    /* Initialize state machine */
    MasterMode = TX_16REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB2I2CSA = dev_addr;
    UCB2IFG &= ~(UCTXIFG + UCRXIFG);            // Clear any pending interrupts
    UCB2IE &= ~UCRXIE;                          // Disable RX interrupt
    UCB2IE |= UCTXIE;                           // Enable TX interrupt

    UCB2CTLW0 |= UCTR + UCTXSTT;                // I2C TX, start condition

    //__bis_SR_register(LPM1_bits | GIE);                     // Enter LPM3, enable interrupts

    return MasterMode;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}


//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B2_VECTOR))) USCI_B2_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB2RXBUF
  uint8_t rx_val = 0;
  //P1OUT |= BIT0;
  switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB2RXBUF;
        if (RXByteCtr){

          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;

        }

        if (RXByteCtr == 1){

          UCB2CTLW0 |= UCTXSTP;

        }
        else if (RXByteCtr == 0){

          UCB2IE &= ~UCRXIE;                        //Disable I2C Rx Interrupt
          MasterMode = IDLE_MODE;                   // I2C State machine IDLE
          //__bic_SR_register_on_exit(LPM1_bits);
        }
        break;

    case USCI_I2C_UCTXIFG0:                         // Vector 24: TXIFG0
        switch (MasterMode){

          case TX_16REG_ADDRESS_MODE:               // Transmit register high  bits
              UCB2TXBUF = ((TransmitRegAddr >> 8) & 0xFF);
              MasterMode = TX_REG_ADDRESS_MODE;     // Switch to transmit low register bits
              break;

          case TX_REG_ADDRESS_MODE:
              UCB2TXBUF = (TransmitRegAddr & 0xFF); // Transmit low register bits
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:

              UCB2IE |= UCRXIE;                     // Enable RX interrupt
              UCB2IE &= ~UCTXIE;                    // Disable TX interrupt
              UCB2CTLW0 &= ~UCTR;                   // Switch to receiver
              MasterMode = RX_DATA_MODE;            // State state is to receive data
              UCB2CTLW0 |= UCTXSTT;                 // Send repeated start

              if (RXByteCtr == 1){

                  //Must send stop since this is the N-1 byte
                  while((UCB2CTLW0 & UCTXSTT));
                  UCB2CTLW0 |= UCTXSTP;             // Send stop condition

              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr){
                  UCB2TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else{

                  //Done with transmission
                  UCB2CTLW0 |= UCTXSTP;           // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB2IE &= ~UCTXIE;              // disable TX interrupt
                  //__bic_SR_register_on_exit(LPM1_bits);
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;

    default: break;

  }
}