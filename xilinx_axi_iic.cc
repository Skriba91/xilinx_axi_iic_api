#include <stdint.h>
#include <xparameters.h>
#include "Xil_io.h"

#define IIC_INITPARAM 0b00000110
#define IIC_INITPARAMEN 0b00000101
#define IIC_IDLEMASK 0b11000000		/**< When IIC status register equal to this mask that means the IIC is ide. Transmit and recive FIFO-s are empty and bus is idle*/
#define IIC_ACKEN 0b00100101
#define IIC_TXEMPTYMASK 0b10000000
#define IIC_RXFULLMASK 0b01000000

#define IIC_CR XPAR_AXI_IIC_0_BASEADDR + 0x100				/**< Register address of IIC Control Register */
#define IIC_ST XPAR_AXI_IIC_0_BASEADDR + 0x104				/**< Register address of IIC Status register
																 bit 7: Transmit FIFO empty. This bit is set High when the transmit FIFO is empty.
																 bit 6: Receive FIFO empty. This is set High when the receive FIFO is empty.
																 bit 5: Receive FIFO full. This bit is set High when the receive FIFO is full.
																 bit 4: Transmit FIFO full. This bit is set High when the transmit FIFO is full.
																 bit 3: Slave Read/Write. When the IIC bus interface has been addressed as a slave (AAS is set), this bit indicates the value of the read/write bit sent by the master.
																 bit 2: Bus Busy. This bit indicates the status of the IIC bus. This bit is set when a START condition is detected and cleared when a STOP condition is detected.
																 bit 1: Addressed as Slave. When the address on the IIC bus matches the slave address in the Address register (ADR), the IIC bus interface is being addressed as a slave and switches to slave mode.
																 bit 0: Addressed By a General Call.
															*/
#define IIC_TX_FIFO XPAR_AXI_IIC_0_BASEADDR + 0x108			/**< Register address of IIC Transmit FIFO */
#define IIC_RX_FIFO XPAR_AXI_IIC_0_BASEADDR + 0x10C			/**<  */
#define IIC_RX_FIFO_PIRQ XPAR_AXI_IIC_0_BASEADDR + 0x120	/**<  */
#define IIC_GPO XPAR_AXI_IIC_0_BASEADDR + 124				/**<  */


/**
* 
*/
typedef struct IIC_PARAMETERS {
	uint8_t GEN_CALL;		/**< Write 1 to enable general call, write 0 to disable */
	uint8_t GPO;			/**< Value of GPO pins after initiation */
	uint8_t RX_FIFO_PIRQ;	/**< Size of the RX fifo. Min 0x00, max 0x0F */
	//TODO: Implement timing parameters and interrupts
} IIC_PARAMETERS;


void i2c_init() {
	Xil_Out32(IIC_GPO, 0);
	Xil_Out32(IIC_RX_FIFO_PIRQ, 0x0F);
	Xil_Out32(IIC_CR, IIC_INITPARAM);
	Xil_Out32(IIC_CR, IIC_INITPARAMEN);
}

/**
*
* @param device_address The seven bit address of the device
* @param reg_msb Higher 8 bit of the target register address
* @param reg_lsb Lower 8 bit of the target register address
* @param data Data to be transmitted
*/
void i2c_write(uint8_t device_address, uint8_t reg_msb, uint8_t reg_lsb, uint8_t data) {
	u32 iic_start = 0x100 + SLAVE_ADDR;
	while (Xil_In32(IIC_ST) != IIC_IDLEMASK);
	Xil_Out32(IIC_TX_FIFO, iic_start);
	Xil_Out32(IIC_TX_FIFO, addr);
	Xil_Out32(IIC_TX_FIFO, 0x200 + data);
	while (Xil_In32(IIC_ST) != IIC_IDLEMASK);
}

void i2c_read() {
	u32 iic_start = 0x100 + SLAVE_ADDR;
	int32_t data;
	while (Xil_In32(IIC_ST) != IIC_IDLEMASK);
	Xil_Out32(IIC_TX_FIFO, iic_start);
	Xil_Out32(IIC_TX_FIFO, start_addr);
	++iic_start;
	Xil_Out32(IIC_TX_FIFO, iic_start);
	Xil_Out32(IIC_TX_FIFO, 0x203);
	while ((Xil_In32(IIC_ST) & IIC_TXEMPTYMASK) != IIC_TXEMPTYMASK);
	for (u8 i = 0; i < 3; ++i) {
		while ((Xil_In32(IIC_ST) & IIC_RXFULLMASK) == IIC_RXFULLMASK);
		data = Xil_In32(IIC_RX_FIFO);
		data <<= 8;
	}
	data >>= 4;
	return data;
}