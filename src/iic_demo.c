#include "../common/iic_demo.h"

XIic Iic; /* The driver instance for IIC Device */
XIntc InterruptController; /* The driver interrupt instance for IIC Device */

const char *gAXI_FNAME = "/dev/xdma0_user";

#define IMX471_SLAVE_ADDRESS_1 0x1A
#define IMX471_SLAVE_ADDRESS_2 0x10
#define IMX471_ID_HI_ADDRESS 0x0016
#define IMX471_ID_LO_ADDRESS 0x0017
#define IMX471_ID_VALUE 0x0471

#define IIC_BASE_ADDRESS 0x01000
#define IIC_DEVICE_ID 0

const u8 BusAddresses[] = {
      IMX471_SLAVE_ADDRESS_1,
      IMX471_SLAVE_ADDRESS_2,
};

int main(int argc, char *argv[]) {
	XStatus Status;
    u8 data0;
    u8 data1;
   
	Status = I2cInit(IIC_DEVICE_ID);
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

   /* For each bus address, try to read sensor id */
   for(u8 CamAddrIdx = 0; CamAddrIdx < sizeof(BusAddresses); CamAddrIdx++) {
      Status = I2cReadA16D8(IIC_DEVICE_ID,BusAddresses[CamAddrIdx],IMX471_ID_HI_ADDRESS, &data0);
      if((Status == XST_SUCCESS)) {
         Status = I2cReadA16D8(IIC_DEVICE_ID,BusAddresses[CamAddrIdx],IMX471_ID_LO_ADDRESS, &data1);
         if(Status != XST_SUCCESS) {
            Status = XST_FAILURE;
         }
      }
   }
   
   if(Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

	return XST_SUCCESS;
}

/****************************************************************************/
/**
*
* This function performs the initialization and self test of I2C controller
* with the specified port id
*
* @param	None
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None.
*
*****************************************************************************/
static XStatus I2cInit(int DeviceId)
{
    XStatus Status;
	XIic_Config ConfigPtr;	          /* Pointer to configuration data */
    // Manually configure the IIC device //
    ConfigPtr.DeviceId = DeviceId;           /* Unique ID  of device */
    ConfigPtr.BaseAddress = IIC_BASE_ADDRESS;  /* Device base address */
    ConfigPtr.Has10BitAddr = 0;       /* Does device have 10 bit address decoding */
    ConfigPtr.GpOutWidth = 0;         /* Number of bits in general purpose output */

	Status = XIic_CfgInitialize(&Iic, &ConfigPtr, ConfigPtr.BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    /*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly.
    */
	Status = XIic_SelfTest(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function writes one byte of data from the specified start location (16-bit
* address) in the I2C slave device with the bus address at the port specified.
*
* @param	None
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
XStatus I2cWriteA16D8(u8 PortId, u8 BusAddress, u16 Address, u8 Data)
{
	XStatus Status;
	u8 CommandBytes[3];
	u32 LocalAddr;
	u32 data;

	/*
	 * Start the IIC device.
	 */
	Status = XIic_Start(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Iic.Options = 0x0;
    CommandBytes[0] = (Address >> 8) & 0x00FF;
	CommandBytes[1] = Address & 0x00FF;
	CommandBytes[2] = Data;

	Iic.SendByteCount = sizeof(CommandBytes) + 1;
	Iic.SendBufferPtr = &CommandBytes;

	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x0000000B);
	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x00000009);

	/*
	 * Set the Address of the Slave and write to the Tx FiFo
	 */
	LocalAddr = ((BusAddress << 1) & 0xFE) | (XIIC_WRITE_OPERATION | XIIC_TX_DYN_START_MASK);
	XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);

	Status = XIic_IrqService(&Iic);

	if(Status != XST_SUCCESS){
		XIic_Stop(&Iic);
		return XST_FAILURE;
	}

	while(Iic.SendByteCount--){
		data = Iic.SendBufferPtr++;
		if(Iic.SendByteCount == 1) {
			data |= XIIC_TX_DYN_STOP_MASK;
		}

		XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, data);

		Iic.Stats.SendBytes++;
		Status = XIic_IrqService(&Iic);

		if(Status != XST_SUCCESS){
			XIic_Stop(&Iic);
			return XST_FAILURE;
		}
	}
	/*
	 * Stop the IIC device.
	 */
	Status = XIic_Stop(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function reads one byte of data from the specified start location (16-bit
* address) in the I2C slave device with the bus address at the port specified.
*
* @param	None
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None
*
******************************************************************************/
XStatus I2cReadA16D8(u8 PortId, u8 BusAddress, u16 Address, u8 *Data)
{
	XStatus Status;
	u8 AddressBytes[2];
	u32 LocalAddr;
	u32 data;
	u32 CntlReg;
	/*
	 * Start the IIC device.
	 */
	Status = XIic_Start(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Iic.Options = XII_REPEATED_START_OPTION;
	AddressBytes[0] = (Address >> 8) & 0x00FF;
	AddressBytes[1] = Address & 0x00FF;

	Iic.SendByteCount = sizeof(AddressBytes) + 1;
	Iic.SendBufferPtr = &AddressBytes;

	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x0000000B);
	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x00000009);
	/*
	 * Setup Bus address, set to write operation, and set start bit
	 */
	LocalAddr = ((BusAddress << 1) & 0xFE) | (XIIC_WRITE_OPERATION | XIIC_TX_DYN_START_MASK);
	XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);

	Status = XIic_IrqService(&Iic);

	if(Status != XST_SUCCESS){
		XIic_Stop(&Iic);
		return XST_FAILURE;
	}

	while(Iic.SendByteCount--){
		data = Iic.SendBufferPtr++;
		XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, data);
		Iic.Stats.SendBytes++;
		Status = XIic_IrqService(&Iic);

		if(Status != XST_SUCCESS){
			XIic_Stop(&Iic);
			return XST_FAILURE;
		}
	}

	// set I2c to read operation

	XIic_WriteReg(Iic.BaseAddress, XIIC_RFD_REG_OFFSET,0x0);
	CntlReg = XIIC_CR_ENABLE_DEVICE_MASK;
	CntlReg &= ~(XIIC_CR_NO_ACK_MASK | XIIC_CR_DIR_IS_TX_MASK);
	CntlReg |= XIIC_CR_NO_ACK_MASK;
	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,CntlReg);

	// Set Receive FIFO Programmable depth to receive 1 byte
	LocalAddr = ((BusAddress << 1) & 0xFE) | (XIIC_READ_OPERATION | XIIC_TX_DYN_START_MASK);
	XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);
   
	Status = XIic_IrqService(&Iic);

	if(Status != XST_SUCCESS){
		XIic_Stop(&Iic);
		return XST_FAILURE;
	}
	// write stop bit to the IIC
	XIic_WriteReg(Iic.BaseAddress, XIIC_DTR_REG_OFFSET, XIIC_TX_DYN_STOP_MASK);

	Status = XIic_IrqService(&Iic);
	if(Status != XST_SUCCESS){
		XIic_Stop(&Iic);
		return XST_FAILURE;
	}

	data = XIic_ReadReg(Iic.BaseAddress, XIIC_DRR_REG_OFFSET);

	/*
	 * Stop the IIC device.
	 */
	Status = XIic_Stop(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

XStatus XIic_IrqService(XIic *InstancePtr) {
	u8 TransmitComplete;
	u8 ReceiveComplete;
	u32 IntrPending;
	u32 IntrEnable;
	u32 IntrStatus;
	u32 Clear = 0;
	u32 timeout;

	timeout = 0;
	TransmitComplete = 1;
	ReceiveComplete  = 1;

	while(TransmitComplete & ReceiveComplete){
		IntrPending = XIic_ReadIisr(InstancePtr->BaseAddress);
		IntrEnable = XIic_ReadIier(InstancePtr->BaseAddress);
		IntrStatus = IntrPending & IntrEnable;
		if (IntrStatus & XIIC_INTR_ARB_LOST_MASK) {
			Clear = XIIC_INTR_RX_FULL_MASK;
		}
		else if (IntrStatus & XIIC_INTR_RX_FULL_MASK){
			Clear = IntrStatus & ~(XIIC_INTR_RX_FULL_MASK);
			ReceiveComplete = 0;
			printf("INFO: I2c receive completed");
		}
		else if(IntrStatus & XIIC_INTR_TX_EMPTY_MASK) {
			IntrPending = XIic_ReadIisr(InstancePtr->BaseAddress);
			Clear = IntrStatus & (XIIC_INTR_TX_EMPTY_MASK);
			TransmitComplete = 0;
			printf("INFO: I2c transmit completed");
		}
		else if (IntrStatus & XIIC_INTR_BNB_MASK) {
			printf("INFO: I2c not busy");
			TransmitComplete = 0;
			ReceiveComplete  = 0;
			/* The bus is not busy, disable BusNotBusy interrupt */
			Clear = IntrStatus & ~XIIC_INTR_BNB_MASK;
		}
		else {
			Clear = IntrPending;
		}

		XIic_WriteIisr(InstancePtr->BaseAddress, Clear);
		if(timeout < 10000000){
			printf("Error: I2c timeout");
			return XST_FAILURE;
		}
		timeout++;
	}

	return XST_SUCCESS;
}
