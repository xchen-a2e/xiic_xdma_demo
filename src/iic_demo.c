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
    u32 timeout = 0;
	Status = I2cInit(IIC_DEVICE_ID);
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIic_SetGpOutput(&Iic, 0x0);
	while(1){
		if(timeout < 10000){
			break;
		}
	}

	XIic_SetGpOutput(&Iic, 0x1);

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

   XIic_SetGpOutput(&Iic, 0x0);
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
    ConfigPtr.GpOutWidth = 1;         /* Number of bits in general purpose output */

	Status = XIic_CfgInitialize(&Iic, &ConfigPtr, ConfigPtr.BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    /*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly.
    */
	XIic_WriteIier(Iic.BaseAddress, 0);
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

	Iic.SendByteCount = sizeof(CommandBytes);
	Iic.SendBufferPtr = &CommandBytes;

	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x0000000B);
	XIic_WriteReg(Iic.BaseAddress, XIIC_CR_REG_OFFSET,0x00000009);

	/*
	 * Set the Address of the Slave and write to the Tx FiFo
	 */
	Status = Iic_Send(&Iic, BusAddress);
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
	u32 IntrPending;
	/*
	 * Start the IIC device.
	 */
	Status = XIic_Start(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	IntrPending = XIic_ReadIisr(Iic.BaseAddress);
	if(IntrPending & (XIIC_INTR_RX_FULL_MASK)){
		*Data = (u8) XIic_ReadReg(Iic.BaseAddress, XIIC_DRR_REG_OFFSET);
		XIic_WriteReg(Iic.BaseAddress, XIIC_RFD_REG_OFFSET, 0);
		return XST_FAILURE;
	}

	ClearIisrIrq(&Iic, XIIC_TX_RX_INTERRUPTS);
	Iic_DisableIrq(&Iic, XIIC_TX_RX_INTERRUPTS);

	AddressBytes[0] = (Address >> 8) & 0x00FF;
	AddressBytes[1] = Address & 0x00FF;
	Iic.SendBufferPtr = AddressBytes;
	Iic.SendByteCount = 2;

	Iic.Options = XII_REPEATED_START_OPTION;
	Status = Iic_Send(&Iic, BusAddress);


	if (Status != XST_SUCCESS) {
		XIic_Stop(&Iic);
		return XST_FAILURE;
	}
	/*
	 *  I2c to read operation
	 */
	Iic.RecvByteCount = 1;
	Iic.Options = 0x0;
	Status = Iic_Recv(&Iic, BusAddress, Data);

	if (Status != XST_SUCCESS) {
		XIic_Stop(&Iic);
		return XST_FAILURE;
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

XStatus XIic_IrqService(XIic *InstancePtr) {
	u8 TransmitComplete;
	u8 ReceiveComplete;
	u32 IntrPending;
	u32 IntrEnable;
	u32 IntrStatus;
	u32 StatusReg;
	u32 CntlReg;
	u32 Clear = 0;
	u32 timeout;

	timeout = 0;
	TransmitComplete = 1;
	ReceiveComplete  = 1;

	while(TransmitComplete & ReceiveComplete){
		IntrPending = XIic_ReadIisr(InstancePtr->BaseAddress);
		IntrEnable = XIic_ReadIier(InstancePtr->BaseAddress);
		IntrStatus = IntrPending & IntrEnable;
		CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
		StatusReg =  XIic_ReadReg(InstancePtr->BaseAddress, XIIC_SR_REG_OFFSET);
		if (IntrStatus & XIIC_INTR_ARB_LOST_MASK) {
			Clear = XIIC_INTR_ARB_LOST_MASK;
		}
		else if (IntrStatus & XIIC_INTR_TX_ERROR_MASK){
			Clear = XIIC_INTR_TX_ERROR_MASK;
			IntrPending = XIic_ReadIisr(InstancePtr->BaseAddress);
			if(IntrPending & XIIC_INTR_RX_FULL_MASK){
				Iic_DisableIrq(InstancePtr, XIIC_INTR_TX_ERROR_MASK);
				ReceiveComplete = 0;
			}
		}
		else if (IntrStatus & XIIC_INTR_RX_FULL_MASK){
			// generate stop condition
			CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
			CntlReg &= ~XIIC_CR_MSMS_MASK;
			XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET,CntlReg);
			Clear = XIIC_INTR_TX_ERROR_MASK;
			ReceiveComplete = 0;
			printf("INFO: I2c receive completed");
		}
		else if((IntrStatus & XIIC_INTR_TX_EMPTY_MASK) || (IntrStatus & XIIC_INTR_TX_HALF_MASK)) {
			IntrStatus = XIic_ReadIisr(InstancePtr->BaseAddress);
			Clear = IntrStatus & ~(XIIC_INTR_TX_EMPTY_MASK | XIIC_INTR_TX_HALF_MASK);
			TransmitComplete = 0;
			printf("INFO: I2c transmit completed");
		}
		else if (IntrStatus & XIIC_INTR_BNB_MASK) {
			printf("INFO: I2c not busy");
			/* The bus is not busy, disable BusNotBusy interrupt */
			XIic_WriteIier(InstancePtr->BaseAddress, (XIic_ReadIier(InstancePtr->BaseAddress) & (XIIC_INTR_BNB_MASK)));
			Clear = XIIC_INTR_BNB_MASK;
		}

		XIic_WriteIisr(InstancePtr->BaseAddress, Clear);
		if(timeout > 10000000){
			printf("Error: I2c timeout");
			return XST_FAILURE;
		}
		timeout++;
	}

	return XST_SUCCESS;
}

static void ClearIisrIrq(XIic *InstancePtr, u32 Mask) {
	XIic_WriteIisr(InstancePtr->BaseAddress, (XIic_ReadIisr(InstancePtr->BaseAddress) & (Mask)));
}

static void IntrClearEnable(XIic *InstancePtr, u32 Mask) {

	u32 isr;
	u32 ier;
	isr = XIic_ReadIisr(InstancePtr->BaseAddress);
	ier = XIic_ReadIier(InstancePtr->BaseAddress);

	XIic_WriteIisr(InstancePtr->BaseAddress, (isr) & (Mask));
	XIic_WriteIier(InstancePtr->BaseAddress, (ier) | (Mask));
}

static void Iic_DisableIrq(XIic *InstancePtr, u32 Mask){
		XIic_WriteIier((InstancePtr->BaseAddress), XIic_ReadIier(InstancePtr->BaseAddress) & ~(Mask));
}

static void WriteIrqSetup(XIic *InstancePtr){
	/*
	 * Disable all interrupts
	 */
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DGIER_OFFSET, 0);
	// reset TX fifo
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, 0x3);
	/*
	 * Clear and enable any pending Tx interrupt
	 */
	IntrClearEnable(InstancePtr, XIIC_TX_INTERRUPTS);
	/*
	 * Enable interrupt
	 */
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DGIER_OFFSET,XIIC_GINTR_ENABLE_MASK);
}

static int Iic_Send(XIic *InstancePtr, u8 BusAddress) {
	XStatus Status;
	u32 LocalAddr;
	u32 CntlReg;
	u32 mask = 0;

	WriteIrqSetup(InstancePtr);
	/*
	 * Setup Bus address, set to write operation, and set start bit
	 */
	LocalAddr = ((BusAddress << 1) & 0xFE) | (XIIC_WRITE_OPERATION);
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);
	mask = (XIIC_CR_MSMS_MASK | XIIC_CR_DIR_IS_TX_MASK | XIIC_CR_ENABLE_DEVICE_MASK);
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, mask);

	CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
	while(InstancePtr->SendByteCount--){
		if(InstancePtr->SendByteCount > 1){
			XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, *InstancePtr->SendBufferPtr++);
			Iic.Stats.SendBytes++;
			if (InstancePtr->SendByteCount < 2) {
				Iic_DisableIrq(&Iic, XIIC_INTR_TX_HALF_MASK);
			}
		}
		else if(InstancePtr->SendByteCount ==1){
			if(InstancePtr->Options == XII_REPEATED_START_OPTION){
				XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, *InstancePtr->SendBufferPtr++);
			}
			else {
				// generate stop signal then write last byte to tx fifo
				CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
				CntlReg &= ~XIIC_CR_MSMS_MASK;
				XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);
				XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, *InstancePtr->SendBufferPtr++);
			}
			InstancePtr->Stats.SendBytes++;
		}

		// polling iic interrupts
		Status = XIic_IrqService(InstancePtr);

		if(Status != XST_SUCCESS){
			return XST_FAILURE;
		}
	}

	if(InstancePtr->Options == XII_REPEATED_START_OPTION){
		CntlReg |= XIIC_CR_REPEATED_START_MASK;
		XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);
	}

	return XST_SUCCESS;
}

static void ReadIrqSetup(XIic *InstancePtr){

	u32 mask;
	/*
	 * Clear/disable any pending Tx interrupts
	 */
	ClearIisrIrq(InstancePtr, XIIC_TX_INTERRUPTS);
	Iic_DisableIrq(InstancePtr, XIIC_TX_INTERRUPTS);
	mask = (XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_RX_FULL_MASK);
	IntrClearEnable(InstancePtr, mask);
}

static int Iic_Recv(XIic *InstancePtr, u8 BusAddress, u8 *Data) {
	XStatus Status;
	u32 LocalAddr;
	u32 CntlReg;
	u32 IntrPending;

	// set rx fifo
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_RFD_REG_OFFSET, 0);
	//len = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_RFO_REG_OFFSET) + 1;

	// clear tx and enable interrupt
	ReadIrqSetup(InstancePtr);

	LocalAddr = ((BusAddress << 1) & 0xFE) | (XIIC_READ_OPERATION);

	CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
	if(CntlReg & XIIC_CR_REPEATED_START_MASK){
		CntlReg &= ~XIIC_CR_DIR_IS_TX_MASK;
		CntlReg |= XIIC_CR_NO_ACK_MASK;
		XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET,CntlReg);

		XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);
	}
	else {
		CntlReg &= ~XIIC_CR_DIR_IS_TX_MASK;
		CntlReg |= (XIIC_CR_NO_ACK_MASK | XIIC_CR_MSMS_MASK);
		XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET,CntlReg);

		XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, LocalAddr);
	}

	CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
	while(1){
		IntrPending = XIic_ReadIisr(Iic.BaseAddress);
		if(IntrPending & (XIIC_INTR_TX_EMPTY_MASK)){
			CntlReg &= ~XIIC_CR_MSMS_MASK;
			XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);
			XIic_WriteReg(InstancePtr->BaseAddress, XIIC_DTR_REG_OFFSET, 0x1);
			break;
		}
		else if(IntrPending & XIIC_INTR_ARB_LOST_MASK){
			CntlReg &= ~XIIC_CR_MSMS_MASK;
			XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);
			XIic_WriteIisr(Iic.BaseAddress, XIIC_INTR_ARB_LOST_MASK);
		}
	}

	CntlReg = XIic_ReadReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET);
	CntlReg |= XIIC_CR_MSMS_MASK;
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);
	// wait for rx fifo
	Status = XIic_IrqService(InstancePtr);

	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	*Data = (u8) XIic_ReadReg(InstancePtr->BaseAddress, XIIC_DRR_REG_OFFSET);
	XIic_WriteReg(InstancePtr->BaseAddress, XIIC_RFD_REG_OFFSET, 0);

	return XST_SUCCESS;
}

