

#ifndef IIC_DEMO_H_
#define IIC_DEMO_H_

/***************************** Include Files *********************************/
#include "xiic.h"
#include "xintc.h"

/************************** Function Prototypes *******************************/
static XStatus I2cInit(int DeviceId);
XStatus I2cReadA16D8(u8 PortId, u8 BusAddress, u16 Address, u8 *Data);
XStatus I2cWriteA16D8(u8 PortId, u8 BusAddress, u16 Address, u8 Data);
XStatus XIic_IrqService(XIic *InstancePtr);
static void ClearIisrIrq(XIic *InstancePtr, u32 Mask);
static void IntrClearEnable(XIic *InstancePtr, u32 Mask);
static void WriteIrqSetup(XIic *InstancePtr);
static void Iic_DisableIrq(XIic *InstancePtr, u32 Mask);

static int Iic_Send(XIic *InstancePtr, u8 BusAddress);
static int Iic_Recv(XIic *InstancePtr, u8 BusAddress, u8 *Data);
/************************** Variable Definitions *****************************/

/*
 * Array of masks associated with the bit position, improves performance
 * in the ISR and acknowledge functions, this table is shared between all
 * instances of the driver. XIN_CONTROLLER_MAX_INTRS is the maximum number of
 * sources of Interrupt controller
 */
u32 XIntc_BitPosMask[XIN_CONTROLLER_MAX_INTRS];

#endif /* IIC_DEMO_H_ */
