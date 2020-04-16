
#ifndef _MAIN_H
#define _MAIN_H

#define NOT_DownSize

#define _Module_DMA   1
#define _Module_GPIO  1
#define _Module_SPI   1
#define _Module_UART  1
#define _Module_PWM   0
#define _Module_I2C   0
#define _Module_USB   1
#define _Module_ADC   0
#define _Module_FPS   1

/* Global Variables: */ 
#define SensorPort    2		//choice SPI1/SPI2

#if( EM32F867A )
  #define FPGA    1
  #define FPGA_XTAL   24000 
  
#else
  #define FPGA    0
  #define FPGA_XTAL   24000 
#endif

#include <string.h>
#include <stdlib.h>
#include <CMSDK_CM4.h>
#include <stdio.h>        // for strcpy, strcmp 
#include <stdbool.h>
#include "EM32F867.h"
#include "system.h"
#include "gpio.h"
#include "dma.h"
#include "spi.h"
#include "math.h"

#if( _Module_UART )
  #include "uart.h"
#endif  

#if( _Module_PWM )
#include "pwm.h"
#endif

#if( _Module_USB )
#include "usb.h"
#include "usb_reg.h"
#include "usb_def.h"
#include "usb_app.h"
#endif

#include "aes256.h"

#if( _Module_I2C )
#include "i2c.h"
#endif

#if( _Module_ADC )
#include "adc.h"
#endif


#if( _Module_FPS )
#include "user_FP1.h"
#include "user_IFCOM.h"

static uint8_t sensor_ESD_on;
static uint16_t sensor_baseavg;
static uint16_t sensor_maxlineoffset;

#endif



#define Null 0
                /* Value of a Null pointer */
#define true  1
#define false 0


#define TESTING
//#define UART2Console
//#define SPITEST
      
#define __nop10() __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
#define __nop100() __nop10();__nop10();__nop10();__nop10();__nop10();__nop10();__nop10();__nop10();__nop10();__nop10();
      
#define ERR_GPIOIO      0x00000004
#define ERR_GPIOINT     0x00000002
#define ERR_EXTINTA     0x00000008
#define ERR_EXTINTB     0x00000009
#define ERR_UART1RX     0x00000010
#define ERR_UART2RX     0x00000011 
#define ERR_UARTDMA     0x00000012     
#define ERR_T1EXTCNT    0x00000020
#define ERR_T2EXTEN     0x00000021
#define ERR_T3INTIMER   0x00000022
#define ERR_T4INTIMER   0x00000023
#define ERR_WDTCNT      0x00000024
#define ERR_RTCCNT      0x00000025
#define ERR_SENSORRD    0x00000030
#define ERR_SENSORWR    0x00000031
#define ERR_SENSORIMG   0x00000032
#define ERR_FLASHWR     0x00000040
#define ERR_REGGPIOA    0x00000050
#define ERR_REGGPIOB    0x00000051
#define ERR_REGDMA      0x00000052
#define ERR_REGTIMER1   0x00000053
#define ERR_REGTIMER2   0x00000054
#define ERR_REGTIMER3   0x00000055 
#define ERR_REGTIMER4   0x00000056
#define ERR_REGUART1    0x00000057
#define ERR_REGUART2    0x00000058
#define ERR_REGSSP1     0x00000059
#define ERR_REGSSP2     0x0000005a
#define ERR_REGWDT      0x0000005b
#define ERR_REGRTC      0x0000005c
#define ERR_REGBACKUP   0x0000005d
#define ERR_REGPWR      0x0000005e
#define ERR_REGGPIOAG   0x00000060
#define ERR_REGGPIOBG   0x00000061
#define ERR_REGDMAG     0x00000062
#define ERR_REGTIMER1G  0x00000063
#define ERR_REGTIMER2G  0x00000064
#define ERR_REGTIMER3G  0x00000065 
#define ERR_REGTIMER4G  0x00000066
#define ERR_REGUART1G   0x00000067
#define ERR_REGUART2G   0x00000068
#define ERR_REGSSP1G    0x00000069
#define ERR_REGSSP2G    0x0000006a
#define ERR_REGWDTG     0x0000006b
#define ERR_REGRTCG     0x0000006c
#define ERR_REGBACKUPG  0x0000006d
#define ERR_REGPWRG     0x0000006e
#define ERR_SWRESET     0x00000070
#define ERR_SWRESETF    0x00000071
#define ERR_WDTRESET    0x00000072
#define ERR_WDTRESETF   0x00000073
#define ERR_PWR0BACK    0x00000080
#define ERR_PWR0IDRAM1  0x00000081
#define ERR_PWR0SRAM0   0x00000082
#define ERR_PWR0SRAM1   0x00000083
#define ERR_PWR0TIMER1G 0x00000084
#define ERR_PWR0TIMER2  0x00000085
#define ERR_PWR0TIMER3  0x00000086
#define ERR_PWR1BACK    0x00000087
#define ERR_PWR1SRAM1   0x00000088
#define ERR_PWR1TIMER3  0x00000089
#define ERR_PWR2BACK    0x0000008a
#define ERR_PWR2IDRAM2  0x0000008b
#define ERR_PWR2SRAM0   0x0000008c
#define ERR_PWR2TIMER1G 0x0000008d
#define ERR_PWR2TIMER2  0x0000008e
#define ERR_PWR3BACKUP  0x0000008f
#define ERR_FLASHKEY    0x00000090
#define ERR_FLASH       0x00000091

      
#define SCRReg			(*( __IO uint32_t *)0xe000ed10)
#define ICSR				(*( __IO uint32_t *)0xe000ed04)
//#define ICSR				(*( __IO uint32_t *)0xe000ed04)
	

// global memory
#define mem0				(*( __IO uint32_t *)0x2003f800)
#define mem1				(*( __IO uint32_t *)0x2003f804)
#define mem2				(*( __IO uint32_t *)0x2003f808)
#define mem3				(*( __IO uint32_t *)0x2003f80c)
#define mem4				(*( __IO uint32_t *)0x2003f810)
#define mem5				(*( __IO uint32_t *)0x2003f814)
#define mem6				(*( __IO uint32_t *)0x2003f818)
#define mem7				(*( __IO uint32_t *)0x2003f81c)
#define mem8				(*( __IO uint32_t *)0x2003f820)
#define mem9				(*( __IO uint32_t *)0x2003f824)
#define mem10				(*( __IO uint32_t *)0x2003f828)
#define mem11				(*( __IO uint32_t *)0x2003f82c)
#define mem12				(*( __IO uint32_t *)0x2003f830)
#define mem13				(*( __IO uint32_t *)0x2003f834)
#define mem14				(*( __IO uint32_t *)0x2003f838)
#define mem15				(*( __IO uint32_t *)0x2003f83c)


// AES register
#define AESEKR0				(*( __IO uint32_t *)0x40015004)
	
// UDC register
#define UDCCtrl				(*( __IO uint32_t *)0x40005000)
#define FCCAFR_VAL		(*( __IO uint32_t *)0x40005408)	
	
// PWM register
#define PWM_PRDA			(*( __IO uint32_t *)0x4000c00c)


#define Timer0_CTRL				(*( __IO uint32_t *)0x40000000)
#define Timer0_VALUE			(*( __IO uint32_t *)0x40000004)
#define Timer0_RELOAD			(*( __IO uint32_t *)0x40000008)
#define Timer0_INTSTATUS	(*( __IO uint32_t *)0x4000000c)
	
#define Timer1_CTRL				(*( __IO uint32_t *)0x40001000)
#define Timer1_VALUE			(*( __IO uint32_t *)0x40001004)
#define Timer1_RELOAD			(*( __IO uint32_t *)0x40001008)
#define Timer1_INTSTATUS	(*( __IO uint32_t *)0x4000100c)
	
#define Timer2_CTRL				(*( __IO uint32_t *)0x40010000)
#define Timer2_VALUE			(*( __IO uint32_t *)0x40010004)
#define Timer2_RELOAD			(*( __IO uint32_t *)0x40010008)
#define Timer2_INTSTATUS	(*( __IO uint32_t *)0x4001000c)
	
#define Timer3_CTRL				(*( __IO uint32_t *)0x40011000)
#define Timer3_VALUE			(*( __IO uint32_t *)0x40011004)
#define Timer3_RELOAD			(*( __IO uint32_t *)0x40011008)
#define Timer3_INTSTATUS	(*( __IO uint32_t *)0x4001100c)

// I2C register
#define I2CON_0						(*( __IO uint32_t *)0x40005000)
#define I2CSAR_0					(*( __IO uint32_t *)0x40005004)
#define I2CON_1						(*( __IO uint32_t *)0x40014000)
#define I2CSAR_1					(*( __IO uint32_t *)0x40014004)	
//#define I2CON0						(*( __IO uint32_t *)0x40005000)
//#define I2CON0						(*( __IO uint32_t *)0x40005000)
//#define I2CON0						(*( __IO uint32_t *)0x40005000)
//#define I2CON0						(*( __IO uint32_t *)0x40005000)


//#define DMA_SAR0			(*( __IO uint32_t *)0x40035000)


// DMA register
#define DMA_SAR0			(*( __IO uint64_t *)0x40035000)
#define DMA_DAR0			(*( __IO uint64_t *)0x40035008)
#define DMA_LLP0			(*( __IO uint64_t *)0x40035010)
#define DMA_CTL0			(*( __IO uint64_t *)0x40035018)
#define DMA_CFG0			(*( __IO uint64_t *)0x40035040)
#define DMA_SGR0			(*( __IO uint64_t *)0x40035048)
#define DMA_DSR0			(*( __IO uint64_t *)0x40035050)
	
#define DMA_SAR1			(*( __IO uint64_t *)0x40035058)
#define DMA_DAR1			(*( __IO uint64_t *)0x40035060)
#define DMA_LLP1			(*( __IO uint64_t *)0x40035068)
#define DMA_CTL1			(*( __IO uint64_t *)0x40035070)
#define DMA_CFG1			(*( __IO uint64_t *)0x40035098)
#define DMA_SGR1			(*( __IO uint64_t *)0x400350a0)
#define DMA_DSR1			(*( __IO uint64_t *)0x400350a8)
	
#define DMA_SAR2			(*( __IO uint64_t *)0x400350b0)
#define DMA_DAR2			(*( __IO uint64_t *)0x400350b8)
#define DMA_LLP2			(*( __IO uint64_t *)0x400350c0)
#define DMA_CTL2			(*( __IO uint64_t *)0x400350c8)
#define DMA_CFG2			(*( __IO uint64_t *)0x400350f0)
#define DMA_SGR2			(*( __IO uint64_t *)0x400350f8)
#define DMA_DSR2			(*( __IO uint64_t *)0x40035100)	
	
#define DMA_SAR3			(*( __IO uint64_t *)0x40035108)
#define DMA_DAR3			(*( __IO uint64_t *)0x40035110)
#define DMA_LLP3			(*( __IO uint64_t *)0x40035118)
#define DMA_CTL3			(*( __IO uint64_t *)0x40035120)
#define DMA_CFG3			(*( __IO uint64_t *)0x40035148)
#define DMA_SGR3			(*( __IO uint64_t *)0x40035150)
#define DMA_DSR3			(*( __IO uint64_t *)0x40035158)	
	
#define DMA_SAR4			(*( __IO uint64_t *)0x40035160)
#define DMA_DAR4			(*( __IO uint64_t *)0x40035168)
#define DMA_LLP4			(*( __IO uint64_t *)0x40035170)
#define DMA_CTL4			(*( __IO uint64_t *)0x40035178)
#define DMA_CFG4			(*( __IO uint64_t *)0x400351a0)
#define DMA_SGR4			(*( __IO uint64_t *)0x400351a8)
#define DMA_DSR4			(*( __IO uint64_t *)0x400351b0)	
	
#define DMA_RawErr		(*( __IO uint64_t *)0x400352e0)
	
#define DMA_RawTfr			(*( __IO uint64_t *)0x400352c0)
#define DMA_RawBlock		(*( __IO uint64_t *)0x400352c8)
#define DMA_RawSrcTran	(*( __IO uint64_t *)0x400352d0)
#define DMA_RawDstTran	(*( __IO uint64_t *)0x400352d8)

#define DMA_StatusErr		(*( __IO uint64_t *)0x40035308)
	
#define DMA_MaskTfr			(*( __IO uint64_t *)0x40035310)
#define DMA_MaskBlock		(*( __IO uint64_t *)0x40035318)
#define DMA_MaskSrcTran	(*( __IO uint64_t *)0x40035320)
#define DMA_MaskDstTran	(*( __IO uint64_t *)0x40035328)

#define DMA_ClearTfr			(*( __IO uint64_t *)0x40035338)
#define DMA_ClearBlock		(*( __IO uint64_t *)0x40035340)
#define DMA_ClearSrcTran 	(*( __IO uint64_t *)0x40035348)
#define DMA_ClearDstTran	(*( __IO uint64_t *)0x40035350)
#define DMA_ClearErr			(*( __IO uint64_t *)0x40035358)
	

#define DMA_Cfg_R			(*( __IO uint64_t *)0x40035398)
#define DMA_ChEn_R		(*( __IO uint64_t *)0x400353a0)
#define DMA_ID_R			(*( __IO uint64_t *)0x400353a8)
#define DMA_TEST_R		(*( __IO uint64_t *)0x400353b0)


#define UARTDATA_0		(*( __IO uint32_t *)0x40003000)
#define UARTSTATE_0		(*( __IO uint32_t *)0x40003004)
#define UARTCTRL_0		(*( __IO uint32_t *)0x40003008)
#define UARTINTClr_0	(*( __IO uint32_t *)0x4000300c)
#define BAUDDIV_0			(*( __IO uint32_t *)0x40003010)
#define DMALengthL_0	(*( __IO uint32_t *)0x40003020)
#define DMALEN_CTRL_0	(*( __IO uint32_t *)0x40003024)
#define TInterval_0		(*( __IO uint32_t *)0x40003028)	
	
#define UARTCID_0			(*( __IO uint32_t *)0x40003ff0)
	
#define UARTDATA_1		(*( __IO uint32_t *)0x40012000)
#define UARTSTATE_1		(*( __IO uint32_t *)0x40012004)
#define UARTCTRL_1		(*( __IO uint32_t *)0x40012008)
#define UARTINTClr_1	(*( __IO uint32_t *)0x4001200c)
#define BAUDDIV_1			(*( __IO uint32_t *)0x40012010)	
#define DMALengthL_1	(*( __IO uint32_t *)0x40012020)
#define DMALEN_CTRL_1	(*( __IO uint32_t *)0x40012024)
#define TInterval_1		(*( __IO uint32_t *)0x40012028)

#define MIRCCTRL_CS		(*( __IO uint32_t *)0x40036000)
  
void SPIInit(void);	
void SPITest(void);	
void SPITest0(void);	
void SPITest1(void);
void SPIMasterTest(void);
void ChkSPIBusy1(void);
void ChkSPIBusy0(void);
//void SPIBurstRead1(void);
//void SPIBurstRead0(void);
void DMATest(void);
//void SysTick_Init(void);
void SystemInitial(void);
//void ParameterInit2(void);
void CLKOUTInit(void);
void WDTTest(void);
void TimerTest(void);
void PowerMKodeTest01(void);
void PowerMKodeTest02(void);
void PowerMKodeTest03(void);
void CLKGating(void);
void CLKTree(void);
void RTC_Test(void);

void UARTTest( uint32_t BaudRate );
void Remap(void);
void RemapTest(void);
unsigned int MemoryTest(void);
void FlashTest(void);
void AutoLoadTest(void);
void CLKTest(void);
void IAPTest( uint32_t testpattern );
void MasterSlaveSPI(void);


void Delay10us(void);

void NVIC_EnableIRQ(IRQn_Type IRQn);// a Enables an interrupt or exception.
void NVIC_DisableIRQ(IRQn_Type IRQn);// a Disables an interrupt or exception.
void NVIC_SetPendingIRQ(IRQn_Type IRQn);// a Sets the pending status of interrupt or exception to 1
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);// a Clears the pending status of interrupt or exception to 0.
uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn);// a Reads the pending status of interrupt or exception.
/*	
This function returns non-zero value if the pending status
is set to 1.
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) a Sets the priority of an interrupt or exception with
configurable priority level to 1.
uint32_t NVIC_GetPriority(IRQn_Type IRQn) a Reads the priority of an interrupt or exception with
configurable priority level.
This function return the current priority level.
*/


void ReadImage(void);
void FlowCTRL(void);
void IPRegTest(void);
void TestError( uint32_t ErrorWord );
void TimerCounterTest(void);
void SW_WDTRSTTest( void );
void ChkPowerMKode2(void);
void ToggleSecond( uint32_t second );
void SleepTest(void);


void ReMapOn( void);
void ReMapOff( void);
void Remap_800(void );
void ReMapOn2( void);
void CopyFlashToSysRAM(void );
void Remap3(void );

uint32_t MemoryTestNew( uint32_t *pMemAddr, uint32_t Length );

#if( _Module_USB )

void USBSetting(void);
//void UsbCommand( USBCommand Command );
void SSELESD_FN( uint8_t SSELESD_in );
void FP_exe_loop(void);

#endif

//void NormalToggle(uint32_t times, uint32_t delay)__attribute__((section(".ARM.__at_0x7c00")));;;
//void NormalToggle2(uint32_t times, uint32_t delay)__attribute__((section(".ARM.__at_0x7d00")));;;
void NormalToggle(uint32_t times, uint32_t delay);
void NormalToggle2(uint32_t times, uint32_t delay);
void IAPRWTest(uint32_t random, uint32_t *pflash, uint32_t page);
void SmallLoop( uint32_t counts );
void MODTest( uint32_t counts );
void PA8WakeUp( void );

void SPIDMA( void );
//void MemDMATest( void )__attribute__((section(".ARM.__at_0x7800")));;
void MemDMATest( void );
void CopyFlashToIDRAM(void );
void CopyFlashToSysRAM(void );
void PWSDTEST(void);
void ReMapOn4( void);

void RDWRProtect (void);

void PWMTest( void );
void Sensor1Init( void );

void RemapUART23INTTest( void );

uint32_t SWSub( uint32_t counts );
void SWTest( uint32_t counts );

void PowerMKodeTest00(void);
void PowerMKodeTest01(void);
void PowerMKodeTest02(void);

void CopyFlashToRAM(void );

void EXTWakeupTest(void);

void TreeCoding( void );

void PassWord_2ms( void );

#if( EM32F867A )
void pdrd_exe( void );// __attribute__((section(".ARM.__at_0x10007800")));
void pdrd_exe2( void );// __attribute__((section(".ARM.__at_0x10007c00")));
void LockTest( void );
void FlashLockAreaDisable( void );
void LockAreaRead(void);
void LockAreaRead_S(void);
void LockAreaTest(void)__attribute__((section(".ARM.__at_0x10006800")));;;
#else
void pdrd_exe( void );//__attribute__((section(".ARM.__at_0x7800")));
void pdrd_exe2( void );//__attribute__((section(".ARM.__at_0x7c00")));
#endif

#endif
