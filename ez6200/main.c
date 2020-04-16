#include "main.h"
/* history 
DMA mark
    DMAREG->CLEARBLOCK = 0x01 << channel;
    DMAREG->CLEARTFR = 0x01 << channel;
    DMAINTStatus[channel] = 0;



*/
void usb_setting(void)
{
    EP_Open(EP1, EP1_PACKET_SIZE, USB_EP_BULK); 
    EP_Open(EP2, EP2_PACKET_SIZE, USB_EP_BULK);
    EP_Open(EP3, EP3_PACKET_SIZE, USB_EP_BULK);
    USB_Intr_Set(EP1, EP1IN_INT_EN); 
    USB_Intr_Set(EP2, EP1IN_INT_EN+EP2OUT_INT_EN);
    USB_Intr_Set(EP3, EP3IN_INT_EN);
    USB_Intr_Set(EP4, 0);
    USB_Intr_Set(EP0, (SETUP_INT_EN+EP0IN_INT_EN+EP0OUT_INT_EN));
}
void EP1UPLOAD(void)
{
	EP1ININT=0;
	uint16_t index, ep1_len;
	
	if((UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1DATREADY == 0) && (ep1_data_size != 0))
	{//GPIO_ToggleBits(GPIOIPA, GPIO_PIN_4);
		// EP1 Buffer Clear
		UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1BUFCLR = 1;
		UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1BUFCLR = 0;
		
		if(ep1_data_size > EP1_PACKET_SIZE)
		{
			EP1DATINOUTCNT = EP1_PACKET_SIZE;
			ep1_len = EP1_PACKET_SIZE;
		}
		else
		{
			EP1DATINOUTCNT = ep1_data_size;
			ep1_len = ep1_data_size;
		}
		
		for(index = 0 ; index < ep1_len ; index++ )
		{
			EP1BUFDATA = ep1_buffer_ptr[ep1_data_index++];
		}
		ep1_data_size -= ep1_len;
		UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1DATREADY = 1;
	}
	
	UDCEP1INTSTA->UDC_EP1_INTSTA.UDC_EP1_INT_STABIT.EP1_IN_INT_SF_CLR = 1;
}
int main (void)
{
	
	//SetMainFreq2(IRCLOW,IRCLOW32,DIV1);//8M
	SetMainFreq2(IRCHIGH,IRCHIGH120,DIV2);
	MIRCCTRL->MIRCTCF = 0x03;
	MIRCCTRL->MIRCCA = 0x04;
	SPI_Master_Init( SPI1, SPI_O0H0, 32000, SPI_CS_IO_FLOAT );
	IO_CTRL_INIT();
	USB_Clock_set(USB_IRC);
	usb_setting();
	
	PHYCTRL->USBPHYRSW = 0;
	Delay1ms();
	Delay1ms();

    USB_Init(0x01);
		//GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE14, GPIO_PuPd_Pulldown15K );
		//SetCLKOut(0,3,0); 
	  //LJIRCCTRL->LJIRCTMV12 = 2;
    while(1)
	{
		if((EP1ININT==1) && (ep1_data_size!=0))
			{
				
			 EP1UPLOAD();
			}
			else
			usb_comm();	 	
	}
	

}       

