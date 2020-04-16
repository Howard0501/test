
//#ifdef USBON

#include "CMSDK_CM4.h"
#include "main.h"
uint8_t ep1_data_in[64]={0};
uint8_t ep3_data_in[64]={0};
uint8_t multi_reg[64];
uint8_t single_reg[1];
uint8_t data_MemPtr[67];
uint8_t register_addr;
uint8_t xcor=0;
uint8_t ycor=0;
uint32_t data_len=0;
uint32_t remain=0;
uint8_t IOIRQ_CTRL=0;
uint16_t DATA_SIZE=0;
uint16_t flashdatalength=0;
uint8_t buffertemp[65535]__attribute__((at(0x20030000)));
uint8_t scan_status=0;
uint8_t AEC_STA=0;
uint8_t PDET=0;
uint8_t img_buf_ready=0;
uint8_t AP_FLAG=0;
uint16_t AP_TOTAL_SIZE=0;
uint16_t AP_CNT=0;
uint16_t AP_UPLOAD_DATA_SIZE=0;
/*om36*/
uint8_t data_array[12864];

void check_flash_busy(void)
{
	uint8_t RDATA[4]={0xAD,0x00,0x01,0x9C};
	uint8_t PAGE[3]={0x07,0x00,0x00};
	while(1)
	{
	
	SPIBurstWrite( SSP1,PAGE,  0x3 );//page
	SPIBurstWrite( SSP1,RDATA,  0x4 );//9c
	uint8_t polling_status[4]={0x17,0x80,0x02,0x05};
	SPIBurstWrite( SSP1,polling_status, 0x4 );
			uint16_t two_status[2];
			if( RequestDMAChannel( DMACH0 ) != DMACH0 )
			while(1);
			if( RequestDMAChannel( DMACH1 ) != DMACH1 )
			while(1);
			SSPEnable(SSP1);
			while(1)
			{
				SPIBurstRead( SSP1, CMD_STA, 2, two_status );
				if((two_status[1]&0x80)==0)
					break;
			}
			SPISendCommand( SSP1, 0x14 );
						PHERDMASetting(DMACH0,MEMORY_SPI1_TX,(uint32_t)two_status,0x2,SRC8DST8);
						PHERDMASetting(DMACH1,SPI1_MEMORY_RX,(uint32_t)two_status,0x2,SRC8DST8);
						DMAStart((DMAChannel)0);
						DMAStart((DMAChannel)1);
						__WFI();
						WaitDMADone((DMAChannel)0);
						WaitDMADone((DMAChannel)1);
			SSPDisable(SSP1);
			
			if(!(two_status[1]&0x01))
				break;
		}
	SPIBurstWrite( SSP1,PAGE,  0x3 );//page
	SPIBurstWrite( SSP1,RDATA,  0x4 );//9c
}
void flash_read(void)
{
		if(DATA_SIZE%16)
			remain=16-(DATA_SIZE%16);
		else
			remain=0;
	if(IOIRQ_CTRL & (0x02))
		{
			while(!scan_status)
			{
				while(IOIRQ_count)
				{
					scan_status=1;
					IOIRQ_count=0;
					if( RequestDMAChannel( DMACH0 ) != DMACH0 )
					while(1);
					if( RequestDMAChannel( DMACH1 ) != DMACH1 )
					while(1);
					SSPEnable(SSP1);
					SPISendCommand( SSP1, 0x14 );
					
						if(DATA_SIZE>100)
					{
					uint32_t REM=16;
					uint32_t REM_LEN=DATA_SIZE>>4;
					MassDMASetting(MSPI1TX,MSPI1RX,(uint32_t)buffertemp, (uint32_t)buffertemp, REM_LEN, REM, remain);
					MassDMAStart();
					__WFI();
					WaitMassDMADone();
					}
					else
					{
						PHERDMASetting(DMACH0,MEMORY_SPI1_TX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						PHERDMASetting(DMACH1,SPI1_MEMORY_RX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						DMAStart((DMAChannel)0);
						DMAStart((DMAChannel)1);
						__WFI();
						WaitDMADone((DMAChannel)0);
						WaitDMADone((DMAChannel)1);
					}
					
					SSPDisable(SSP1);
				}		
			}	
					scan_status=0;
		}
		
		else
			{
			uint16_t two_status[2];
			if( RequestDMAChannel( DMACH0 ) != DMACH0 )
			while(1);
			if( RequestDMAChannel( DMACH1 ) != DMACH1 )
			while(1);
			SSPEnable(SSP1);
			while(1)
			{
				SPIBurstRead( SSP1, CMD_STA, 2, two_status );
				if((two_status[1]&0x80)==0)
					break;
			}
			SPISendCommand( SSP1, 0x14 );
			
						if(DATA_SIZE>100)
					{
					uint32_t REM=16;
					uint32_t REM_LEN=DATA_SIZE>>4;
					MassDMASetting(MSPI1TX,MSPI1RX,(uint32_t)buffertemp, (uint32_t)buffertemp, REM_LEN, REM, remain);
					MassDMAStart();
					__WFI();
					WaitMassDMADone();
					}
					else
					{
						PHERDMASetting(DMACH0,MEMORY_SPI1_TX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						PHERDMASetting(DMACH1,SPI1_MEMORY_RX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						DMAStart((DMAChannel)0);
						DMAStart((DMAChannel)1);
						__WFI();
						WaitDMADone((DMAChannel)0);
						WaitDMADone((DMAChannel)1);
					}
					
			SSPDisable(SSP1);
			}
			EPX_TX(EP1, (uint8_t*)buffertemp,DATA_SIZE);
			UsbEpxStatusClr();
			
			img_buf_ready=0xff;
}

void read_img(void)
{
	
		if(DATA_SIZE%16)
			remain=16-(DATA_SIZE%16);
		else
			remain=0;
	if(IOIRQ_CTRL & (0x02))
		{
			while(!scan_status)
			{
				while(IOIRQ_count)
				{
					scan_status=1;
					IOIRQ_count=0;
					if( RequestDMAChannel( DMACH0 ) != DMACH0 )
					while(1);
					if( RequestDMAChannel( DMACH1 ) != DMACH1 )
					while(1);
					SSPEnable(SSP1);
					SPISendCommand( SSP1, 0x10 );
					if(DATA_SIZE>100)
					{
					uint32_t REM=16;
					uint32_t REM_LEN=DATA_SIZE>>4;
					MassDMASetting(MSPI1TX,MSPI1RX,(uint32_t)buffertemp, (uint32_t)buffertemp, REM_LEN, REM, remain);
					MassDMAStart();
					__WFI();
					WaitMassDMADone();
					}
					else
					{
						PHERDMASetting(DMACH0,MEMORY_SPI1_TX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						PHERDMASetting(DMACH1,SPI1_MEMORY_RX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						DMAStart((DMAChannel)0);
						DMAStart((DMAChannel)1);
						__WFI();
						WaitDMADone((DMAChannel)0);
						WaitDMADone((DMAChannel)1);
					}
					SSPDisable(SSP1);
				}		
			}	
					scan_status=0;
		}
		
		else
			{
			uint16_t two_status[2];
			if( RequestDMAChannel( DMACH0 ) != DMACH0 )
			while(1);
			if( RequestDMAChannel( DMACH1 ) != DMACH1 )
			while(1);
			SSPEnable(SSP1);
			while(1)
			{
				SPIBurstRead( SSP1, CMD_STA, 2, two_status );
				if(two_status[0]&0x10)
					break;
			}
			SPISendCommand( SSP1, 0x10 );
			if(DATA_SIZE>100)
					{
					uint32_t REM=16;
					uint32_t REM_LEN=DATA_SIZE>>4;
					MassDMASetting(MSPI1TX,MSPI1RX,(uint32_t)buffertemp, (uint32_t)buffertemp, REM_LEN, REM, remain);
					MassDMAStart();
					__WFI();
					WaitMassDMADone();
					}
					else
					{
						PHERDMASetting(DMACH0,MEMORY_SPI1_TX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						PHERDMASetting(DMACH1,SPI1_MEMORY_RX,(uint32_t)buffertemp,DATA_SIZE,SRC8DST8);
						DMAStart((DMAChannel)0);
						DMAStart((DMAChannel)1);
						__WFI();
						WaitDMADone((DMAChannel)0);
						WaitDMADone((DMAChannel)1);
					}
			SSPDisable(SSP1);
			}
			EPX_TX(EP1, (uint8_t*)buffertemp,DATA_SIZE);
			UsbEpxStatusClr();
			
			img_buf_ready=0xff;
}

void IO_CTRL_INIT(void)
{
	//SYSREGCTRL->POWEN = 1;
	//CLKGatingDisable(PCLKG_ALL);
	CLKGatingDisable( HCLKG_GPIOA );
	/*GPIO control PA6~PA10*/
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE4, GPIO_PuPd_Pulldown15K );
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE6, GPIO_PuPd_Pulldown15K );
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE7, GPIO_PuPd_Pulldown15K );
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE8, GPIO_PuPd_Pulldown15K );
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE9, GPIO_PuPd_Pulldown15K );
		GPIO_SetOutput( GPIOIPA, GPIO_PINSOURCE10, GPIO_PuPd_Pulldown15K );
		GPIO_SetInput(GPIOIPA,GPIO_PINSOURCE2,GPIO_PuPd_Pulldown15K);
		EnableGPIOINT(GPIOIPA,GPIO_PIN_2,RISING);
}
void usb_comm(void)
{
	uint8_t ep2_out_cmd[64];
	switch(UsbGetEpxStatus())
	{
		case USBD_EP2_OUT:
			UsbEpxGetCmd(EP2, ep2_out_cmd);
		
			if(ep2_out_cmd[0] == PROTOCOL_HEADER_OUT )	//check 40
			{
				switch(ep2_out_cmd[2])
				{
					case PROTOCOL_DECODE :
						ep1_data_in[0]=PROTOCOL_HEADER_IN;
						ep1_data_in[1]=0x01;
						ep1_data_in[2]=PROTOCOL_DECODE;
						EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
						break;	
					case DIRECT_CMD : 
						if(ep2_out_cmd[3] & 0xC0){
							register_addr = (ep2_out_cmd[3] & 0x3F);
							ep2_out_cmd[3] &= 0xC0;
						}
				
						ep1_data_in[0]=PROTOCOL_HEADER_IN;
						ep1_data_in[2]=DIRECT_CMD;
						
						switch(ep2_out_cmd[3])
						{
							case CMD_NOP:
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,CMD_NOP);
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;

							case CMD_STSC:
								AEC_STA=0x00;
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,CMD_STSC);
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;

							case CMD_STA:
							{
								uint16_t two_status[2];
								ep1_data_in[1]=0x03;
								SPIBurstRead( SSP1, CMD_STA, 2, two_status );
								ep1_data_in[3]=two_status[0];
								ep1_data_in[4]=two_status[1];
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							}
							case CMD_STOP:
								PDET=0;
								img_buf_ready=0x00;
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,CMD_STOP);
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;

							case CMD_FUSE_LOAD:
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,CMD_FUSE_LOAD);
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;

							case CMD_PAGE:
							{
								uint8_t writedata[3]={0x07,0x00,ep2_out_cmd[4]};
								ep1_data_in[1]=0x01;
								SPIBurstWrite( SSP1,writedata,  3 );
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();			
								break;
							}
								
							case CMD_SRST:
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,CMD_SRST);
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;
							
							case FLASHRW:
								ep1_data_in[1]=0x1;
								ep1_data_in[2]=0x1;
							
								
								//SPIBurstWrite( SSP1,&ep2_out_cmd[3],  DATA_SIZE+3 );
								
								if((!AP_FLAG)&&(ep2_out_cmd[4]&0x80))
								{
									DATA_SIZE=((ep2_out_cmd[4]&0x7f)<<8)+ep2_out_cmd[5];
									SPIBurstWrite( SSP1,&ep2_out_cmd[3],  ep2_out_cmd[1]-1 );
									//IOIRQ_CTRL=0x02;
									flash_read();
								}
									
								else
								{
									uint8_t write_flash_cmd=0;
									if(!AP_FLAG)
									{
										DATA_SIZE=((ep2_out_cmd[4]&0x7f)<<8)+ep2_out_cmd[5];
										write_flash_cmd=ep2_out_cmd[6];
										AP_FLAG=1;
										AP_TOTAL_SIZE=DATA_SIZE+3;
										buffertemp[AP_CNT++]=FLASHRW;
										AP_TOTAL_SIZE-=1;
									}
									
									for(int i=4;i<64;i++)
											buffertemp[AP_CNT++]=ep2_out_cmd[i];
									
									if(AP_TOTAL_SIZE>60)
										AP_TOTAL_SIZE-=60;
									else
										AP_TOTAL_SIZE=0;
									
									if(AP_TOTAL_SIZE==0)
									{
										if(DATA_SIZE>4)
											check_flash_busy();
										
										SPIBurstWrite( SSP1,buffertemp, DATA_SIZE+3 );
										AP_FLAG=0;
										AP_CNT=0;
										write_flash_cmd=0;
									}
									EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
									UsbEpxStatusClr();
								}
								break;
								
							case CMD_READ:
							{
								uint16_t multi;
								ep1_data_in[1]=0x02;
								SPIBurstRead( SSP1, register_addr+0x40, 0x1, &multi );
								ep1_data_in[3]=multi;
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							}

							case CMD_WRITE:
							{
								uint8_t writedata[64]={0};
								ep1_data_in[1]=0x01;
								writedata[0]=register_addr+0x80;
								writedata[1]=0x0;
								writedata[2]=ep2_out_cmd[1]-2;//after ap update this one=ep2_out_cmd[1]-2;
								
								for( int i=0; i<ep2_out_cmd[1]-2; i++ )
								{
								writedata[3+i]=ep2_out_cmd[i+4];
								}
								SPIBurstWrite( SSP1,writedata,  ep2_out_cmd[1]+1 );
								EPX_TX(EP1, (uint8_t *)ep1_data_in, 64);
								UsbEpxStatusClr();
								break;
							}
							case CMD_REG_BURST_READ:
								ep1_data_in[1]=ep2_out_cmd[1]-2;
								uint16_t multi[64];
								
									SPIBurstRead( SSP1, register_addr+0xC0, ep2_out_cmd[1]-3, multi );
									for (int i=0;i<64;i++)
									{
										if(i<3)
											multi_reg[i] = ep1_data_in[i];
										else if (i>2 && i<ep2_out_cmd[1])
											multi_reg[i] = multi[i-3];
										else
											multi_reg[i] = 0;
									}
								EPX_TX(EP1,(uint8_t *)multi_reg , 64);
							  UsbEpxStatusClr();
								break;
							
							case CMD_WIDTH:
							{
								uint16_t multi;
								ep1_data_in[1]=0x02;
								SPIBurstRead( SSP1, CMD_WIDTH, 0x1, &multi );
								ep1_data_in[3]=multi;
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							}

							case CMD_HEIGHT:
							{
								uint16_t multi;
								ep1_data_in[1]=0x02;
								SPIBurstRead( SSP1, CMD_HEIGHT, 0x1, &multi );
								ep1_data_in[3]=multi;
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							}
							
							case CMD_VERSION:
							{
								uint16_t multi;
								ep1_data_in[1]=0x02;
								SPIBurstRead( SSP1, CMD_VERSION, 0x1, &multi );
								ep1_data_in[3]=multi;
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							}

							case TRIMPAD_LOAD:
								ep1_data_in[1]=0x01;
								SPISendCommand(SSP1,TRIMPAD_LOAD);
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
								break;
							
							default:
								break;
						}
						break;

						case SPI_SPEED :
						{
							ep1_data_in[0]=PROTOCOL_HEADER_IN;
							ep1_data_in[1]=0x01;
							ep1_data_in[2]=SPI_SPEED;
							if(ep2_out_cmd[3]==0)//30
							{
								SetMainFreq2(IRCHIGH,IRCHIGH120,0);
							}
							else if(ep2_out_cmd[3]==1)//25
							{
								SetMainFreq2(IRCHIGH,IRCHIGH120,0);
								MIRCCTRL->MIRCTCF = 0x03; 
								MIRCCTRL->MIRCCA = 0x0A; 
							}
							
							
							else if(ep2_out_cmd[3]==3)//15
							{
								SetMainFreq2(IRCHIGH,IRCHIGH120,1);
							}
							
							else if(ep2_out_cmd[3]==4)//10
							{
								SetMainFreq2(IRCHIGH,IRCHIGH120,2);
								MIRCCTRL->MIRCTCF = 0x03; 
								MIRCCTRL->MIRCCA = 0x26; 
							}
							else
							{
								SetMainFreq2(IRCHIGH,IRCHIGH120,1);
								MIRCCTRL->MIRCTCF = 0x03; 
								MIRCCTRL->MIRCCA = 0x28; 
							}
								
							EPX_TX(EP1, ep1_data_in, 64);	 
							UsbEpxStatusClr();	
						break;
						}
							
						case CTRL_PORT_MODE :
						{
							ep1_data_in[0]=PROTOCOL_HEADER_IN;
							ep1_data_in[1]=0x02;
							ep1_data_in[2]=CTRL_PORT_MODE;
							uint32_t pin=1<<ep2_out_cmd[3];
							GPIO_WriteBit( GPIOIPA, pin, ep2_out_cmd[4]);
							ep1_data_in[3]=GPIO_ReadBit( GPIOIPA, pin );
							
							EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
							UsbEpxStatusClr();
						break;
						}
						
						case STATUS_POLLING :
						{
							uint16_t two_status[2]={0};
							uint16_t wrong_time[2]={0};
							ep1_data_in[1]=0x5;
							ep1_data_in[2]=STATUS_POLLING;
							uint16_t polling_cnt=(ep2_out_cmd[3]<<8)+ep2_out_cmd[4];
							for(int cnt=0;cnt<polling_cnt;cnt++)
							{
								SPIBurstRead( SSP1, CMD_STA, 2, two_status );
								if(two_status[0]!=ep2_out_cmd[5])
									wrong_time[0]++;
								if(two_status[1]!=ep2_out_cmd[6])
									wrong_time[1]++;
							}
							ep1_data_in[4]=(uint8_t)wrong_time[0];
							ep1_data_in[3]=wrong_time[0]>>8;
							ep1_data_in[6]=(uint8_t)wrong_time[1];
							ep1_data_in[5]=wrong_time[1]>>8;
							EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
							UsbEpxStatusClr();
							break;
						}
						case	AUTO_SENSE	:
						case	MANUAL_SENSE	:
						{
							uint8_t writedata[3]={0x07,0x00,0x00};
							SPIBurstWrite( SSP1,writedata,  3 );				//page0
							IOIRQ_CTRL=SSPReadData( SSP1, 0x6E);				//IRQ register
							
							
								if(IOIRQ_CTRL&0x06)
								{
								if(IOIRQ_CTRL&0x80)
								EnableGPIOINT(GPIOIPA,GPIO_PIN_2,FALLING);
								else 
								EnableGPIOINT(GPIOIPA,GPIO_PIN_2,RISING); 
								}
							
							DATA_SIZE=(ep2_out_cmd[3]<<8)+ep2_out_cmd[4];
							
							IOIRQ_count=0;
							SPISendCommand(SSP1,CMD_STSC);										//STSC cmd
							read_img();
							break;
						}
						
						case AUTO_SENSE_FE	:
							read_img();
							break;
						
						case AEC_REPORT:
							{
								ep1_data_in[2]=0x09; 
								uint16_t two_status[2];
								ep1_data_in[1]=0x03;
								SPIBurstRead( SSP1, CMD_STA, 2, two_status );
								ep1_data_in[3]=two_status[1];
								if(AEC_STA>0)
								{
								ep1_data_in[3]+=0x40;
								AEC_STA&=0;
								}
								EPX_TX(EP1,(uint8_t *)ep1_data_in , 64);
								UsbEpxStatusClr();
							}	
							break;
							
						case WOE_CHECK:
								while(1)
								{
									uint8_t woe;
									
									woe=SSPReadData( SSP1, 0x19);	
									if(woe!=0x06)
									GPIO_ToggleBits(GPIOIPA, GPIO_PIN_4);
									
									woe=SSPReadData( SSP1, 0x1A);	
									if(woe!=0xC0)
									GPIO_ToggleBits(GPIOIPA, GPIO_PIN_4);
									
									woe=SSPReadData( SSP1, 0x1F);	
									if(woe!=0x02)
									GPIO_ToggleBits(GPIOIPA, GPIO_PIN_4);
									
									woe=SSPReadData( SSP1, 0x20);	
									if(woe!=0xBF)
									GPIO_ToggleBits(GPIOIPA, GPIO_PIN_4);
									
								}
								break;
								
						case IMG_READY:
								PDET=1;
								
								UsbEpxStatusClr();
								break;
								
						case 0xAA:
							while(1)
							{
							SPISendData( SSP1, 0xAA, 0x5B );
							//Delay1us();
							SPISendData( SSP1, 0xAA, 0x4B );
								while(1)
								{
								uint16_t two_status[2];
								SPIBurstRead( SSP1, CMD_STA, 2, two_status );
									if(!(two_status[1]&0x10))
										break;
								}
							for(int i=0;i<10;i++)
							{
								if(SSPReadStatus( SSP1 )&0x04)
								{
									GPIO_WriteBit( GPIOIPA, GPIO_PIN_4, Bit_SET);
									GPIO_WriteBit( GPIOIPA, GPIO_PIN_4, Bit_RESET);
								}
							}
							/*GPIO_WriteBit( GPIOIPA, GPIO_PIN_10, Bit_SET);
							GPIO_WriteBit( GPIOIPA, GPIO_PIN_10, Bit_RESET);*/
							
							}
					default:
						break;
				}
			}
		default:
			break;
	}		
}
