#include "CMSDK_CM4.h"


/*****user protocol*****/
#define PROTOCOL_HEADER_OUT		0x40
#define PROTOCOL_HEADER_IN		0x80
//Command list define
#define	PROTOCOL_DECODE					0x0
#define	DIRECT_CMD						0x1
#define	AUTO_SENSE				0x2
#define	MANUAL_SENSE			0x3
#define SPI_SPEED				0x4
#define	CTRL_PORT_MODE		0x5
#define STATUS_POLLING 0x6
#define AEC_REPORT 0x9
#define AUTO_SENSE_FE	0xFE
#define WOE_CHECK		0xEE
// Type = 0x01 (Direct command)
#define CMD_NOP					0x0
#define CMD_STSC				0x1
#define CMD_STA					0x3
#define CMD_FUSE_LOAD			0x4
#define CMD_STOP				0x5
#define CMD_PAGE				0x7
#define CMD_HEIGHT				0x8
#define CMD_WIDTH				0x9
#define CMD_VERSION				0xA
#define CMD_IMG_BURST_READ		0x10
#define FLASHRW	0x17
#define TRIMPAD_LOAD	0x27
#define CMD_SRST				0x31
#define CMD_READ				0x40
#define CMD_WRITE				0x80
#define CMD_REG_BURST_READ		0xC0
#define IMG_READY		0xBD
// subroutine
#define frame_mode	0xFA
#define line_mode 	0x4C

void usb_comm(void);
void IO_CTRL_INIT(void);
extern uint8_t xcor;
extern uint8_t ycor;
extern uint32_t data_len;
extern uint32_t remain;
extern uint8_t buffertemp[65535];
extern uint8_t AEC_STA;
extern uint8_t PDET;
extern uint8_t img_buf_ready;
extern uint8_t ep3_data_in[64];