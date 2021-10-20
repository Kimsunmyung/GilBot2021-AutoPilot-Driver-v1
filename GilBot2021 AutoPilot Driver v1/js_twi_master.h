/************************************************************************
2020-09-27 02:55 TWI MASTER 인터럽트 타입 형식 사용준비.
************************************************************************/
#ifndef		F_CPU
#define		F_CPU			18432000UL
#endif

#include <util/twi.h>
#include <avr/interrupt.h>

#define		IOETR_TWIM_MAXBUF		32
#define		IOETR_TWIM_START		((1<<TWINT) | (1<<TWEA) | (1<<TWSTA) | (0<<TWSTO) | (1<<TWEN) | (1<<TWIE))
#define		IOETR_TWIM_STOP			((1<<TWINT) | (1<<TWEA) | (0<<TWSTA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE))
#define		IOETR_TWIM_ACK			((1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE))
#define		IOETR_TWIM_NACK			((1<<TWINT) | (0<<TWEA) | (1<<TWEN) | (1<<TWIE))
#define		IOETR_TWIM_REG_ADR		TWAR
#define		IOETR_TWIM_REG_CTR		TWCR
#define		IOETR_TWIM_REG_DAT		TWDR
#define		IOETR_TWIM_REGION		1

unsigned char bug_report = 0;

char js_twim_rxbuf[IOETR_TWIM_MAXBUF];
struct IOETR_TWIM_VAR {
	unsigned char com_state, calladr;
	unsigned char tx_hop, tx_len, rx_hop, rx_len;
	char tx_data[IOETR_TWIM_MAXBUF], rx_data[IOETR_TWIM_MAXBUF];
} ioetr_twim;
enum{
	IOETR_TWIM_STATE_IDLE = 0,
	IOETR_TWIM_STATE_START_MT,
	IOETR_TWIM_STATE_START_MR
};

//콜백함수로 지정되어 있음. 초기화 작업시 해당됨.
static void (*ioetr_twim_rxcall)(char *, unsigned char);

/* 콜백 함수 입력 및 통신클럭 입력 (해당포트 입력모드 설정, 내부풀업 필요시 설정)
콜백함수: void i2c_rx(unsigned char *rxdata, unsigned char rxlen)을 먼저 선언.*/
void js_twi_master_init(void (*recv)(char *, unsigned char), unsigned long int scl_freq);

/* i2c 단순 송신모드
타겟 주소와 보낼 프레임과 글자수로 지정됨.
정상적인 루틴이 시작될 경우 1이 반환되며, 통신을 점유중일 경우 0이 반환됨. */
unsigned char js_twim_tx(unsigned char address, char* tx_data, unsigned char tx_len);

/* i2c 단순 수신모드
타겟 주소와 수신하고자 하는 길이수로 지정됨. 단, 슬레이브단에서 길이수 이하의 값으로 NACK가 발생시
콜백함수로 보내어지는 수신데이터 길이는 수신 길이만큼 재조정됨.
정상적인 루틴이 시작될 경우 1이 반환되며, 통신을 점유중일 경우 0이 반환됨. */
unsigned char js_twim_rx(unsigned char address, unsigned char rx_len);

//램 로그 데이터 확인을 위한 임시 기능
/*unsigned char log_buf[32], log_hop = 0;
void signal_togle(unsigned char log_data){
	log_buf[log_hop++] = log_data;
	if(log_hop >= 32) log_hop = 0;
}*/

//Example: js_twi_master_init(i2c_rx, 100000);
void js_twi_master_init(void (*recv)(char *, unsigned char), unsigned long int scl_freq){
	unsigned char i;
	
	IOETR_TWIM_REG_ADR=0x00;
	IOETR_TWIM_REG_CTR=0x00;
	IOETR_TWIM_REG_DAT=0x00;
	for(i=0; i<250; i++) asm("nop");
	
	IOETR_TWIM_REG_ADR = 0;
	TWBR = ((F_CPU / scl_freq) - 16) / (2 * 1);
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	IOETR_TWIM_REG_CTR = IOETR_TWIM_ACK;
	
	ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
	
	ioetr_twim_rxcall = recv;
	asm("sei");
}
unsigned char js_twim_tx(unsigned char address, char* tx_data, unsigned char tx_len){
	unsigned char i;
	if(ioetr_twim.com_state != IOETR_TWIM_STATE_IDLE) return 0;
	IOETR_TWIM_REG_CTR = 0x00;
	ioetr_twim.com_state = IOETR_TWIM_STATE_START_MT;
	ioetr_twim.calladr = (address<<1);
	if(tx_len > IOETR_TWIM_MAXBUF) tx_len = IOETR_TWIM_MAXBUF;
	for(i=0; i<tx_len; i++){
		ioetr_twim.tx_data[i] = tx_data[i];
	}
	ioetr_twim.tx_len = tx_len;
	IOETR_TWIM_REG_CTR = IOETR_TWIM_START;
	return 1;
}
unsigned char js_twim_rx(unsigned char address, unsigned char rx_len){
	if(ioetr_twim.com_state != IOETR_TWIM_STATE_IDLE) return 0;
	IOETR_TWIM_REG_CTR = 0x00;
	ioetr_twim.com_state = IOETR_TWIM_STATE_START_MR;
	ioetr_twim.calladr = (address<<1) | 1;
	if(rx_len > IOETR_TWIM_MAXBUF) rx_len = IOETR_TWIM_MAXBUF;
	ioetr_twim.rx_len = rx_len;
	IOETR_TWIM_REG_CTR = IOETR_TWIM_START;
	return 1;
}
ISR(TWI_vect){
	unsigned char i, ack_ctrl = IOETR_TWIM_STOP;
	switch(TWSR & 0xF8){
		case TW_START:
		IOETR_TWIM_REG_DAT = ioetr_twim.calladr;
		ack_ctrl = IOETR_TWIM_ACK;
		ioetr_twim.tx_hop = 0;
		ioetr_twim.rx_hop = 0;
		break;
		
		#if IOETR_TWIM_REGION //Master Transmitter
		case TW_MT_SLA_ACK:
		case TW_MT_DATA_ACK:
		if(ioetr_twim.tx_len > ioetr_twim.tx_hop){
			IOETR_TWIM_REG_DAT = ioetr_twim.tx_data[ioetr_twim.tx_hop++];
			ack_ctrl = IOETR_TWIM_ACK;
		}
		else{
			ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		}
		break;
		
		case TW_MT_SLA_NACK:
		case TW_MT_DATA_NACK:
		ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		break;
		
		case TW_MT_ARB_LOST:
		ack_ctrl = IOETR_TWIM_ACK;
		ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		break;
		#endif
		
		#if IOETR_TWIM_REGION //Master Receiver
		case TW_MR_SLA_ACK:
		if(ioetr_twim.rx_len > 0){
			ack_ctrl = IOETR_TWIM_ACK;
		}
		else{
			ack_ctrl = IOETR_TWIM_NACK;
		}
		break;
		
		case TW_MR_SLA_NACK:
		ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		break;
		
		case TW_MR_DATA_ACK:
		ioetr_twim.rx_data[ioetr_twim.rx_hop++] = IOETR_TWIM_REG_DAT;
		if(ioetr_twim.rx_len > ioetr_twim.rx_hop){
			ack_ctrl = IOETR_TWIM_ACK;
		}
		else{
			ack_ctrl = IOETR_TWIM_NACK;
		}
		break;
		
		case TW_MR_DATA_NACK:
		for(i=0; i<ioetr_twim.rx_hop; i++){
			js_twim_rxbuf[i] = ioetr_twim.rx_data[i];
		}
		ioetr_twim_rxcall(js_twim_rxbuf, i);
		ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		break;
		#endif
		
		default:
		ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
		break;
	}
	
	IOETR_TWIM_REG_CTR = ack_ctrl;
}
