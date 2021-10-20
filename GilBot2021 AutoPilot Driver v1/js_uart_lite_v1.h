/***************************************************************************
송수신 인터럽트 처리된 시스템 헤더
2018-01-31 13:30

js_uart_v4.h의 참조에 따라 단순 글자모드 lite버전 생성

보레이트 초기화 과정에서 설정할 수 있도록
단순 글자모드에서 마지막 수신 글자를 초기화 과정에서 설정할 수 있도록
uart_init(serial_rx, baudrate)
baudrate는 AVR의 성능에 따라 처리가 가능함. (110~115200)
***************************************************************************/
#include <avr/interrupt.h>

#define		TIMEOUT_CNT			10
#define		REG_UDR				UDR0
#define		REG_UDRE			UDRE0
#define		PIN_UCSRA			UCSR0A

unsigned char rx_frame[64], out_buffer[64], next_data = 0;
unsigned char data_length = 0, frame_trancing = 0, uart_i = 0;
char last_word = 13; 

//콜백함수로 지정되어 있음.
static void (*rx_frame_active)(unsigned char *, unsigned char);

/* 데이터 배열
글자단위 시리얼 데이터 전송 함수 tx_data는 배열.
*/
void out_uart_char(unsigned char *tx_data, unsigned char tx_len);

/* 콜백함수명, 자신의ID, 전송모드
메인문에 콜백함수를 위한 함수명 작성 (Ex)
void serial_rx(unsigned char *rx_data, unsigned char len){
	수신 코드 완료 플래그 작성 (인터럽트 처리 부분 이므로 상세 프로그래밍을 금지함)
}
위의 함수명을 uart_init(serial_rx, baudrate, last_char)순으로 작성 및 실행
baudrate=AVR의 성능에 따라 좌우됨.
last_char=마지막 글자를 인식시켜 수신 콜백을 발생시킬 문자 지정(기본: 줄내림 13)*/
void uart_init(void (*recv)(unsigned char *, unsigned char), unsigned long baudrate, char last_char);

/*
시리얼 송/수신을 종료함. 절전모드 등을 수행하기 위해 필요할 것
*/
void uart_system_stop(void);

void uart_init(void (*recv)(unsigned char *, unsigned char), unsigned long baudrate, char last_char){
	unsigned int cal_burr = F_CPU/16/baudrate-1;
	rx_frame_active = recv;
	
// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART0 Mode: Asynchronous
	UCSR0A=0x00; UCSR0B=0xD8; UCSR0C=0x06;
	UBRR0H=cal_burr>>8; UBRR0L=cal_burr&0xFF;
	
    data_length = 0;
    frame_trancing = 0;
	last_word = last_char;
	
// Global enable interrupts
	asm volatile("sei");
}

void uart_system_stop(void){
	UCSR0A=0x00; UCSR0B=0x00; UCSR0C=0x00;
	UBRR0H=0x00; UBRR0L=0x00;
	
	data_length = 0;
	frame_trancing = 0;
}

void out_uart_char(unsigned char *tx_data, unsigned char tx_len){
	if(frame_trancing) return;
	
	UCSR0B=0x48; //Receive OFF (receive, interrupt off)
	
	for(uart_i=0; uart_i<tx_len; uart_i++){
		out_buffer[uart_i] = tx_data[uart_i];
	}
	frame_trancing = 1;
	data_length = uart_i;
	
	while(!(PIN_UCSRA&(1<<REG_UDRE))); //first send first byte.
	UDR0 = out_buffer[0];
	uart_i = 1;
}

ISR(USART_TX_vect){ // USART Transmitter interrupt service routine
    if(frame_trancing){
        UDR0 = out_buffer[uart_i++];
        if(uart_i >= data_length) frame_trancing = 0;
    }else{
		UCSR0B=0xD8; //Receive ON
        next_data = 0;
    }
}

ISR(USART_RX_vect){ // USART Receiver interrupt service routine
    rx_frame[next_data] = REG_UDR;
	if(rx_frame[next_data] == last_word){
		rx_frame_active(rx_frame, next_data);
		next_data = 0;
	}
	if(next_data >= 30) next_data = 0;
	else next_data++;
}
