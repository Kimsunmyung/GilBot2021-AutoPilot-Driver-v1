/***************************************************************************
�ۼ��� ���ͷ�Ʈ ó���� �ý��� ���
2018-01-31 13:30

js_uart_v4.h�� ������ ���� �ܼ� ���ڸ�� lite���� ����

������Ʈ �ʱ�ȭ �������� ������ �� �ֵ���
�ܼ� ���ڸ�忡�� ������ ���� ���ڸ� �ʱ�ȭ �������� ������ �� �ֵ���
uart_init(serial_rx, baudrate)
baudrate�� AVR�� ���ɿ� ���� ó���� ������. (110~115200)
***************************************************************************/
#include <avr/interrupt.h>

#define		TIMEOUT_CNT			10
#define		REG_UDR				UDR0
#define		REG_UDRE			UDRE0
#define		PIN_UCSRA			UCSR0A

unsigned char rx_frame[64], out_buffer[64], next_data = 0;
unsigned char data_length = 0, frame_trancing = 0, uart_i = 0;
char last_word = 13; 

//�ݹ��Լ��� �����Ǿ� ����.
static void (*rx_frame_active)(unsigned char *, unsigned char);

/* ������ �迭
���ڴ��� �ø��� ������ ���� �Լ� tx_data�� �迭.
*/
void out_uart_char(unsigned char *tx_data, unsigned char tx_len);

/* �ݹ��Լ���, �ڽ���ID, ���۸��
���ι��� �ݹ��Լ��� ���� �Լ��� �ۼ� (Ex)
void serial_rx(unsigned char *rx_data, unsigned char len){
	���� �ڵ� �Ϸ� �÷��� �ۼ� (���ͷ�Ʈ ó�� �κ� �̹Ƿ� �� ���α׷����� ������)
}
���� �Լ����� uart_init(serial_rx, baudrate, last_char)������ �ۼ� �� ����
baudrate=AVR�� ���ɿ� ���� �¿��.
last_char=������ ���ڸ� �νĽ��� ���� �ݹ��� �߻���ų ���� ����(�⺻: �ٳ��� 13)*/
void uart_init(void (*recv)(unsigned char *, unsigned char), unsigned long baudrate, char last_char);

/*
�ø��� ��/������ ������. ������� ���� �����ϱ� ���� �ʿ��� ��
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
