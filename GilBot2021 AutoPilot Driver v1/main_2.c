/*
D0: ToPC_RXD		C0: NTC_MOTOR			B0: SDRV_M0
D1: ToPC_TXD		C1: EXT_AD				B1: SDRV_ENA
D2: STATE_LED		C2:						B2: SDRV_STEP or SERVO PWM(1B)
D3:	ToPC_PWR		C3:						B3: DOWNLOAD
D4: SDRV_FAULT		C4: ToCarSDA			B4: DOWNLOAD
D5: SDRV_DIR		C5: ToCarSCL			B5: DOWNLOAD
D6: SDRV_M2			AD6:					B6: 18.432M
D7:	SDRV_M1			AD7:					B7: 18.432M

호환보드 [AI AutoPilot Driver v1] (Schematic, ArtWork, Firmware IOELECTRON Js)
ATmega328P (Ext. 18.432MHz)
퓨즈비트: [BOD 1.8V] [워치독 OFF] [외부크리스탈 모드]

서보드라이빙 모드일 경우 외부 보조 전원 피드백단 3k -> 47k 변경시 (4.9692v -> 6.7468v)
서보드라이빙 운영 전압: 4.8 ~ 7.2 [v] stall 2.5A 수준
*/
#define			F_CPU			18432000UL

#define			TEST_MODE		2

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "ServoDrive.h"
#include "IOETR_TWI_MASTER.h"
#include "js_uart_v4.h"

#define			TTLOUT_DIR_OUT(x)		(x==0?(PORTD&=~(1<<5)):(PORTD|=(1<<5)))
#define			TTLOUT_ENA_OUT(x)		(x==0?(PORTB|=(1<<1)):(PORTB&=~(1<<1)))
#define			TTLOUT_STP_OUT(x)		(x==0?(PORTB&=~(1<<2)):(PORTB|=(1<<2)))
#define			TTLOUT_M1_OUT(x)		(x==0?(PORTB&=~(1<<0)):(PORTB|=(1<<0)))
#define			TTLOUT_M2_OUT(x)		(x==0?(PORTD&=~(1<<7)):(PORTD|=(1<<7)))
#define			TTLOUT_M3_OUT(x)		(x==0?(PORTD&=~(1<<6)):(PORTD|=(1<<6)))
#define			TTLIN_SW_F				!(PINC&(1<<4))
#define			TTLIN_SW_B				!(PINC&(1<<5))

void inout_ddrset(unsigned char mode);
void system_init(void);

/************************************************************************
i2c_transmit(ai_mode, drv_mode, drv_throttle, lamp_act, signal_lamp, horn_act)
(0~3)	ai_mode: 0=자동화 사용 안함, 1=사용자 허가하에 사용, 2=강제사용, 3=없음
(0~3)	drv_mode: 0=리니어브레이크, 1=전진모드, 2=후진모드, 3=급브레이크
(0~127)	drv_throttle: drv_mode1과 2에만 반응하는 속도레벨
(0~1)	lamp_act: 0=램프 끄기, 1=램프 켜기
(0~3)	signal_lamp: 0=방향등X, 1=좌측방향등, 2=우측방향등, 3=비상등
(0~1)	horn_act: 0=크락션끄기, 1=크락션켜기
************************************************************************/

struct ai_control{
	unsigned int get_steer, get_steer_left, get_steer_right, get_steer_center, get_voltage;
	unsigned int get_hall_l, get_hall_r, cal_single_poles;
	unsigned char get_current_vector, get_current, get_brk_temperature;
	unsigned char get_lamp, get_signal, get_emb, get_aimode, get_drv_mode;
} from_gilbot;
unsigned char i2c_ret_cmd = 0, i2c_available = 0, i2c_timeout = 0, i2c_trx_loop = 0;
unsigned char i2c_tx_respone = 0, i2c_trx_mode = 0;
void i2c_transmit(unsigned char ai_mode, unsigned char drv_mode, unsigned char drv_throttle, unsigned char lamp_act, unsigned char signal_lamp, unsigned char horn_act);
void i2c_received(char *rx_data, unsigned char rx_len);
void i2c_processing(void);

struct ai_control_2{
	unsigned char set_drv_mode, set_drv_throttle, set_lamp, set_signal, set_aimode, set_horn;
	unsigned char set_steer_degree; //200 upper OFF
	unsigned int set_steer; //1500 upper OFF
} from_ctrl_pc;
unsigned char serial_available = 0, serial_rx_timeout = 0;
void serial_rx(unsigned int txid, unsigned int rxid, unsigned char *rx_data, unsigned char len);
void serial_processing(unsigned char len);

unsigned int old_hall_l = 0, old_hall_r = 0;
unsigned int speed_calculate(unsigned int read_hall_l, unsigned int read_hall_r);

unsigned int timer_1ms = 0, test_timing = 0;

int main(void){
	wdt_enable(WDTO_30MS);
	system_init();
    while (1){
		wdt_reset();
		if(i2c_available) i2c_processing();
		else{
			if(i2c_trx_loop >= 7){
				i2c_trx_loop = 0;
				if(i2c_tx_respone > 3){
					i2c_trx_mode = 0;
					ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
					bug_report++;
				}
				
				if(i2c_trx_mode){
					js_twim_rx(0x59, 7); //수신요청
				}
				else if(i2c_tx_respone > 3){
					i2c_tx_respone = 0;
					js_twim_tx(0x55, "1234", 2);
				}
				else{
					#if TEST_MODE == 0
					//ai_mode, drv_mode, drv_throttle, lamp_act, signal_lamp, horn_act
					i2c_transmit(from_ctrl_pc.set_aimode, from_ctrl_pc.set_drv_mode, from_ctrl_pc.set_drv_throttle, from_ctrl_pc.set_lamp, from_ctrl_pc.set_signal, from_ctrl_pc.set_horn);
					if(from_ctrl_pc.set_steer_degree <= 180){ //degree mode run
						servo_drive_active(from_ctrl_pc.set_aimode, from_ctrl_pc.set_steer_degree*1.0);
					}
					else if(from_ctrl_pc.set_steer <= 1023){ //steer level mode run
						servo_drive_active(from_ctrl_pc.set_aimode, from_ctrl_pc.set_steer_degree*1.0);
					}
					else{
						servo_drive_active(0, 90.0);
					}
					#elif TEST_MODE == 1
					if(test_timing > 3500){
						test_timing = 0;
						servo_drive_active(1, 30.0);
					}
					else if(test_timing > 3000){
						i2c_transmit(2, 1, 30, 1, 0, 1); //송신
						servo_drive_active(1, 150.0);
					}
					else if(test_timing > 2500){
						i2c_transmit(1, 0, 0, 0, 3, 0); //송신
						servo_drive_active(1, 30.0);
					}
					else if(test_timing > 2000){
						i2c_transmit(2, 0, 0, 1, 2, 1); //송신
						servo_drive_active(1, 150.0);
					}
					else if(test_timing > 1500){
						i2c_transmit(2, 0, 0, 0, 1, 0); //송신
						servo_drive_active(1, 30.0);
					}
					else if(test_timing > 1000){
						i2c_transmit(1, 2, 50, 1, 0, 1); //송신
						servo_drive_active(1, 150.0);
					}
					else if(test_timing > 500){
						i2c_transmit(1, 2, 30, 0, 0, 0); //송신
						servo_drive_active(1, 30.0);
					}
					else{
						i2c_transmit(1, 1, 30, 1, 0, 1); //송신
						servo_drive_active(1, 150.0);
					}
					#elif TEST_MODE == 2
					servo_drive_active(1, 90.0);
					#endif
					i2c_tx_respone++;
				}
				i2c_trx_mode = !i2c_trx_mode;
			}
		}
		
// 		if(i2c_available) i2c_processing();
// 		else if(i2c_trx_loop >= 14 && i2c_trx_loop < 250){
// 			i2c_trx_loop = 0;
// 			js_twim_rx(0x59, 7); //수신요청
// 		}
// 		else if(i2c_trx_loop == 7){
// 			i2c_trx_loop++;
// 			
// 		}
// 		else if(i2c_timeout == 0){
// 			if(i2c_trx_loop >= 20){
// 				ioetr_twim.com_state = IOETR_TWIM_STATE_IDLE;
// 				i2c_trx_loop = 0;
// 			}
// 			
		if(timer_1ms >= 1000){
			timer_1ms = 0;
			from_gilbot.cal_single_poles = speed_calculate(from_gilbot.get_hall_l, from_gilbot.get_hall_r);
		}
		
		if(serial_available){
			serial_processing(serial_available);
		}
    }
}

void inout_ddrset(unsigned char mode){
	// D0: ToPC_RXD			C0: NTC_MOTOR			B0: SDRV_M0
	// D1: ToPC_TXD			C1: EXT_AD				B1: SDRV_ENA
	// D2: STATE_LED		C2:						B2: SDRV_STEP or SERVO PWM(1B)
	// D3: ToPC_PWR			C3:						B3: DOWNLOAD
	// D4: SDRV_FAULT		C4: ToCarSDA			B4: DOWNLOAD
	// D5: SDRV_DIR			C5: ToCarSCL			B5: DOWNLOAD
	// D6: SDRV_M2			AD6:					B6: 18.432M
	// D7: SDRV_M1			AD7:					B7: 18.432M
	if(mode == 0){ //init power down mode port ddr set.
		PORTB = 0b00000000; DDRB = 0b00000000;
		PORTC = 0b00000000; DDRC = 0b00000000;
		PORTD = 0b00000000; DDRD = 0b00000000;
	}
	else{ //init run power mode port ddr set.
		PORTB = 0b00000000; DDRB = 0b00000111;
		PORTC = 0b00110000; DDRC = 0b00000000;
		PORTD = 0b00011000; DDRD = 0b11101100;
	}
}
void system_init(void){
	inout_ddrset(1);
	
	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 18.000 kHz (17+1) 1kHz Loop: 1ms
	// Mode: CTC top=OCR0A
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A=0x02; TCCR0B=0x05;
	TCNT0=0x00; OCR0A=17; OCR0B=0x00;
	
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=0x02;
	
	servo_drive_init(2);
	js_twi_master_init(i2c_received, 100000);
	uart_init(serial_rx, '\r', 0, 38400);
	
	out_uart_char((unsigned char *)"SYSTEM BOOT\r", 12);
}

void i2c_transmit(unsigned char ai_mode, unsigned char drv_mode, unsigned char drv_throttle, unsigned char lamp_act, unsigned char signal_lamp, unsigned char horn_act){
	//반환요청값(8), 자동화모드(2), 방향등(2), 부저(2), 전후드라이빙(2), 전조등(1), 스로틀레벨(7) [00000000 11122344 56666666]
	char out_buffer[8], chkdata;
	
	out_buffer[0] = 192 + i2c_ret_cmd++;
	if(i2c_ret_cmd >= 3) i2c_ret_cmd = 0;
	
	out_buffer[1] = (ai_mode&0b00000011)<<6;
	out_buffer[1] |= (signal_lamp&0b00000011)<<4;
	out_buffer[1] |= (horn_act&0b00000011)<<2;
	out_buffer[1] |= (drv_mode&0b00000011);
	
	out_buffer[2] = (lamp_act&1)<<7;
	out_buffer[2] |= (drv_throttle&127);
	
	chkdata = out_buffer[0];
	chkdata ^= out_buffer[1];
	chkdata ^= out_buffer[2];
	out_buffer[3] = chkdata;
	
	js_twim_tx(0x59, out_buffer, 4);
}
void i2c_received(char *rx_data, unsigned char rx_len){
	i2c_available = rx_len;
}
void i2c_processing(void){
	unsigned char i, chkdata=js_twim_rxbuf[0];
	unsigned int int_temp = 0;
	
	for(i=1; i<i2c_available-1; i++){
		chkdata ^= js_twim_rxbuf[i];
	}
	if(chkdata != js_twim_rxbuf[i]){
		i2c_available = 0;
		return;
	}
	i2c_tx_respone = 0;
	i2c_timeout = 100;
	
	switch(js_twim_rxbuf[0]){
		case 192:
		//자동화 허용상태 [2bit], 드라이빙 상태 [2bit], 전자브레이크 동작 상태 [2bit], 방향등 상태 [2bit]
		//배터리 전류 [8bit]
		//현재 스티어 레벨 [10bit], 배터리 전압 [12bit], 배터리 충방상태 [1bit], 전조등 상태 [1bit]
		from_gilbot.get_aimode = (js_twim_rxbuf[1]&0b11000000)>>6;
		from_gilbot.get_drv_mode = (js_twim_rxbuf[1]&0b00110000)>>4;
		from_gilbot.get_emb = (js_twim_rxbuf[1]&0b00001100)>>2;
		from_gilbot.get_signal = (js_twim_rxbuf[1]&0b00000011);
		from_gilbot.get_current = js_twim_rxbuf[2];
		int_temp = js_twim_rxbuf[3];
		int_temp <<= 2;
		int_temp |= (js_twim_rxbuf[4]&0b11000000)>>6;
		from_gilbot.get_steer = int_temp;
		int_temp = (js_twim_rxbuf[4]&0b00111111);
		int_temp <<= 6;
		int_temp |= (js_twim_rxbuf[5]&0b11111100);
		from_gilbot.get_voltage = int_temp;
		from_gilbot.get_current_vector = (js_twim_rxbuf[5]&0b00000010)>>1;
		from_gilbot.get_lamp = (js_twim_rxbuf[5]&0b00000001);
		break;
		
		case 193:
		//현재 스티어 레벨 [10bit], 스티어 좌측 제한 레벨 [10bit],
		//스티어 우측 제한 레벨 [10bit], 스티어 중간 레벨 [10bit]
		int_temp = js_twim_rxbuf[1];
		int_temp <<= 2;
		int_temp |= (js_twim_rxbuf[2]&0b11000000)>>6;
		from_gilbot.get_steer = int_temp;
		int_temp = (js_twim_rxbuf[2]&0b00111111);
		int_temp <<= 4;
		int_temp |= (js_twim_rxbuf[3]&0b11110000)>>4;
		from_gilbot.get_steer_left = int_temp;
		int_temp = (js_twim_rxbuf[3]&0b00001111);
		int_temp <<= 6;
		int_temp |= (js_twim_rxbuf[4]&0b11111100)>>2;
		from_gilbot.get_steer_right = int_temp;
		int_temp = (js_twim_rxbuf[4]&0b00000011);
		int_temp <<= 8;
		int_temp |= js_twim_rxbuf[5];
		from_gilbot.get_steer_center = int_temp;
		break;
		
		case 194:
		//현재 스티어 레벨[10bit], 좌 모터 홀 데이터 [12bit], 우 모터 홀 데이터 [12bit], 회생 브레이크 온도 [6bit]
		int_temp = js_twim_rxbuf[1];
		int_temp <<= 2;
		int_temp |= (js_twim_rxbuf[2]&0b11000000)>>6;
		from_gilbot.get_steer = int_temp;
		int_temp = (js_twim_rxbuf[2]&0b00111111);
		int_temp <<= 6;
		int_temp |= (js_twim_rxbuf[3]&0b11111100)>>2;
		from_gilbot.get_hall_l = int_temp;
		int_temp = (js_twim_rxbuf[3]&0b00000011);
		int_temp <<= 8;
		int_temp |= js_twim_rxbuf[4];
		int_temp <<= 2;
		int_temp |= (js_twim_rxbuf[5]&0b11000000)>>6;
		from_gilbot.get_hall_r = int_temp;
		from_gilbot.get_brk_temperature = js_twim_rxbuf[5]&0b00111111;
		break;
		
		default: break;
	}
	i2c_available = 0;
}

void serial_rx(unsigned int txid, unsigned int rxid, unsigned char *rx_data, unsigned char len){
	serial_available = len;
}
void serial_processing(unsigned char len){
	unsigned char i, *rx_data = rx_buffer, temp_char, ret_cmd, str_temp[8];
	unsigned char cal_checkdata = rx_buffer[0], read_checkdata;
	unsigned int temp_int;
	
 	if(len < 17){
		 sprintf((char *)out_buffer, "DataLenERR\r");
		 out_uart_char((unsigned char *)out_buffer, strlen((char *)out_buffer));
		 serial_available = 0;
		 return; //본 함수에서 수신시 [13]데이터 제외됨
	 }
	//[18 Byte] Rx: 드라이브모드(1), 스로틀레벨(2), 핸들각도(3), 핸들각도INT모드(4), 시그널동작(1), 램프동작(1), 부저동작(1), 반환요청(1), 체크데이터(3), [13]
	//[20 Byte] Tx: 핸들각도(3), 핸들각도INT모드(4), 차량 좌우 합산 홀수(4), 반환명(1), 반환데이터(4), 체크데이터(3), [13]
	
	//체크 데이터 및 수신 체크 데이터 계산
	for(i=1; i<len-3; i++){
		cal_checkdata ^= rx_data[i];
	}
	read_checkdata = (rx_data[i]-'0')*100;
	read_checkdata += (rx_data[i+1]-'0')*10;
	read_checkdata += (rx_data[i+2]-'0');
	
	//체크 데이터 대조 확인
	if(cal_checkdata != read_checkdata){
		sprintf((char *)out_buffer, "CKERR.[C:%d][R:%d][L:%d]\r", cal_checkdata, read_checkdata, len);
		out_uart_char((unsigned char *)out_buffer, strlen((char *)out_buffer));
		serial_available = 0;
		return;
	}
	serial_rx_timeout = 100;
	
	//F:전진, R:후진, N:중립, B:비상정지 [0]
	switch(rx_data[0]){
		case 'F': from_ctrl_pc.set_drv_mode = 1; break;
		case 'R': from_ctrl_pc.set_drv_mode = 2; break;
		case 'B': from_ctrl_pc.set_drv_mode = 3; break;
		default: from_ctrl_pc.set_drv_mode = 0; break;
	}
	
	//00~99 스로틀 레벨 검출 [1, 2]
	temp_char = (rx_data[1] - '0') * 10;
	temp_char += (rx_data[2] - '0');
	from_ctrl_pc.set_drv_throttle = temp_char;
	
	//000~999 스티어 각도 모드 검출 [3, 4, 5]
	temp_char = (rx_data[3] - '0') * 100;
	temp_char += (rx_data[4] - '0') * 10;
	temp_char += (rx_data[5] - '0');
	from_ctrl_pc.set_steer = temp_char;
	
	//0000~9999 스티어 레벨 모드 검출 [6, 7, 8, 9]
	temp_int = (rx_data[6] - '0') * 1000;
	temp_int += (rx_data[7] - '0') * 100;
	temp_int += (rx_data[8] - '0') * 10;
	temp_int += (rx_data[9] - '0');
	from_ctrl_pc.set_steer = temp_int;
	
	//L:방향등좌, R:방향등우, E:방향등좌우, X:방향등끔 [10]
	switch(rx_data[10]){
		case 'L': from_ctrl_pc.set_signal = 1; break;
		case 'R': from_ctrl_pc.set_signal = 2; break;
		case 'E': from_ctrl_pc.set_signal = 3; break;
		default: from_ctrl_pc.set_signal = 0; break;
	}
	
	//O:라이트 켬, X:라이트 끔 [11]
	if(rx_data[11] == 'O') from_ctrl_pc.set_lamp = 1;
	else from_ctrl_pc.set_lamp = 0;
	
	//O:부저 켬, X:부저 끔 [12]
	if(rx_data[12] == 'O') from_ctrl_pc.set_horn = 1;
	else from_ctrl_pc.set_horn = 0;
	
	//반환요청 데이터 검출 [13]
	ret_cmd = rx_data[13] - '0';
	
	//[20 Byte] Tx: 핸들각도(3), 핸들각도INT모드(4), 차량 좌우 합산 홀수(4), 반환명(1), 반환데이터(4), 체크데이터(3), [13]
	
	switch(ret_cmd){
		case 1: temp_int = from_gilbot.get_steer_left; break; //핸들링 레벨 좌측 제한
		case 2: temp_int = from_gilbot.get_steer_center; break; //핸들링 레벨 중간 값
		case 3: temp_int = from_gilbot.get_steer_right; break; //핸들링 레벨 우측 제한
		case 4: temp_int = from_gilbot.get_voltage; break; //현재전압
		case 5: temp_int = from_gilbot.get_hall_l; break; //뒤좌측홀데이터
		case 6: temp_int = from_gilbot.get_hall_r; break; //뒤우측홀데이터
		case 7: temp_int = ((from_gilbot.get_current_vector&1)<<8)|from_gilbot.get_current; break; //전류레벨
		case 8: temp_int = from_gilbot.get_brk_temperature; break; //브레이크 온도
		case 9: //드라이브모드, 램프, 시그널, 전자브레이크
		temp_int = from_gilbot.get_drv_mode*1000;
		temp_int += from_gilbot.get_lamp*100;
		temp_int += from_gilbot.get_signal*10;
		temp_int += from_gilbot.get_emb;
		break;
		default: temp_int = 0; break;
	}
	
	sprintf((char *)out_buffer, "%c,%03d,%04d,%04d,%d,%04d,", i2c_timeout?'O':'X', 999, from_gilbot.get_steer, from_gilbot.get_hall_l+from_gilbot.get_hall_r, ret_cmd, temp_int);
	
	cal_checkdata = out_buffer[0];
	for(i=0; i<15; i++) cal_checkdata ^= out_buffer[i];
	
	sprintf((char *)str_temp, "%03d\r", cal_checkdata);
	
	strcat((char *)out_buffer, (char *)str_temp);
		
	out_uart_char(out_buffer, strlen((char *)out_buffer));
	serial_available = 0;
}

unsigned int speed_calculate(unsigned int read_hall_l, unsigned int read_hall_r){
	unsigned int temp_int = 0, read_polls = 0;
	if(old_hall_l < read_hall_l){
		temp_int += read_hall_l - old_hall_l;
	}
	else if(old_hall_l > read_hall_l){
		temp_int += 4095-old_hall_l+read_hall_l;
	}
	read_polls += temp_int;
	
	temp_int = 0;
	if(old_hall_r < read_hall_r){
		temp_int += read_hall_r - old_hall_r;
	}
	else if(old_hall_r > read_hall_r){
		temp_int += 4095-old_hall_r+read_hall_r;
	}
	read_polls += temp_int;
		
	old_hall_l = read_hall_l;
	old_hall_r = read_hall_r;
	
	return read_polls/2;
}

ISR(TIMER0_COMPA_vect){
	if(i2c_trx_loop < 255) i2c_trx_loop++;
	if(i2c_timeout) i2c_timeout--;
	if(timer_1ms < 65535) timer_1ms++;
	test_timing++;
}