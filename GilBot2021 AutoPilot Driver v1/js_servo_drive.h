
void servo_drive_init(unsigned char drive_mode);
void servo_drive_active(unsigned char ena, float degree);

unsigned char servo_drive_mode = 0;
float old_degree = 200.0;

void servo_drive_init(unsigned char drive_mode){
	servo_drive_mode = drive_mode;
	if(drive_mode == 2){
		//0.078125 degree resolution Mode
		// Timer/Counter 1 initialization
		// Clock source: System Clock
		// Clock value: 2304.000 kHz (46079 + 1 => 46080 Duty for 50Hz)
		// Mode: Fast PWM top=ICR1
		// OC1A output: Discon.
		// OC1B output: Inverted
		// Noise Canceler: Off
		// Input Capture on Falling Edge
		// Timer1 Overflow Interrupt: Off
		// Input Capture Interrupt: Off
		// Compare A Match Interrupt: Off
		// Compare B Match Interrupt: Off
		TCCR1A=0x32; TCCR1B=0x1A;
		TCNT1H=0x00; TCNT1L=0x00;
		ICR1H=0xB3; ICR1L=0xFF;
		OCR1AH=0x00; OCR1AL=0x00;
		OCR1BH=0xB3; OCR1BL=0xFF;
		
		// Timer/Counter 1 Interrupt(s) initialization
		TIMSK2=0x00;
	}
	else if(drive_mode == 1){
		//0.625 degree resolution Mode
		// Timer/Counter 1 initialization
		// Clock source: System Clock
		// Clock value: 288.000 kHz (5759 + 1 => 5760 Duty for 50Hz)
		// Mode: Fast PWM top=ICR1
		// OC1A output: Discon.
		// OC1B output: Inverted
		// Noise Canceler: Off
		// Input Capture on Falling Edge
		// Timer1 Overflow Interrupt: Off
		// Input Capture Interrupt: Off
		// Compare A Match Interrupt: Off
		// Compare B Match Interrupt: Off
		TCCR1A=0x32; TCCR1B=0x1B;
		TCNT1H=0x00; TCNT1L=0x00;
		ICR1H=0x16; ICR1L=0x7F;
		OCR1AH=0x00; OCR1AL=0x00;
		OCR1BH=0x16; OCR1BL=0x7F;
		
		// Timer/Counter 1 Interrupt(s) initialization
		TIMSK2=0x00;
	}
	
}

void servo_drive_active(unsigned char ena, float degree){
	unsigned int level = 0;
	if(servo_drive_mode == 2){
		if(ena == 0){
			old_degree = 200.0;
			OCR1BH=0xB3; OCR1BL=0xFF;
			return;
		}
		
		if(degree < 0.0 || degree > 180.0) return;
		
		//level = degree * 12.8 + 2304; //1ms duty product
		level = degree * 25.6 + 1152; //1152 ~ 5760 for DM-S2000MD (500~2500us)
		level = 46079 - level;
	}
	else if(servo_drive_mode == 1){
		if(ena == 0){
			OCR1BH=0x16; OCR1BL=0x7F;
			old_degree = 200.0;
			return;
		}
		
		if(degree < 0.0 || degree > 180.0) return;
		
		//level = degree * 1.6 + 173; //1ms duty product
		level = degree * 3.2 + 144; //144 ~ 720 for DM-S2000MD (500~2500us)
		level = 5759 - level;
	}
	
	if(old_degree != degree){
		old_degree = degree;
		OCR1BH=(level>>8); OCR1BL=(level&0xFF);
	}
}