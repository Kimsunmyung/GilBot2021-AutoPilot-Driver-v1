
//#define		ADC_VREF_TYPE		0x20	//8bit mode
#define		ADC_VREF_TYPE		0x00	//10bit mode

unsigned char adc_reading = 0, adc_reading_ch = 0;
struct{
	unsigned int read;
	unsigned char enable;
} adc_dma[16];

//ch_bit is 0b15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0 channel
//0b0000000000000111 => ch 0, 1, 2 enable
//use check is ATmega328. (2020-03-09)
void adc_channel_init(unsigned int ch_bit){
	unsigned char i;
	if(ch_bit == 0){
		DIDR0 = 0x00;
		ADMUX = ADC_VREF_TYPE & 0xFF;
		ADCSRA = 0x00;
		return;
	}
	//8bit ADC mode enable
	DIDR0 = 0x00;
	ADMUX = ADC_VREF_TYPE & 0xFF;
	ADCSRA = 0x85;
	for(i=0; i<16; i++){
		if((ch_bit & 1) == 1) adc_dma[i].enable = 1;
		else adc_dma[i].enable = 0;
		ch_bit >>= 1;
		
		adc_dma[i].read = 0;
	}
	adc_reading = 0;
}

//non stoped while loop action ADC reading
//channel access is adc_dma[channel].read
void adc_read_run(void){
	unsigned char i = 0, y = 0;
	if(adc_reading){
		if((ADCSRA & 0x10)!=0){
			ADCSRA |= 0x10;
			//adc_dma[adc_reading_ch].read = ADCH; //8bit mode read
			adc_dma[adc_reading_ch].read = ADCW; //10bit mode read
			adc_reading = 0;
		}
		return;
	}
	adc_reading = 1;
	
	adc_reading_ch++;
	for(i=0, y=adc_reading_ch; i<16; i++, y++){
		if(y >= 16) y = 0;
		if(adc_dma[y].enable){
			adc_reading_ch = y;
			break;
		}
	}
	if(adc_reading_ch == 8) ADMUX = 0b11001000; //internal temperature
	else ADMUX = adc_reading_ch | (ADC_VREF_TYPE & 0xFF);
	
	ADCSRA |= 0x40;
}