/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

#include "main.h"

//Interrupt service routine called by timer interrupt at frequency SAMPLE_RATE_HZ
void TC0_Handler(void)
{
	if(tc_get_status(TC0,CHANNEL0)){
		if (TOTAL_COUNTER > GOAL_SAMPLES)
		{
			fft_exe();
		}
		else {
			readRegister(Z_HIGH, &rx_data, 2);
			val = (q31_t)((rx_data[0]<<8)+rx_data[1]); //high << 8 + low
			val = (val << 30-READ_RES);
			
			holder[TOTAL_COUNTER] = val;
			holder[TOTAL_COUNTER+1] = (q31_t)0;
			
			toggle(PIO_PB11);
			TOTAL_COUNTER+=2;
	
			HIGH_COUNTER++;
			if(HIGH_COUNTER > HIGH_DIV) {
				HIGH_COUNTER = 0;
				pio_set(PIOB, PIO_PB12);				on_off_key();
			}//End if HIGH_COUNTER
			
		}//End else if TOTAL COUNTER
		
		NVIC_ClearPendingIRQ(TC0_IRQn);
	}//End get status
}//End handler

//  void PWM_Handler(void)
//  {
// 	static uint32_t ul_count = 0;  /* PWM counter value */
// 	static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */
// 	static uint8_t fade_in = 1;  /* LED fade in flag */
// 	uint32_t events = pwm_channel_get_interrupt_status(PWM);
// 
// 	/* Interrupt on PIN_PWM_LED0_CHANNEL */
// 	if ((events & (1 << PWM_CHANNEL_0)) == (1 << PWM_CHANNEL_0)) {
// 		ul_count++;
// 
// 		/* Fade in/out */
// 		if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
// 			/* Fade in */
// 			if (fade_in) {
// 				ul_duty++;
// 				if (ul_duty == PERIOD_VALUE) {
// 					fade_in = 0;
// 				}
// 			} else {
// 				/* Fade out */
// 				ul_duty--;
// 				if (ul_duty == INIT_DUTY_VALUE) {
// 					fade_in = 1;
// 				}
// 			}
// 
// 			/* Set new duty cycle */
// 			ul_count = 0;
// 			g_pwm_channel_led.channel = PWM_CHANNEL_0;
// 			pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
// 			g_pwm_channel_led.channel = PWM_CHANNEL_0;
// 			pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
// 		}
// 	}
//  }


int main (void)
{
	sysclk_init();
	board_init();
	
	// set the pins to use the uart peripheral
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(ID_PIOA);

	pio_configure(PINS_UART0_PIO, PINS_UART0_TYPE, PINS_UART0_MASK, PINS_UART0_ATTR);
	pio_configure(PINS_PWM0_PIO, PINS_PWM0_TYPE, PINS_PWM0_MASK, PINS_PWM0_ATTR);

	//enable the uart peripherial clock
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(PIOB); 

	const sam_uart_opt_t uart1_settings =
	{ sysclk_get_cpu_hz(), UART_SERIAL_BAUDRATE, UART_SERIAL_MODE };

	uart_init(UART0,&uart1_settings);      //Init UART1 and enable Rx and Tx
	 
	spi_init();
	
	//Initialize FFT Sampling
 	int status = 1;
	status = arm_cfft_radix4_init_q31(&fft_inst_fix, FFT_SIZE, 0, 1); //Selects forward transform and bit reversal of output
 	if (status == ARM_MATH_ARGUMENT_ERROR){
		console_write(ERROR_MESSAGE, sizeof(ERROR_MESSAGE));
 	}
	 else if(status == ARM_MATH_SUCCESS) {
		console_write(INIT_SUCCESS, sizeof(INIT_SUCCESS));
	 }
	
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO11;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12;
	
	//Disable watch dog
	WDT->WDT_MR |= WDT_MR_WDDIS; 
	
	//gpio_configure_pin(PIO_PB10_IDX, PIO_PERIPH_B|PIO_DEFAULT);
	pio_set_output(PIOB, PIO_PB10, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB11, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOB, PIO_PB12, HIGH, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA30, LOW, DISABLE, ENABLE);
	pio_set_output(PIOA, PIO_PA31, LOW, DISABLE, ENABLE);
	pio_set_input(PIOB, PIO_PB1, PIO_PULLUP);
	
	//Initizlize PWM
	pwm_init_start();
	
	//Initialzie Timer Interrupt
	tc_config(SAMPLE_RATE_HZ, TC0, CHANNEL0, ID_TC0);
	//tc_config(PULSE_RATE_HZ, TC0, CHANNEL1, ID_TC0);
	
	on_off_key();
	delay_ms(100);

	
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn,0);
 	NVIC_EnableIRQ(TC0_IRQn);
	 
	while(1) {
		gpio_set_pin_high(PIO_PB10_IDX);

		
		if(pio_get(PIOB, PIO_INPUT | PIO_PERIPH_B, PIO_PB1)) {
			toggle_motor(LRA_on_off);
			toggle(PIO_PB10);
		}
		
		 
	}//End while
}//End main

void console_write(char* input, int len){
	for(int i = 0; i < len; i++) {
		while(!uart_is_tx_buf_empty(UART0));
		uart_write(UART0, input[i]);
	}
	return;
}

void spi_init(void) {
	//Initialize SPI
	pmc_enable_periph_clk(ID_SPI);
	pio_configure(PINS_SPI_PIO, PINS_SPI_TYPE, PINS_SPI_MASK, PINS_SPI_ATTR);

	//Choose appropriate SPI_MODE_n
	// 	Mode CPOL NCPHA SHIFTEDGE CAPTUREEDGE INACTIVELEVEL
	// 	0 0 1 Falling Rising Low
	// 	1 0 0 Rising Falling Low
	// 	2 1 1 Rising Falling High
	// 	3 1 0 Falling Rising High

	spi_master_init(SPI);
	spi_master_setup_device(SPI, &CHANNEL_0, SPI_MODE_3, SPI_BAUD, SPIselect); //mySPIselect is not used
	spi_enable(SPI);

	writeRegister(26, 0x00); //Config reg
	writeRegister(28, 0x00); //Accel config
	writeRegister(29, 0x08); //Config2
	writeRegister(35, 0x00); //FIFO enable 0x08
	writeRegister(106, 0x50); //User control 0101 0000
	writeRegister(107, 0x00);//Power managment 0000 0000
	//writeRegister(108, 0x1F); //Disable all axis and gryo except ACC_X 0001 1111
	writeRegister(108, 0x00); //Enable all axis and gryo
}

void writeRegister(uint8_t reg, uint8_t val) {
	//uint8_t data_Tx = reg;
	spi_select_device(SPI, &CHANNEL_0);
	uint8_t data_Tx[2] = {reg, val};
	spi_write_packet(SPI, data_Tx, 2);
	//spi_write_single(SPI, reg);
	//spi_write_single(SPI, val);
	spi_deselect_device(SPI, &CHANNEL_0);
	return status;
}

status_code_t readRegister(uint8_t reg, uint8_t* val, int bytesToRead){
	uint8_t inByte = 0;
	uint8_t REG_ADDR_R = reg | 0x80;

	spi_select_device(SPI, &CHANNEL_0);
	//spi_write_single(SPI, REG_ADDR_R);
	status = spi_read_packet(SPI, val, bytesToRead);
	spi_deselect_device(SPI, &CHANNEL_0);
	return status;
}


// Configuration function for the Timer.
void tc_config(uint32_t freq_desired, Tc* TCn, uint32_t CHANNEL, uint32_t PER_ID)
{
	// INPUTS:
	//	freq_desired	The desired rate at which to call the ISR, Hz.
	
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t counts;	
	
	// Configure PMC				
	pmc_enable_periph_clk(PER_ID);

	// Configure TC for a 4Hz frequency and trigger on RC compare.
	tc_find_mck_divisor(
		(uint32_t)freq_desired,	// The desired frequency as a uint32. 
		ul_sysclk,				// Master clock freq in Hz.
		&ul_div,				// Pointer to register where divisor will be stored.
		&ul_tcclks,				// Pointer to reg where clock selection number is stored.
		ul_sysclk);				// Board clock freq in Hz.
	tc_init(TCn, CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	
	// Find the best estimate of counts, then write it to TC register C.
	counts = (ul_sysclk/ul_div)/freq_desired;	
	tc_write_rc(TCn, 0, counts);						

	// Enable interrupts for this TC, and start the TC.		
	tc_enable_interrupt(TCn, CHANNEL,TC_IER_CPCS);				// Enable interrupt.
	tc_start(TCn,CHANNEL);										// Start the TC.

}

void shiftLeft(void) {
  for(int i = 0; i < FFT_SIZE*2; i++)
  {
    samples_fix[i] = samples_fix[i+2];
  }
}

void copyArray(void) {
  for(int i = 0; i < FFT_SIZE*2; i++) {
    samples_hold[i] = samples_fix[i];
  }
}

void toggle(uint32_t LED_INPUT){
	if(TOGGLE_S){
		pio_set(PIOB, LED_INPUT);
		TOGGLE_S = 0;
	}
	else{
		pio_clear(PIOB, LED_INPUT);
		TOGGLE_S = 1;
	}//End else
	
}

void toggle_motor(uint32_t MOTOR_INPUT){
	if(TOGGLE_M){
		pio_set(PIOA, MOTOR_INPUT);
		TOGGLE_M = 0;
	}
	else{
		pio_clear(PIOA, MOTOR_INPUT);
		TOGGLE_M = 1;
	}//End else
	

}

void pwm_init_start(void) {
	//pio_set(PIOA, PIO_PA30);//ERM
	pio_set(PIOA, PIO_PA31);//LRA
		//Enable PIO
	pmc_enable_periph_clk(PIOA);
	pmc_enable_periph_clk(ID_PWM);
	
	pwm_channel_disable(PWM, PWM_CHANNEL_0);

	
	pwm_clock_t clock_setting = {
	.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
	.ul_clkb = 0,
	.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	
	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_init(PWM, &g_pwm_channel_led);
	
	NVIC_DisableIRQ(PWM_IRQn);
	NVIC_ClearPendingIRQ(PWM_IRQn);
	NVIC_SetPriority(PWM_IRQn, 0);
	NVIC_EnableIRQ(PWM_IRQn);

	/* Enable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM, PWM_CHANNEL_0, 0);
	//pwm_channel_enable_interrupt(PWM, PWM_CHANNEL_0, 0);
	pwm_channel_enable(PWM, PWM_CHANNEL_0);
	
}

void on_off_key(void){
	uint32_t ul_duty;
	if(!message[message_counter]){
		ul_duty = 20;
		TOGGLE_M = 0;
	}
	else{
		ul_duty = 50;
		TOGGLE_M = 1;
	}//End else
	
	g_pwm_channel_led.channel = PWM_CHANNEL_0;
	pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
	
}

// void on_off_key(void){
// 	uint32_t ul_duty;
// 	if(TOGGLE){
// 		ul_duty = 20;
// 		TOGGLE_M = 0;
// 	}
// 	else{
// 		ul_duty = 50;
// 		TOGGLE_M = 1;
// 	}//End else
// 	
// 	g_pwm_channel_led.channel = PWM_CHANNEL_0;
// 	pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
// 	
// }

void fft_exe(void){
	NVIC_DisableIRQ(TC0_IRQn);
	pio_clear(PIOA, PIO_PA31);//LRA
		
	for(int i =BOTTOM_BIN; i < TOP_BIN+1; i++) {
		char printer[20];
		sprintf(printer, "%d ,", i *BIN_SIZE);
		console_write(printer, sizeof(printer));
	}
		//console_write(BREAKER, sizeof(BREAKER));
		console_write("\r\n", 2);
	
	for(int i = 0; i < GOAL_SAMPLES-FFT_SIZE; i++) {
		//Disable timer interrupt so that the FFT can complete before being interrupted
		
		//console_write(BREAKER, sizeof(BREAKER));
		//sprintf(buffer, "EXECTUING FFT %d \r\n", i);
		//console_write(buffer, sizeof(buffer));
		//delay_ms(1000);
		
		  for(int it = 0; it < FFT_SIZE*2; it++) {
			samples_hold[it] = holder[i+it];
			//sprintf(buffer, "samples_hold[%d] = %d \r\n ", FFT_SIZE*i+it, samples_hold[it]);
			//console_write(buffer, sizeof(buffer));
		 }
// 		
// 		for(int j = 0; j++; j < FFT_SIZE*2) {
// 			samples_hold[j] = holder[FFT_SIZE*i+j];
// 			sprintf(buffer, "samples_hold[%d] = %d \r\n ", FFT_SIZE*i+j, samples_hold[j]);
// 			console_write(buffer, sizeof(buffer));
// 		}
		
		//Perform the FFT transform
		arm_cfft_radix4_q31(&fft_inst_fix, samples_hold);
			 
		// Calculate magnitude of complex numbers outq31put by the FFT.
 		arm_cmplx_mag_q31(samples_hold, magnitudes_fix, FFT_SIZE);
			
		//Copies magnitudes over to new array for finding max; only copy first half becuase the 
		//complex forier transform has even symmetry so only the first half of the data is relevent
		for (int i = 0; i < FFT_SIZE/2; ++i) {
			max_finder[i] = magnitudes_fix[i];
		}
		max_finder[0] = 0;
				
		//Uncomment to display the DC mangitude component of the signal
		//sprintf(print_buf, "magnitudes[0] \t %d \r\n", magnitudes_fix[0]);
		//console_write(print_buf, sizeof(print_buf));
			
		//Set the DC component of the magnitudes to zero; it tends to be larger than the other
		//frequency compenents, so it scews the max frequency finder
			
		for (int i = 0; i < (FILTER_LOW/BIN_SIZE); i++)
		{
			max_finder[i] = 0;
		}
		for (int i = FILTER_HIGH/BIN_SIZE; i < FFT_SIZE; i++){
			max_finder[i] = 0;
		}

			
		//Find the magnitude of the dominent frequency
		arm_max_q31(max_finder, FFT_SIZE, &max_val, &max_index);
			
		/*for(int i = 0; i < FFT_SIZE; i++) {
			sprintf(buffer, "%d - %d: %d", (SAMPLE_RATE_HZ/FFT_SIZE)*i, (SAMPLE_RATE_HZ/FFT_SIZE)*i+(SAMPLE_RATE_HZ/FFT_SIZE), magnitudes_fix[i]);
			console_write(buffer, sizeof(print_buf));
		}*/
	
		//Calulate the upper and lower range of the dominent frequency 
		dom_freq_l = BIN_SIZE*max_index;
		dom_freq_h = dom_freq_l + BIN_SIZE;
			
			
		/*for(int i = 0; i < FFT_SIZE; i++) {
			sprintf(buffer, "%d - %d\t", (SAMPLE_RATE_HZ/FFT_SIZE)*i, (SAMPLE_RATE_HZ/FFT_SIZE)*i+(SAMPLE_RATE_HZ/FFT_SIZE));
			console_write(buffer, sizeof(print_buf));
			console_write("\r\n", 2);
		}*/
// 		for(int i = 0; i < FFT_SIZE/2; i++) {
// 			sprintf(buffer, "magnitudes_fix[%d] = %d \r\n ", i, magnitudes_fix[i]);
// 			console_write(buffer, sizeof(buffer));
// 		}
		
		for(int i = BOTTOM_BIN; i < TOP_BIN+1; i++) {
			char printer[20];
			sprintf(printer, "%d, ", max_finder[i]);
			console_write(printer, sizeof(printer));
		}
		//console_write(BREAKER, sizeof(BREAKER));
		console_write("\r\n", 2);
		//delay_ms(1000);
		
		//Print the dominent frequency range
		//sprintf(print_buf, "%d - %d  @ %d Val: %d \r\n", dom_freq_l, dom_freq_h, max_index, max_val);
		//console_write(print_buf, sizeof(print_buf));
			
		//Uncomment to output accelerometer data
		//sprintf(buffer, "Z_HIGH: %x\tZ_LOW: %x\tZ_VAL: %d \r\n", rx_data[0], rx_data[1], total);
		//console_write(buffer, sizeof(buffer));
		}//End for i < FFT_NO
}