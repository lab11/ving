#include <asf.h>
#include "spi.h"
#include "spi_master.h"
#include "arm_math.h"
#include "pio.h"
#include "tc.h"

#define RX 1
#define TX 1

#define START_DELAY_TIME 0

//Rate of acceleromter sampling
#define SAMPLE_RATE_HZ 2000
//Rate of motor pulse
#define PULSE_RATE_HZ 20
//Size of FFT
#define FFT_SIZE 64
//Remove frequency bins about HIGH and below LOW
#define FILTER_LOW 100
#define FILTER_HIGH 500
//Set the samples to take before FFT
#define GOAL_SAMPLES 4000
//Set the bins to print
#define BOTTOM_BIN 6
#define TOP_BIN 6

//Message
#define MESSAGE_LEN 16
//const int message[MESSAGE_LEN] = { 1, 0, 1, 0, 0, 0, 1, 0 };
const int message[MESSAGE_LEN] = { 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0 };
int message_counter = 0;
int sample_counter = 0;
int samples_done = 0;
int message_to_send = 1;
q31_t holder[GOAL_SAMPLES];
q31_t RESONANT_BIN[GOAL_SAMPLES];
int message_trans[MESSAGE_LEN] = {0};
int summer[MESSAGE_LEN] = {0};

int BIT_HIGH_LEVEL = 0;
int BIT_LOW_LEVEL = 0;
int BIT_VARY = 0;
int MESSAGE_START_INDEX = 0;
int start_not_found = 1;
	
#define BIN_SIZE (SAMPLE_RATE_HZ/FFT_SIZE)
#define FFT_NO (GOAL_SAMPLES/FFT_SIZE)
#define BAUD_LEN_SAMPLES (SAMPLE_RATE_HZ/PULSE_RATE_HZ)*2
//#define BAUD_LEN_SAMPLES 250
#define READ_RES 16
	
//Channel definitions
#define CHANNEL0 0
#define CHANNEL1 1

int HIGH_COUNTER = 0;
int HIGH_DIV = (SAMPLE_RATE_HZ/PULSE_RATE_HZ);
int TOTAL_COUNTER = 0;

status_code_t status;
uint32_t SPIselect = 1;
int TOGGLE_S = 0;
int TOGGLE_M = 0;

#define ERROR_MESSAGE "INIT ERROR \r\n"
#define INIT_SUCCESS "INIT SUCCESS \r\n"
#define BREAKER "\r\n--------------------------------------\r\n"

// =============== UART1 =============== 
#define UART_SERIAL_BAUDRATE        57600
#define UART_SERIAL_CHANNEL_MODE	UART_MR_CHMODE_NORMAL
#define UART_SERIAL_MODE			UART_MR_PAR_NO


#define PINS_UART0          (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART0_MASK     (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_PIO      PIOA
#define PINS_UART0_ID       ID_PIOA
#define PINS_UART0_TYPE     PIO_PERIPH_A
#define PINS_UART0_ATTR     PIO_DEFAULT

//=================SPI==================
#define SPI_BAUD 4000000
#define Z_LOW 64
#define Z_HIGH 63
#define PINS_SPI          (PIO_PA12A_MISO | PIO_PA13A_MOSI|PIO_PA11A_NPCS0 |PIO_PA14A_SPCK)
#define PINS_SPI_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_SPI_MASK     (PIO_PA12A_MISO | PIO_PA13A_MOSI| PIO_PA11A_NPCS0 | PIO_PA14A_SPCK)
#define PINS_SPI_PIO      PIOA
#define PINS_SPI_ID       ID_PIOA
#define PINS_SPI_TYPE     PIO_PERIPH_A
#define PINS_SPI_ATTR     PIO_DEFAULT
struct spi_device CHANNEL_0 = {.id = 0};
char buffer[255];
char print_buf[255];
uint8_t rx_data[2];


//=================PWM============================
//PWM, pa0 LRA, pa1 ERM PWM
#define ERM_on_off PIO_PA30
#define LRA_on_off PIO_PA31

pwm_channel_t g_pwm_channel_led;

#define PINS_PWM0          (PIO_PA0A_PWMH0)
#define PINS_PWM0_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_PWM0_MASK     (PIO_PA0A_PWMH0)
#define PINS_PWM0_PIO      PIOA
#define PINS_PWM0_ID       ID_PIOA
#define PINS_PWM0_TYPE     PIO_PERIPH_A
#define PINS_PWM0_ATTR     PIO_DEFAULT

/** PWM frequency in Hz */
#define PWM_FREQUENCY      50000
/** Period value of PWM output waveform */
#define PERIOD_VALUE      100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    30 //40 is off, 30, is slightly on, 0 is full on


//=================FFT Globals================


q31_t samples_fix[FFT_SIZE*2];
q31_t samples_hold[FFT_SIZE*2];
q31_t magnitudes_fix[FFT_SIZE];
q31_t max_finder[FFT_SIZE];



volatile uint8_t high = 0;
volatile uint8_t low = 0;
q31_t val = 0;
q31_t max_val = 0;
uint32_t max_index = 0;
volatile int dom_freq_h = 0;
volatile int dom_freq_l = 0;
const arm_cfft_radix4_instance_q31 fft_inst_fix;
volatile int hasSampled = false;
volatile int isDone = false;
int sampleCounter = 0;


//=================Function Definitions================
void writeRegister(uint8_t reg, uint8_t val);
status_code_t readRegister(uint8_t reg, uint8_t* val, int bytesToRead);
void spi_init(void);
void console_write(char* input, int len);
void shiftLeft(void);
void copyArray(void);
void toggle(uint32_t LED_INPUT);
void toggle_motor(uint32_t MOTOR_INPUT);
void pwm_init_start(void);
void on_off_key(void);
void tc_config(uint32_t freq_desired, Tc* TCn, uint32_t CHANNEL, uint32_t PER_ID);
void fft_exe(void);
void moving_average(void);
void start_off(void);
void find_first_peak(void);
void system_clock_begin(void);

//=================Ring Buffer Definitions================
//q31_t ring_buffer[GOAL_SAMPLES];
int head = 0;
int tail = 0;
