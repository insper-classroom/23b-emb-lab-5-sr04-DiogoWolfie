#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)



//pino de saída TRIG
#define PIN_PIO_T PIOA
#define PIN_PIO_ID_T ID_PIOA
#define PIN_PIO_PIN_T 2
#define PIN_PIO_MASK_T (1<<PIN_PIO_PIN_T)

//pino de entrada do ECHO
#define PIN_PIO_E PIOC
#define PIN_PIO_ID_E ID_PIOC
#define PIN_PIO_PIN_E 19
#define PIN_PIO_MASK_E (1<<PIN_PIO_PIN_E)
//flag do echo
volatile char echo_flag;
volatile char flag_rtt;

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)



//Queues
QueueHandle_t xQueueOLED;


//criando meus semárofos
SemaphoreHandle_t xSemaphoreBut;


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
static void	P_TRIG_INIT(void);
static void TRIG_10 (void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void echo_callback(void) {
	printf("echo callback foi chamado");
	if(pio_get(PIN_PIO_E, PIO_INPUT, PIN_PIO_MASK_E)){
		RTT_init(8500, 0, 0);
		printf("rtt iniciou\n");
	}else{
		uint32_t tempo =  rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueOLED, (void *)&tempo, 10);
		printf("rtt foi enviado\n");
	}
}

void but_callback(void){
	
	xSemaphoreGiveFromISR(xSemaphoreBut, 0);
	printf("botão apertado\n");

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("trequinho de sensor", 0, 0, &sysfont);
	gfx_mono_draw_string("oii", 0, 20, &sysfont);
	uint32_t msg;
	char str[10];

	for (;;)  {
		
		if(xSemaphoreTake(xSemaphoreBut, 1000)){
			printf("dei início ao trig\n");
			
			TRIG_10();
			printf("trig 10 foi chamado\n");
		}
		
		if (xQueueReceive(xQueueOLED, &msg, (TickType_t) 0)) {
			printf("recebi o valor do rtt\n");
			double som = 340;
			double distancia = msg*som/2;
			sprintf(str, "%d", distancia);
			gfx_mono_draw_string(str, 0, 20, &sysfont);
			
			
			}else{
			gfx_mono_draw_string("nao chegou", 0, 20, &sysfont);
			printf("não funcionou o rtt\n");
			
		}

	}
}




/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

static void	P_ECHO_INIT(void){
	pmc_enable_periph_clk(PIN_PIO_ID_E);
	pio_configure(PIN_PIO_E, PIO_INPUT, PIN_PIO_MASK_E, PIO_DEFAULT );
	//pio_set_input(PIN_PIO_E,PIN_PIO_MASK_E,PIO_DEFAULT);
	pio_enable_interrupt(PIN_PIO_E, PIN_PIO_MASK_E);
	pio_handler_set(PIN_PIO_E, PIN_PIO_ID_E, PIN_PIO_MASK_E, PIO_IT_EDGE , echo_callback);
	NVIC_EnableIRQ(PIN_PIO_ID_E);
	NVIC_SetPriority(PIN_PIO_ID_E, 4);

}

static void	P_TRIG_INIT(void){
	pmc_enable_periph_clk(PIN_PIO_ID_T);
	pio_set_output(PIN_PIO_T,PIN_PIO_MASK_T,0,0,0);
	
}

//deixa o trig em alta por 10us
static void TRIG_10 (void){
	
	pio_set(PIN_PIO_T, PIN_PIO_MASK_T);
	delay_us(10);
	pio_clear(PIN_PIO_T,PIN_PIO_MASK_T);
}

//RTT
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

//task echo





/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueOLED = xQueueCreate(100, sizeof(uint32_t));
	if (xQueueOLED == NULL)
	printf("falha em criar a queue xQueueOLED \n");
	
	//iniciando meus semáforos
	xSemaphoreBut = xSemaphoreCreateBinary();
	
	if (xSemaphoreBut == NULL)
	printf("falha em criar o semaforo \n");
	
	

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	printf("iniciei\n");
	P_TRIG_INIT();
	P_ECHO_INIT();
	BUT_init();
	
	/* Start the scheduler. */
	vTaskStartScheduler();
	

	/* RTOS não deve chegar aqui !! */
	while(1){}
	
	//como to usando rtos, não posso mais utilizar esse while(1)
	
	
	

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
