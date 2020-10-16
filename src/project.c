#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/dma.h>
#include <libopencmsis/core_cm3.h>
#define LED_COUNT (78)
#define LEDS_NUM                39
#define WS2812_BIT_BUFFER_SIZE (24 * LEDS_NUM)

#include <libopencm3/cm3/cortex.h>
#include <stdint.h>
#include <libopencm3/cm3/sync.h>

volatile uint32_t pulse_width = 0;
volatile uint32_t last_captured = 0;
volatile uint32_t signal_polarity = 1;

volatile uint32_t message = 0; 
volatile int currentbit = 0;

uint32_t current_captured = 0;
void tim3_isr(void)
{	
	if(TIM_SR(TIM3) & TIM_SR_CC2IF){
		current_captured = TIM_CCR2(TIM3);
		signal_polarity = 1 - signal_polarity;

		if(signal_polarity ==0){

			pulse_width = abs(current_captured - last_captured);
			if((((float)pulse_width/125000) > 0.00046) && (((float)pulse_width/125000) < 0.00066)){message &= ~(1<<(31-currentbit));currentbit++;}
			else if((((float)pulse_width/125000) > 0.0015) && (((float)pulse_width/125000) < 0.0019)){message |= 1<<(31-currentbit);currentbit++;}
		}
	
		last_captured = current_captured;

	}


	if((TIM_SR(TIM3) & TIM_SR_UIF)){
		timer_clear_flag(TIM3,TIM_SR_UIF);
		timer_clear_flag(TIM3,TIM_SR_CC2IF);
	}
}


void delay_timer_init(void)
{
	rcc_periph_clock_enable(RCC_TIM7);
	timer_set_prescaler(TIM7, 15);
	timer_disable_preload(TIM7);
	timer_one_shot_mode(TIM7);
	timer_update_on_overflow(TIM7);
	timer_enable_irq(TIM7, TIM_DIER_UIE);
	timer_clear_flag(TIM7, TIM_SR_UIF);		
	nvic_set_priority(NVIC_TIM7_IRQ, 2);
	nvic_enable_irq(NVIC_TIM7_IRQ);
}


static void delay_us(uint16_t val)
{
	if (!val)
		return;

	timer_set_period(TIM7, val);
	timer_generate_event(TIM7, TIM_EGR_UG);

	__dmb(); // data memory barrier (asm instuction DMB)
	// DMB acts as a data memory barrier. It ensures that all explicit memory accesses that
	// appear, in program order, before the DMB instruction are completed before any explicit
	// memory accesses that appear, in program order, after the DMB instruction. DMB does not
	// affect the ordering or execution of instructions that do not access memory.

	timer_enable_counter(TIM7);

	__asm__ volatile ("wfi");	// Enter sleep mode. WFI -- wait for interrupt
}


static void delay_ms(uint32_t val)
{
	while (val--)
	delay_us(1000);
}


void tim7_isr(void)
{
	timer_clear_flag(TIM7, TIM_SR_UIF);
}

void pwm_pin_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
    	gpio_mode_setup(GPIOE, GPIO_MODE_AF , GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOE,GPIO_AF1,GPIO9);
	gpio_set_output_options(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO9);
}

void pwm_timer_init(void)
{
	rcc_periph_clock_enable(RCC_TIM1);
	timer_disable_counter(TIM1);
	
	timer_direction_up(TIM1);
	timer_set_prescaler(TIM1, 159);
	
	timer_set_period(TIM1,999);
	timer_enable_oc_preload(TIM1,TIM_OC1);
	timer_enable_preload(TIM1);
	timer_generate_event(TIM1,TIM_EGR_UG);
	timer_set_oc_mode(TIM1,TIM_OC1,TIM_OCM_PWM1);
	
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_break_main_output(TIM1);
	timer_set_oc_value(TIM1,TIM_OC1,999);
	timer_enable_counter(TIM1);
	
	delay_timer_init();
	cm_enable_interrupts();
}

void pwm(void)
{
	pwm_pin_init();

	pwm_timer_init();

	int brightness = 0;
	int flag = 0;
	int delay = 0;
    	while (1) {
		
		if((brightness > 0)&&(flag == 1)){brightness= brightness - (1 + (int)(0.1*brightness));}
		else if(flag == 1){flag = 0;}
		
		if((brightness < 999)&&(flag == 0)){brightness+=1 + (int)(0.1*brightness);}
		else if(flag == 0){flag = 1;}
		
		timer_set_oc_value(TIM1,TIM_OC1,brightness);
		
		if(brightness == 0){delay = 1000000;}else
		if(brightness < 850){delay = (1000 - brightness)*100;}else
		if(brightness > 850){delay = 400000;}

		delay_ms((int)(delay/1000));
		
	}
}



typedef union {
      struct __attribute__ ((__packed__)) {
	uint8_t _unused;
	uint8_t b;
	uint8_t r;
	uint8_t g;
      } colors;
      uint32_t grbu;
} ws2812_led_t;

enum ws2812_stage {
	ws2812_idle,
	ws2812_sending,
	ws2812_done,
	ws2812_reset
};

struct ws2812_status {
	ws2812_led_t *leds;
	int led_count;
	int leds_sent;
	volatile enum ws2812_stage stage;
	uint16_t bit_buffer[WS2812_BIT_BUFFER_SIZE];
} ws2812_status;

struct led_status {
    ws2812_led_t leds[LED_COUNT];
    struct {
        int r;
        int g;
        int b;
    } dir[LED_COUNT];

    uint32_t timer;
} led_status;

/* Private function declarations. */

void ws2812_init(void);
void ws2812_dma_start(void);
void ws2812_dma_stop(void);
void ws2812_send(ws2812_led_t *leds, int led_count);
bool ws2812_is_sending(void);
void ws2812_gpio_init(void);
void ws2812_tim_init(void);
void ws2812_dma_init(void);
void ws2812_init_bit_buffer(void);
void ws2812_fill_low_bit_buffer(void);
void ws2812_fill_high_bit_buffer(void);
void ws2812_fill_bit_buffer(bool low_high);
void ws2812_clear_bit_buffer(void);
void clock_setup(void);
void leds_init(void);
void leds_run(void);

/* API functions. */
void ws2812_init(void)
{
	ws2812_gpio_init();
	ws2812_tim_init();
	ws2812_dma_init();

	ws2812_status.stage = ws2812_idle;
}



void ws2812_dma_start(void)
{
	dma_enable_stream(DMA1, DMA_STREAM6);
}

void ws2812_dma_stop(void)
{
	dma_disable_stream(DMA1, DMA_STREAM6);
}

void ws2812_send(ws2812_led_t *leds, int led_count)
{
	ws2812_status.leds = leds;
	ws2812_status.led_count = led_count;
	ws2812_status.leds_sent = 0;

	ws2812_init_bit_buffer();
	ws2812_dma_start();
}

bool ws2812_is_sending(void)
{
	return (ws2812_status.stage != ws2812_idle);
}

/* Private function implementations. */
void ws2812_init_bit_buffer(void)
{
	ws2812_fill_low_bit_buffer();
	ws2812_fill_high_bit_buffer();
}

void ws2812_fill_low_bit_buffer(void)
{
	ws2812_fill_bit_buffer(false);
}

void ws2812_fill_high_bit_buffer(void)
{
	ws2812_fill_bit_buffer(true);
}

void ws2812_fill_bit_buffer(bool low_high)
{
	int offset = 0;
	int bitcount = WS2812_BIT_BUFFER_SIZE / 2;
	int led = ws2812_status.leds_sent;
	int i;

	ws2812_status.stage = ws2812_sending;

	if(low_high) {
		offset = bitcount;
	}

	/*
	 * 60 = 1
	 * 29 = 0
	 */
	for(i = 0; i < bitcount; i++) {
		if (i < ((ws2812_status.led_count - ws2812_status.leds_sent) * 24)) {
			if (((ws2812_status.leds[ws2812_status.leds_sent + (i/24)].grbu >> (31 - (i % 24)))
				 & 0x00000001) != 0) {
				ws2812_status.bit_buffer[offset + i] = 60;
			} else {
				ws2812_status.bit_buffer[offset + i] = 29;
			}
			led = ws2812_status.leds_sent + ((i + 0) / 24);
		} else {
			ws2812_status.stage = ws2812_done;
			break;
		}
	}

	for(; i < bitcount; i++) {
			ws2812_status.bit_buffer[offset + i] = 0;
	}

	ws2812_status.leds_sent = led + 1;
}

void ws2812_clear_bit_buffer(void) {
	for(int i = 0; i < WS2812_BIT_BUFFER_SIZE; i++) {
		ws2812_status.bit_buffer[i] = 0;
	}
}

/* Hardware dependent code. */
void ws2812_gpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
    /* Not sure why this does not really do the job if we have a pullup to 5V */
    /* gpio_set_output_options(GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO12); */
}

void ws2812_tim_init(void)
{
	rcc_periph_clock_enable(RCC_TIM4);
    //timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 104); /* 168000000 / 2 / 800000 (800khz pwm) */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_preload(TIM4);

    timer_enable_irq(TIM4, TIM_DIER_UDE);

    timer_enable_counter(TIM4);
}

void ws2812_dma_init(void)
{
	rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6, (uint32_t)(&ws2812_status.bit_buffer[0]));
    dma_set_number_of_data(DMA1, DMA_STREAM6, WS2812_BIT_BUFFER_SIZE);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
}

/* Interrupt handlers. */
void dma1_stream6_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);

        gpio_toggle(GPIOD, GPIO13);

        if(ws2812_status.stage != ws2812_idle){
        	if(ws2812_status.stage == ws2812_done) {
        		ws2812_fill_low_bit_buffer();
        		ws2812_status.stage = ws2812_idle;
        	} else {
        		ws2812_fill_low_bit_buffer();
        	}
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);

        gpio_toggle(GPIOD, GPIO14);

        if(ws2812_status.stage != ws2812_idle){
            if(ws2812_status.stage == ws2812_done) {
        		ws2812_fill_high_bit_buffer();
        	    ws2812_status.stage = ws2812_idle;
            } else {
        	    ws2812_fill_high_bit_buffer();
            }
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
}

void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

void leds_init(void) {
    int i;
    int8_t color;
    int led_increment = (255/LED_COUNT) * 2;

    for(i=0; i < LED_COUNT; i++) {
      led_status.leds[i].grbu = 0;
    }

    for (i = 0; i < LED_COUNT; i++) {
#if 1
        color = -255 + (led_increment * i);
        if (color < 0) color = color * -1;
        led_status.leds[i].colors.r = color;
        if ((i % 6) < 3) {
            led_status.dir[i].r = 1;
        } else {
            led_status.dir[i].r = -1;
        }
#else
        led_status.leds[i].colors.r = 0;
        led_status.dir[i].r = 0;
#endif

#if 1
        color = -255 + (led_increment * (LED_COUNT / 3) ) + (led_increment * i);
        if (color < 0) color = color * -1;
        led_status.leds[i].colors.g = color;
        if ((i % 6) < 3) {
            led_status.dir[i].g = -1;
        } else {
            led_status.dir[i].g = 1;
        }
#else
        led_status.leds[i].colors.g = 0;
        led_status.dir[i].g = 0;
#endif

#if 0
        color = -255 + (led_increment * (LED_COUNT / 3) * 2) + (led_increment * i);
        if (color < 0) color = color * -1;
        led_status.leds[i].colors.b = color;
        if ((i % 6) < 3) {
            led_status.dir[i].b = 1;
        } else {
            led_status.dir[i].b = -1;
        }
#else
        led_status.leds[i].colors.b = 0;
        led_status.dir[i].b = 0;
#endif
    }

    led_status.timer = 0;

    ws2812_send(led_status.leds, LED_COUNT);
}

void leds_run(void) {
    int i;

    /* This time has to be at least 40us so that the led string is reset. */
    for(i=0; i<50000; i++){
        __asm("nop");
    }

    if(!ws2812_is_sending()) {
        for(i = 0; i < LED_COUNT; i++) {
            led_status.leds[i].colors.r += led_status.dir[i].r;
            if(led_status.dir[i].r != 0) {
                if(led_status.leds[i].colors.r == 255) led_status.dir[i].r = -1;
                if(led_status.leds[i].colors.r == 0) led_status.dir[i].r = 1;
            }
            led_status.leds[i].colors.g += led_status.dir[i].g;
            if(led_status.dir[i].g != 0) {
                if(led_status.leds[i].colors.g == 255) led_status.dir[i].g = -1;
                if(led_status.leds[i].colors.g == 0) led_status.dir[i].g = 1;
            }
            led_status.leds[i].colors.b += led_status.dir[i].b;
            if(led_status.dir[i].b != 0) {
                if(led_status.leds[i].colors.b == 255) led_status.dir[i].b = -1;
                if(led_status.leds[i].colors.b == 0) led_status.dir[i].b = 1;
            }
        }
        ws2812_send(led_status.leds, LED_COUNT);
    }    
}


void leds_rainbow (void) {
    int i;

    /* This time has to be at least 40us so that the led string is reset. */
    for(i=0; i<50000; i++){
        __asm("nop");
    }

    if(!ws2812_is_sending()) {
        for(i = 0; i < LED_COUNT; i++) {
        if((i >= 0 ) && (i <= LED_COUNT/3)) {
            led_status.leds[i].colors.r = 255 ;
            led_status.leds[i].colors.g = 0 ;
            led_status.leds[i].colors.b = 0 ;
        }
         if((i > LED_COUNT/ 3 ) && (i <= (2 *LED_COUNT)/3)) {
            led_status.leds[i].colors.r = 0 ;
            led_status.leds[i].colors.g = 255 ;
            led_status.leds[i].colors.b = 0 ;
        }
        if((i > (2*LED_COUNT)/ 3 ) && (i <= LED_COUNT)) {
            led_status.leds[i].colors.r = 0 ;
            led_status.leds[i].colors.g = 255 ;
            led_status.leds[i].colors.b = 0 ;
        }
    }
        ws2812_send(led_status.leds, LED_COUNT);  
}

}

void leds_pwm(void) {
    int i;

    /* This time has to be at least 40us so that the led string is reset. */
    for(i=0; i<50000; i++){
        __asm("nop");
    }

    if(!ws2812_is_sending()) {

        for(i = 0; i < LED_COUNT; i++) {
             led_status.leds[i].colors.r += led_status.dir[i].r;
            led_status.leds[i].colors.g += led_status.dir[i].g;
            led_status.leds[i].colors.b += led_status.dir[i].b;
            if((led_status.dir[i].r != 0) && (led_status.dir[i].g != 0) && (led_status.dir[i].b != 0)) {
                if((led_status.leds[i].colors.r == 255) && (led_status.leds[i].colors.g == 255) && (led_status.leds[i].colors.b == 255)) {
                led_status.dir[i].r = -1;
                led_status.dir[i].g = -1;
                led_status.dir[i].b = -1;
}
            if((led_status.leds[i].colors.r == 255) && (led_status.leds[i].colors.g == 255) && (led_status.leds[i].colors.b == 255)) {
                led_status.dir[i].r = 1;
                led_status.dir[i].g = 1;
                led_status.dir[i].b = 1;
}   
                
    }
    
    ws2812_send(led_status.leds, LED_COUNT);
}
}
}

void leds_running(int k) {
    int i;
    
    /* This time has to be at least 40us so that the led string is reset. */
    for(i=0; i<50000; i++){
        __asm("nop");
    }

    if(!ws2812_is_sending()) {

        for(i = 0; i < LED_COUNT; i++) {
             if ((i >= k ) && (i < k + 10)) {
            led_status.leds[i].colors.r = 0 ;
            led_status.leds[i].colors.g = 0 ;
            led_status.leds[i].colors.b = 255 ;

             }
             else {
             led_status.leds[i].colors.r = 255;
             led_status.leds[i].colors.g = 255;
             led_status.leds[i].colors.b = 255;
}
           
}  


        ws2812_send(led_status.leds, LED_COUNT);         
    }
    
   
}

void timer_for_ir_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOC,GPIO_AF2,GPIO7);

	rcc_periph_clock_enable(RCC_TIM3);
	timer_set_prescaler(TIM3, 127);
	timer_set_period(TIM3,0xFFFF);
	timer_direction_up(TIM3);//count up
	nvic_enable_irq(NVIC_TIM3_IRQ);
	cm_enable_interrupts();
	timer_disable_counter(TIM3);
	timer_enable_preload(TIM3);

	TIM_CCMR1(TIM3) |= TIM_CCMR1_CC2S_OUT;//clear bits
	timer_ic_set_filter(TIM3,TIM_IC2,TIM_IC_OFF);
	timer_ic_set_prescaler(TIM3,TIM_IC2,TIM_IC_PSC_OFF);

	TIM_CCMR1(TIM3) |= TIM_CCMR1_CC2S_IN_TI2;//CC2 is mapped on timer input 1
	
	timer_ic_set_polarity(TIM3,TIM_IC2,TIM_IC_BOTH );
	timer_ic_enable(TIM3,TIM_IC2);

	timer_ic_set_input(TIM3,TIM_IC2,TIM_IC_IN_TI2);
	timer_enable_irq(TIM3,TIM_DIER_CC2IE);//allow channel 2 of timer 3 to generate interrupts 
	timer_interrupt_source(TIM3,TIM_SR_CC2IF);
	timer_clear_flag(TIM3,TIM_SR_UIF);
	timer_enable_compare_control_update_on_trigger(TIM3);
	timer_enable_preload_complementry_enable_bits(TIM3);

	timer_enable_counter(TIM3);
}

int main(void)
{
    	int i;
    	int k = 0;
			

	timer_for_ir_init();

    	while (1) {
		switch (message)
		{
			case (0xFFA25D):
			timer_disable_counter(TIM3);
			clock_setup();
			ws2812_init();
			leds_init();
			while(1){
				leds_running(k);
				if(k == LED_COUNT/2) k = 0;
				else  k++;
				     
				for(i=0; i<9000000; i++){
					__asm("nop");
				}
			}
			break;
	
			case (0xFF629D):
			timer_disable_counter(TIM3);
			clock_setup();
			ws2812_init();
			leds_init();
			while(1){
				leds_pwm();
			}
			break;			

			case (0xFFE21D):
			timer_disable_counter(TIM3);
			clock_setup();
			ws2812_init();
			leds_init();
			while(1){
				leds_rainbow ();
			}
			break;

			case (0xFF22DD):
			timer_disable_counter(TIM3);
			clock_setup();
			ws2812_init();
			leds_init();
			while(1){
			leds_run();
			}
			break;
	
			default :
			break;
		} 
    
    }

    
   
    return 0;
}
