#include <stdint.h>
#include "stm32l4xx.h"

void delay(volatile uint32_t time);

// Used to indicate wake-up in EXTI handler
volatile uint8_t wake_flag = 0;

void delay(volatile uint32_t time) 
{
    while (time--);
}

static void led_init(void)
{
    // Enable LED clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    
    // PA5 as output (LED)
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;    
}

static void gpio_init(void)
{
    // Enable GPIO clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    
    // PC13 as input with pull-up
    GPIOC->MODER &= ~GPIO_MODER_MODE13;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD13_0;
}

static void but_exti_init(void)
{
     // EXTI13 > PC13
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    // Unmask and configure falling edge trigger
    EXTI->IMR1 |= EXTI_IMR1_IM13;
    EXTI->FTSR1 |= EXTI_FTSR1_FT13;

    // Enable EXTI interrupt
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
}

static void css_exti_init(void)
{
    // Enable EXTI line 19 (CSS LSE interrupt)
    EXTI->IMR1 |= EXTI_IMR1_IM19;
    
    NVIC_EnableIRQ(TAMP_STAMP_IRQn);
}

static void exti_init(void)
{
   but_exti_init();
   css_exti_init();
}

static void syscfg_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

static void pwr_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;  
}
// Configure GPIOs and EXTI13
static void gpio_exti_init(void)
{  
    gpio_init();
    
    syscfg_init();
    
    pwr_init();

    exti_init();
}

// Enter STOP2 low-power mode
static void enter_stop_mode(void) 
{
    // Clear WUF flag
    PWR->SCR |= PWR_SCR_CWUF;

    // Set regulator to low-power in STOP mode
    PWR->CR1 |= PWR_CR1_LPMS_STOP2;

    // Set SLEEPDEEP bit
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    LPTIM1->CR |= LPTIM_CR_SNGSTRT;
    __WFI();  // Wait for interrupt (STOP mode entered)
}

static void timer_init(void)
{
    // Enable power interface and backup access
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    while (!(PWR->CR1 & PWR_CR1_DBP));
    
    // Reset backup domain
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    
    // Enable interrupts by CSS on LSE
    RCC->CIER |= RCC_CIER_LSECSSIE;
    
    // Enable LSE
    RCC->BDCR |= RCC_BDCR_LSEON;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));
    
    // Enable Clock Security System on LSE
    RCC->BDCR |= RCC_BDCR_LSECSSON; 
    
    // Enable LSI and wait for it to be ready
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));

    // Set LSE as LPTIM1 clock source
    RCC->CCIPR &= ~(RCC_CCIPR_LPTIM1SEL);
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL;

    // Enable LPTIM1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    // Use internal clock (from LSE via CCIPR) — not external pin
    LPTIM1->CFGR &= ~LPTIM_CFGR_CKSEL;

    // No prescaler
    LPTIM1->CFGR &= ~LPTIM_CFGR_PRESC;

    // Enable the timer
    LPTIM1->CR |= LPTIM_CR_ENABLE;

    // Set Auto Reload for 0.5s (LSE = 32768 Hz)
    LPTIM1->ARR = 16384; // 0.5 sec

    // Enable interrupt
    LPTIM1->IER |= LPTIM_IER_ARRMIE;
    NVIC_EnableIRQ(LPTIM1_IRQn);

    // Start continuous counting
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}


// Timer overflow interrupt
void LPTIM1_IRQHandler(void);
void LPTIM1_IRQHandler(void)
{
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        // Clear interrupt flag
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
        
        // Toggle LED
        if (GPIOA->ODR & GPIO_ODR_OD5)    // If LED is ON
        {
            GPIOA->BRR = GPIO_BRR_BR5;    // LED OFF
        } else                            // Otherwise
        {
            GPIOA->BSRR = GPIO_BSRR_BS5;  // LED ON
        }
    }
}

// LPTIM failure interrupt
void TAMP_STAMP_IRQHandler(void);
void TAMP_STAMP_IRQHandler(void)
{
    // Check if LSE CSS flag is set (LSE Clock Security System failure)
    if (RCC->CIFR & RCC_CIFR_LSECSSF) {
        // Clear the CSS failure flag by writing 1 to it
        RCC->CICR = RCC_CICR_LSECSSC;

        // Your recovery code here
        // For example, switch to LSI or flag an error
    }
}

// Button press interrupt (to turn off LSE and cause CSS interrupt)
void EXTI15_10_IRQHandler(void);
void EXTI15_10_IRQHandler(void) 
{
    if (EXTI->PR1 & EXTI_PR1_PIF13) {
        EXTI->PR1 |= EXTI_PR1_PIF13;  // Clear interrupt flag
        
        if (RCC->BDCR & RCC_BDCR_LSEON) // If LSE is ON
            RCC->BDCR &= ~RCC_BDCR_LSEON; // Turn off LSE 
    }
}

static void timer_IR_init(void)
{
    // Enable overflow interrupt
    LPTIM1->IER |= LPTIM_IER_ARRMIE;
    NVIC_EnableIRQ(LPTIM1_IRQn);
}
int main(void)
{
    // Enable LED
    led_init();
    
    // Set an LPTIM on lse clock with 0.5s interval
    timer_init();
    
    // Set interrupt on clock overflow
    timer_IR_init();
    
    // Set interrupt on pressing the button (to turn off LSE and cause CSS interrupt)
    gpio_exti_init();
    
    // LED OFF initially
    GPIOA->BRR = GPIO_BRR_BR5;
    while (1) {
        // Enter STOP2 mode
        enter_stop_mode();   
    }
}