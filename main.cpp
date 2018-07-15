#include "msp.h"
#include "main.hpp"
#include "Scheduler.hpp"
#include "Mailbox.hpp"
#include "Task.hpp"
#include "LED.hpp"
#include "LCD.hpp"
#include "LCD_DRIVER.hpp"
#include "Accelerometer.hpp"
#include "Joystick.hpp"

// ##########################
// Global/Static declarations
// ##########################
uint8_t Task::m_u8NextTaskID = 0; // - Init task ID
volatile static uint64_t g_SystemTicks = 0; // - The system counter.
Scheduler g_MainScheduler; // - Instantiate a Scheduler
// - Instantiate the new Mailbox
Mailbox g_MainMailbox;
// - Instantiate the mailbox pointer for Task and Scheduler class
Mailbox * Task::m_pMailbox = &g_MainMailbox;
Mailbox * Scheduler::m_pMailbox = &g_MainMailbox;
//The ID for the accelerometer task for the ADC14 IRQ
uint8_t g_u8AccelerometerTaskID;
extern volatile bool g_bState=false;
extern volatile int g_iTimeCounter=0;
extern volatile bool g_bButtonPressed=false;

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 3MHz
        45000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

// #########################
//          MAIN
// #########################
void main(void)
{
    // - Instantiate two new Tasks
    //LED BlueLED(BIT2);
    //LED GreenLED(BIT1);
    LCD SCREEN;
    Joystick PEN;
    PEN.SetLinkedTask(SCREEN.m_u8TaskID);
    // - Run the overall setup function for the system
    Setup();
    // - Attach the Tasks to the Scheduler;
    //g_MainScheduler.attach(&BlueLED, 300);
    g_MainScheduler.attach(&PEN, 300);
    g_MainScheduler.attach(&SCREEN, 300);

    // - Run the Setup for the scheduler and all tasks
    g_MainScheduler.setup();
    // - Main Loop
    while(1)
    {
    	__wfe(); // Wait for Event
        if(g_SystemTicks != g_MainScheduler.m_u64ticks)
        {
            //- Only execute the tasks if one tick has passed.
            g_MainScheduler.m_u64ticks = g_SystemTicks;
            g_MainScheduler.run();
        }
    }
}


// **********************************
// GPIO setup
// @input - none
// @output - none
// **********************************
void Init_GPIO(void) {
    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    //PMM_unlockLPM5();
}

// **********************************
// Setup function for the application
// @input - none
// @output - none
// **********************************
void Setup(void)
{
	// ****************************
	//         DEVICE CONFIG
	// ****************************
	// - Disable WDT
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

	    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);


	// ****************************
	//         PORT CONFIG
	// ****************************
	// - P1.0 is connected to the Red LED
	// - This is the heart beat indicator.
	P1->DIR |= BIT0; //Red LED

	// ****************************
	//       TIMER CONFIG
	// ****************************
	// - Configure Timer32_1  with MCLK (3Mhz), Division by 1, Enable the interrupt, Periodic Mode
	// - Enable the interrupt in the NVIC
	// - Start the timer in UP mode.
	// - Re-enable interrupts
	TIMER32_1->LOAD = TIMER32_COUNT; //~1ms ---> a 3Mhz
	TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
	NVIC_SetPriority(T32_INT1_IRQn,1);
	NVIC_EnableIRQ(T32_INT1_IRQn);
	__enable_irq();

	return;
}

extern "C"
{
    // - Handle the Timer32 Interrupt
	void T32_INT1_IRQHandler(void)
	{
		TIMER32_1->INTCLR = 0U;
		P1->OUT ^= BIT0; // - Toggle the heart beat indicator (1ms)
		g_SystemTicks++;
		return;
	}
	/* This interrupt is fired whenever a conversion is completed and placed in
	 * ADC_MEM2. This signals the end of conversion and the results array is
	 * grabbed and placed in resultsBuffer */


	void ADC14_IRQHandler(void)
	{
	    uint64_t status;

	    status = MAP_ADC14_getEnabledInterruptStatus();
	    MAP_ADC14_clearInterruptFlag(status);

	    /* ADC_MEM2 conversion completed */
	    if(status & ADC_INT2)
	    {

	    }
	}

	/*
	 * Port 1 interrupt handler. This handler is called whenever switches attached
	 * to P1.1 (S1) and P1.4 (S2) are pressed.
	 */
	void PORT1_IRQHandler(void)
	{
	    if(P1IN & GPIO_PIN1)
	    {
	        if(g_bButtonPressed){
	            // Set debounce flag on first high to low transition
	            g_bButtonPressed=false;
	        }

	        // Start debounce timer
	        Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
	    }
	}

	/*
	 * Timer A1 interrupt handler. This handler determines whether to reset button
	 * debounce after debounce timer expires.
	 */
	void TA1_0_IRQHandler(void)
    {
        // Button S2 released
        if (P1IN & BIT2)
        {
            g_bButtonPressed=true;
            if(g_bState){
                    //If 30s have passed then turn it off.
                    if(g_iTimeCounter==1000){
                        g_iTimeCounter=0;
                        Timer_A_stop(TIMER_A0_BASE);
                    } else {
                        g_iTimeCounter++;
                        Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
                    }
            } else Timer_A_stop(TIMER_A0_BASE);
        }
    }
}
