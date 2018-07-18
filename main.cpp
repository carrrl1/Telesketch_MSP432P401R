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
// Defines
// ##########################
#define USE_ACCL 1

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
// Pointer to the LCD task
LCD * g_pLCD_Task;
//The ID for the accelerometer task for the ADC14 IRQ
uint8_t g_u8AccelerometerTaskID;
extern volatile bool g_bButtonDebounceS1=true;
extern volatile bool g_bButtonDebounceS2=true;
extern volatile bool g_bButtonDebounceJ=true;

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,         // SMCLK/64 ~ 750 kMHz
        375000,                                  // 500ms timer period
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
    g_pLCD_Task = &SCREEN;

    #if USE_ACCL
        Accelerometer PEN;
    #else
        Joystick PEN;
    #endif

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
void INIT_GPIO(void) {

    //LED Setup, assuming PORT2
    // - P2.0 is connected to the RGB LED
    P2->DIR |= BIT1; //Green LED
    P2->OUT &= BIT1; // Initialize the LED Value

    /* Confinguring P5.1 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT5);

    /* Confinguring P3.5 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN5);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT3);

    /* Confinguring P4.1 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
}

void EN_GPIO_INT(void) {
    MAP_Interrupt_enableInterrupt(INT_PORT5);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    MAP_Interrupt_enableInterrupt(INT_PORT3);
}

void DIS_GPIO_INT(void) {
    MAP_Interrupt_disableInterrupt(INT_PORT5);
    MAP_Interrupt_disableInterrupt(INT_PORT4);
    MAP_Interrupt_disableInterrupt(INT_PORT3);
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
	INIT_GPIO();

	/* Configuring TimerA1 and TimerA2 for Up Mode  using Driverlib*/
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    MAP_Interrupt_enableInterrupt(INT_TA1_0);


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

	 // Enable interrupts
	__enable_irq();

	return;
}

extern "C"
{
    // - Handle the Timer32 Interrupt
	void T32_INT1_IRQHandler(void)
	{
		TIMER32_1->INTCLR = 0U;
		//P1->OUT ^= BIT0; // - Toggle the heart beat indicator (1ms)
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
	 * Port 5 interrupt handler. This handler is called whenever switches attached
	 * to P5.1 (S1)
	 */
	void PORT5_IRQHandler(void)
	{
	    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

	    /* Handles S1 button press */
	    if (status & GPIO_PIN1)
	    {
	        if (g_bButtonDebounceS1)
	        {
	            g_bButtonDebounceS1 = false;

	            DIS_GPIO_INT();

	            P2->OUT ^= BIT1;
	            g_pLCD_Task->SetCurrentCMD(CMD_CHANGE_BACKGROUND);

	            /* Start button debounce timer */
	            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
	        }
	    }
	}

	/*
     * Port 4 interrupt handler. This handler is called whenever switches attached
     * to P4.3 (Joystick)
     */
    void PORT4_IRQHandler(void)
    {
        uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);

        /* Handles S1 button press */
        if (status & GPIO_PIN1)
        {
            if (g_bButtonDebounceJ)
            {
                g_bButtonDebounceJ = false;

                DIS_GPIO_INT();

                P2->OUT ^= BIT1;
                g_pLCD_Task->SetCurrentCMD(CMD_CHANGE_PEN_COLOR);

                /* Start button debounce timer */
                MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            }
        }
    }

	/*
     * Port 3 interrupt handler. This handler is called whenever switches attached
     * to P3.5 (S1)
     */
    void PORT3_IRQHandler(void)
    {
        uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

        /* Handles S1 button press */
        if (status & GPIO_PIN5)
        {
            if (g_bButtonDebounceS2)
            {
                g_bButtonDebounceS2 = false;

                DIS_GPIO_INT();

                P2->OUT ^= BIT1;
                g_pLCD_Task->SetCurrentCMD(CMD_CHANGE_PEN_SIZE);

                /* Start button debounce timer */
                MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            }
        }
    }


	/*
	 * Timer A1 interrupt handler. This handler determines whether to reset button
	 * debounce after debounce timer expires.
	 */
	void TA1_0_IRQHandler(void)
	{

	    if (P5IN & GPIO_PIN1)
	    {
	        g_bButtonDebounceS1 = true;
	    }
	    if (P3IN & GPIO_PIN5)
	    {
	        g_bButtonDebounceS2 = true;
	    }
	    if (P4IN & GPIO_PIN1)
	    {
	        g_bButtonDebounceJ = true;
	    }

	    if ((P5IN & GPIO_PIN1) && (P3IN & GPIO_PIN5) && (P4IN & GPIO_PIN1))
	    {
	        MAP_Timer_A_stopTimer(TIMER_A1_BASE);
	        EN_GPIO_INT();

	    }
	    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
	                TIMER_A_CAPTURECOMPARE_REGISTER_0);
	}
}
