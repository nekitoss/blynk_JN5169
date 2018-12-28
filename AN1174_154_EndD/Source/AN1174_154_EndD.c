/****************************************************************************
 *
 * MODULE:             enddevice.c
 *
 * COMPONENT:          enddevice.c,v
 *
 * VERSION:
 *
 * REVISION:           1.3
 *
 * DATED:              2006/11/06 10:03:46
 *
 * STATUS:             Exp
 *
****************************************************************************
*
* This software is owned by NXP B.V. and/or its supplier and is protected
* under applicable copyright laws. All rights are reserved. We grant You,
* and any third parties, a license to use this software solely and
* exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
* You, and any third parties must reproduce the copyright and warranty notice
* and any other legend of ownership on each copy or partial copy of the
* software.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.

* Copyright NXP B.V. 2012. All rights reserved
*
***************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>

inline void SET_PIN_MODE_OUTPUT(int pin) { vAHI_DioSetDirection(0, 1UL << (pin)); }
inline void SET_PIN_LOW(int pin) { vAHI_DioSetOutput(0, 1UL << (pin)); }
inline void SET_PIN_HIGH(int pin) { vAHI_DioSetOutput(1UL << (pin), 0); }

#define LED_PIN 15
#define TIMER_COUNTER_VALUE 6750000UL // 0.5sec = (27/2)mHz/x; x = 0.5*13500000 = 6 750 000(ticks)

PRIVATE void vInitSystem(void);
PRIVATE void ticktimer_callback();

volatile bool led_status=FALSE;

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system and runs
 * main loop.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
    /* Disable watchdog if enabled by default */
    #ifdef WATCHDOG_ENABLED
    vAHI_WatchdogStop();
    #endif

    vInitSystem();

    while (1)
    {
//    	blynk();
    }
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Simply jumps to AppColdStart
 * as, in this instance, application will never warm start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}


/****************************************************************************
 *
 * NAME: vInitSystem
 *
 * DESCRIPTION:
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitSystem(void)
{
    (void)u32AHI_Init();

    SET_PIN_MODE_OUTPUT(LED_PIN);

    //Tick Timer
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE); //disable timer
	vAHI_TickTimerWrite(0); //start from zero
	vAHI_TickTimerInterval(TIMER_COUNTER_VALUE);	//count on which interrupt, MAX=0x0FFFFFFF
	vAHI_TickTimerIntEnable(TRUE); //enable interrupts
	vAHI_TickTimerRegisterCallback(ticktimer_callback); //set interrupt callback function
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART); //start timer and set mode E_AHI_TICK_TIMER_CONT (continue counting) E_AHI_TICK_TIMER_RESTART (restart from zero) E_AHI_TICK_TIMER_STOP (stop timer) E_AHI_TICK_TIMER_DISABLE (disable timer)
}

PRIVATE void ticktimer_callback(uint32 u32Device, uint32 u32ItemBitmap)
{
	//led_status = !led_status;
	led_status = (led_status ? FALSE : TRUE);
	if (led_status)
		SET_PIN_HIGH(LED_PIN);
	else
		SET_PIN_LOW(LED_PIN);

}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
