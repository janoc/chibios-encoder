/*
 * Pin connections
 *
 * PA5: CLK  (LCD)
 * PA7: MOSI (LCD)
 * PE3: /CS  (LCD)
 *
 * PD12: Encoder A
 * PD13: Encoder B
 *
 * PA8:  PWM out    (motor)
 * PA9:  Turn left  (motor)
 * PA10: Turn right (motor)
 * 
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "pwm.h"

#include <stm32_tim.h>
#include <TIMv1/pwm_lld.h>

#include "shell.h"
#include "chprintf.h"

#include "usb-setup.h"
#include "lcd-setup.h"

/*===========================================================================*/
/* Globals                                                                   */
/*===========================================================================*/
#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

#define SIGN(x) ((x) >= 0? 1 : -1)

#define TURN_LEFT_PIN  GPIOA_PIN9
#define TURN_RIGHT_PIN GPIOA_PIN10

//#define LED_PIN GPIOE_LED3_RED
#define LED_PIN GPIOE_LED6_GREEN

// encoder overflow LEDs
#define LED_OVERFLOW   GPIOE_LED7_GREEN
#define LED_UNDERFLOW  GPIOE_LED9_BLUE

static uint16_t PID_Target = 0;
static uint16_t PID_MAX_ERR = 2;

static float PID_KP = 0.0514;
static float PID_KI = 0.000005;
static float PID_KD = 0.008;
static int PID_OUT = 0;
static double PID_ITerm = 0;

//static uint32_t EncoderPosition = 0;
//static uint16_t EncoderPosition_H = 0;
//#define ENCODER_POS ((EncoderPosition_H << 16) | (0xFFFF & STM32_TIM4->CNT))

#define ENCODER_POS ((uint16_t)(0xFFFF & STM32_TIM4->CNT))


/*===========================================================================*/
/* Utilities                                                                 */
/*===========================================================================*/

void set_speed(int speed)
{
    // too slow, stop - saves the motor
    if(abs(speed) < 20)
        speed = 0;

    if(speed < 0)
    {
        speed = -speed;
        palClearPad(GPIOA, TURN_LEFT_PIN);
        palSetPad(GPIOA, TURN_RIGHT_PIN);        
    }

    else if(speed > 0)
    {
        palClearPad(GPIOA, TURN_RIGHT_PIN);
        palSetPad(GPIOA, TURN_LEFT_PIN);
    }

    else
    {
        palClearPad(GPIOA, TURN_LEFT_PIN);
        palClearPad(GPIOA, TURN_RIGHT_PIN);
    }

    // clamping
    if(speed > 100)
        speed = 100;

    int width = PWM_FRACTION_TO_WIDTH(&PWMD1, 100, speed);
    pwmEnableChannel(&PWMD1, 0, width);    
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[])
{
    size_t n, size;

    (void)argv;
    if (argc > 0)
    {
        chprintf(chp, "Usage: mem\r\n");
        return;
    }
    n = chHeapStatus(NULL, &size);
    chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
    chprintf(chp, "heap fragments   : %u\r\n", n);
    chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
    static const char *states[] = {THD_STATE_NAMES};
    Thread *tp;

    (void)argv;
    if (argc > 0)
    {
        chprintf(chp, "Usage: threads\r\n");
        return;
    }

    chprintf(chp, "    addr    stack prio refs     state time\r\n");
    tp = chRegFirstThread();

    do
    {
        chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
                 (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                 (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                 states[tp->p_state], (uint32_t)tp->p_time);
        tp = chRegNextThread(tp);
    }
    while (tp != NULL);
}

/* static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) */
/* { */
/*     (void)argv; */
/*     if (argc > 0) */
/*     { */
/*         chprintf(chp, "Usage: write\r\n"); */
/*         return; */
/*     } */

/*     while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) */
/*     { */
/*         chprintf(chp, "Pos: %x:%x %u\r\n", EncoderPosition_H, (0xFFFF & STM32_TIM4->CNT), ENCODER_POS); */
/*         chThdSleepMilliseconds(20); */
/*     } */
/*     chprintf(chp, "\r\n\nstopped\r\n"); */
/* } */

static void cmd_pos(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1)
    {
        chprintf(chp, "Usage: pos <position>\r\n");
        return;
    }

    uint16_t arg = atoi(argv[0]);
    chprintf(chp, "Setpoint: %u\r\n", arg);
    PID_Target = arg;
}

static void cmd_pid(BaseSequentialStream *chp, int argc, char *argv[])
{
    char buf[256];

    (void)argv;
    if (argc != 3)
    {
        chprintf(chp, "Usage: pid [kp ki kd]\r\n"); 
        snprintf(buf, 255, "Current settings: P: %1.8g  I: %1.8g  D: %1.8g\r\n", PID_KP, PID_KI, PID_KD);
        chprintf(chp, buf); // needed like this, because chprintf doesn't support floats!
        return;
    }

    PID_KP = atof(argv[0]);
    PID_KI = atof(argv[1]);
    PID_KD = atof(argv[2]);

    snprintf(buf, 255, "PID settings: P: %1.8g  I: %1.8g  D: %1.8g\r\n", PID_KP, PID_KI, PID_KD);
    chprintf(chp, buf); // needed like this, because chprintf doesn't support floats!
    
};

static void cmd_err(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1)
    {
        chprintf(chp, "Usage: err [tolerance]\r\n");
        chprintf(chp, "Current error tolerance: %3d\r\n", PID_MAX_ERR);
        return;
    }

    int32_t arg = strtol(argv[0], NULL, 10);
    chprintf(chp, "Max error: %ld\r\n", arg);
    PID_MAX_ERR = arg;
}

static const ShellCommand commands[] =
{
    {"mem", cmd_mem},
    {"threads", cmd_threads},
    //{"write", cmd_write},
    {"pos", cmd_pos},
    {"pid", cmd_pid},
    {"err", cmd_err},
    {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
    (BaseSequentialStream *)&SDU1,
    commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg)
{

    (void)arg;
    chRegSetThreadName("blinker");

    palSetPadMode(GPIOE, LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);

    while (TRUE)
    {
        systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 1000;
        palClearPad(GPIOE, LED_PIN);
        chThdSleepMilliseconds(time);
        palSetPad(GPIOE, LED_PIN);
        chThdSleepMilliseconds(time);
    }

    // dead code, avoids warning
    msg_t foo;
    return foo;
}

static WORKING_AREA(waPIDThread1, 128);
static msg_t PIDThread(void *arg)
{
    chRegSetThreadName("pid");

    const int SPEED_MAX = 100;
    const int SPEED_MIN = 25;
    
    float lastInput = 0;
    float dInput    = 0;    

    unsigned long lastTime = chTimeNow();
    
    while(1)
    {
        unsigned long now = chTimeNow();
        //float elapsed = ST2MS(now - lastTime) / 1000.0;
        float elapsed = ((float)(now - lastTime)) / CH_FREQUENCY;
        lastTime = now;
        
        volatile int16_t error = PID_Target - ENCODER_POS;

        if(labs(error) > PID_MAX_ERR)
        {
            PID_ITerm += PID_KI * error * elapsed;

            // clamp PID_ITerm to prevent windup
            if(fabs(PID_ITerm) > SPEED_MAX)
                PID_ITerm = SIGN(PID_ITerm) * SPEED_MAX;
            else if (fabs(PID_ITerm) < SPEED_MIN)
                PID_ITerm = SIGN(PID_ITerm) * SPEED_MIN;
            
            dInput = (PID_Target - lastInput) / elapsed;
            
            PID_OUT = (int)(PID_KP*error + PID_ITerm - PID_KD*dInput);

            // clamp motor speed to possible limits
            if(abs(PID_OUT) > SPEED_MAX)
                PID_OUT = SIGN(PID_OUT) * SPEED_MAX;
            else if(abs(PID_OUT) < SPEED_MIN)
                PID_OUT = SIGN(PID_OUT) * SPEED_MIN;
        } else {
            PID_OUT = 0;
            error = 0;
            PID_ITerm = 0;
        }

        lastInput = PID_Target;
        set_speed(PID_OUT);

        chThdSleepMicroseconds(100);
    }
    
    // dead code, avoids warning
    msg_t foo;
    return foo;
}

static WORKING_AREA(waDisplayThread1, 1024);
static msg_t DisplayThread(void *arg)
{
    chRegSetThreadName("lcd");

    lcdInit(&SPID1);
    lcdWriteString(&SPID1, "PID Servo v0.1");
    chThdSleepMilliseconds(1000);

    while(1)
    {
        char line1[17] = "P:";
        char line2[17] = "Line2";

        snprintf(line1, 16, "P:%u E:%d", (unsigned short)ENCODER_POS, PID_Target - ENCODER_POS);
        //snprintf(line1, 16, "P:%lu", PID_Target);
        //snprintf(line2, 16, "%1.2f %1.2f %1.2f", PID_KP, PID_KI, PID_KD);
        snprintf(line2, 16, "Speed: %d", PID_OUT);
        
        lcdClearDisplay(&SPID1);
        lcdWriteString(&SPID1, line1);
        lcdSetAddr(&SPID1, 0x40); // second line
        lcdWriteString(&SPID1, line2);
        
        chThdSleepMilliseconds(100);
    }

    // dead code, avoids warning
    msg_t foo;
    return foo;
}

/*
 * Encoder init
 */

static void encoderInit(void)
{
    palSetPadMode(GPIOE, LED_UNDERFLOW, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOE, LED_OVERFLOW, PAL_MODE_OUTPUT_PUSHPULL);

    palClearPad(GPIOE, LED_UNDERFLOW);
    palClearPad(GPIOE, LED_OVERFLOW);

    palSetPadMode(GPIOD, GPIOD_PIN12, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP); // AF2 = TIM4_CH1
    palSetPadMode(GPIOD, GPIOD_PIN13, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP); // AF2 = TIM4_CH2

    rccResetTIM4();
    rccEnableTIM4(FALSE);    // Turn on clock & disable the low power mode (D'OH! super intuitive API!)

    stm32_tim_t *tim4 = STM32_TIM4;
    tim4->CR1 |= (1 << 11) | (1 << 2) | 1;  // Enable the counter, set update event source to over/underflow and remap the UIF interrupt bit    
    tim4->SMCR = 3;          // Encoder mode 3
    tim4->CCER = 0;          // rising edge polarity
    tim4->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
    tim4->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
    tim4->CNT = 0;           // Initialize counter
    tim4->EGR = 1;           // Generate an update event
    tim4->DIER |= 1;         // Update interrupt on

    //nvicEnableVector(STM32_TIM4_NUMBER, CORTEX_PRIORITY_MASK(6));
}

/*
 * PWM setup
 */

static PWMConfig pwmcfg =
{
    10000, /* 10Khz PWM clock frequency */
    100, /* PWM period 0.01s */
    NULL, /* No callback */
    /* Only channel 1 enabled */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
    },
    0
};

static void pwmSetup(void)
{
    /* Enables PWM output (of TIM1, channel 1) on PA8 */
    palSetPadMode(GPIOA, GPIOA_PIN8, PAL_MODE_ALTERNATE(6));
    pwmStart(&PWMD1, &pwmcfg);
    pwmEnableChannel(&PWMD1, 0, 50);

    /* Direction pins */
    palSetPadMode(GPIOA, TURN_LEFT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, TURN_RIGHT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(GPIOA, TURN_LEFT_PIN);
    palClearPad(GPIOA, TURN_RIGHT_PIN);
}

/*
 * Application entry point.
 */
int main(void)
{
    Thread *shelltp = NULL;

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /*
     * Shell manager initialization.
     */
    shellInit();

    /*
     * Init encoder
     */
    encoderInit();

    /*
     * Start PWM
     */
    pwmSetup();

    /*
     * Creates the blinker thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    /*
     * Creates the LCD thread
     */
    chThdCreateStatic(waDisplayThread1, sizeof(waDisplayThread1), NORMALPRIO, DisplayThread, NULL);

    /*
     * Creates the PID thread
     */
    chThdCreateStatic(waPIDThread1, sizeof(waPIDThread1), NORMALPRIO, PIDThread, NULL);
       
    /*
     * Normal main() thread activity, in this demo it does nothing except
     * sleeping in a loop and check the button state.
     */
    while (TRUE)
    {
        if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
        else if (chThdTerminated(shelltp))
        {
            chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
            shelltp = NULL;           /* Triggers spawning of a new shell.        */
        }
        chThdSleepMilliseconds(1000);
    }
}
