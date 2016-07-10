

/*
 * Encoder interrupt handler for overflow
 */
CH_IRQ_HANDLER(STM32_TIM4_HANDLER)
{
    CH_IRQ_PROLOGUE();

    /* IRQ handling code, preemptable if the architecture supports it.*/

    chSysLockFromIsr();
    /* Invocation of some I-Class system APIs, never preemptable.*/

    // OK, this is an update interrupt not only event caused 
    // by initializing the timer
    if(STM32_TIM4->CNT & (1 << 31))
    {
        if(STM32_TIM4->CR1 & (1 << 4))
        {
            // counting down -> underflow!
            --EncoderPosition_H;

            palClearPad(GPIOE, LED_OVERFLOW);
            palTogglePad(GPIOE, LED_UNDERFLOW);
      
        } else {
            // counting up -> overflow!
            ++EncoderPosition_H;

            palClearPad(GPIOE, LED_UNDERFLOW);
            palTogglePad(GPIOE, LED_OVERFLOW);
        }

        // reset the update interrupt flag
        STM32_TIM4->SR &= ~1;

        // FIXME: reset the TIM4 interrupt flag
    }
        
    chSysUnlockFromIsr();

    /* More IRQ handling code, again preemptable.*/

    CH_IRQ_EPILOGUE();
}


// turn the interrupt on after configuring it in the timer registers (i.e. that the timer should generate interrupt in the first place!)
nvicEnableVector(STM32_TIM4_NUMBER, CORTEX_PRIORITY_MASK(6));
