#include "main.h"

#define IN_START (0x55)
#define IN_STOP (0xAA)

#define OUT_START (0x55)
#define OUT_STOP (0xAA)

#define IN_MIN (950)
#define IN_MAX (4095)

#define MAX_PWM_HIGH (MAX_PWM * 1)

#define TICKS_TO_RELEASE (100)
#define TICKS_TO_POWEROFF (500)

#define LOW_VOLTAGE (25)
#define EXP_FILTER_RANGE (64)
#define POWER_BEFORE_DELIMER (3)

#define HALL_CHANGES_PER_CIRCLE (42)

#define MAX_TICKS_PER_CHANGE (1000)

// inversed polarity for all channels
#define CCER_DEF (TIM_CCER_CC1P_Msk | TIM_CCER_CC1NP_Msk | \
                  TIM_CCER_CC2P_Msk | TIM_CCER_CC2NP_Msk | \
                  TIM_CCER_CC3P_Msk | TIM_CCER_CC3NP_Msk)

#define LED_ON (STOP_LED_GPIO_Port->ODR |= STOP_LED_Pin)
#define LED_OFF (STOP_LED_GPIO_Port->ODR &= ~STOP_LED_Pin)
#define LED_TOGGLE (STOP_LED_GPIO_Port->ODR ^= STOP_LED_Pin)

#define MAX_CURRENT (30000)

#define set_chs(val_ch1, val_ch2, val_ch3) \
    {                                      \
        TIM1->CCR1 = val_ch1;              \
        TIM1->CCR2 = val_ch2;              \
        TIM1->CCR3 = val_ch3;              \
    }

#define on_off_chs(channels) (TIM1->CCER = (CCER_DEF | channels))

#define PHASE (HAL_Y_GPIO_Port->IDR & (HAL_Y_Pin | HAL_G_Pin | HAL_B_Pin))

typedef enum
{
    PHASE1 = (HAL_Y_Pin),
    PHASE2 = (HAL_Y_Pin | HAL_G_Pin),
    PHASE3 = (HAL_G_Pin),
    PHASE4 = (HAL_G_Pin | HAL_B_Pin),
    PHASE5 = (HAL_B_Pin),
    PHASE6 = (HAL_Y_Pin | HAL_B_Pin),

    PHASE0 = 0,
    PHASE111 = (HAL_Y_Pin | HAL_G_Pin | HAL_B_Pin),
} HAL_phase_t;

typedef enum
{
    CH_NONE = 0,
    CH1 = LL_TIM_CHANNEL_CH1,
    CH1N = LL_TIM_CHANNEL_CH1N,
    CH2 = LL_TIM_CHANNEL_CH2,
    CH2N = LL_TIM_CHANNEL_CH2N,
    CH3 = LL_TIM_CHANNEL_CH3,
    CH3N = LL_TIM_CHANNEL_CH3N,
} ch_t;

typedef struct __attribute__((__packed__))
{
    uint8_t start;

    uint8_t mode;
    uint8_t LED;

    uint16_t current;
    uint16_t set_current;

    uint8_t freq;
    uint8_t batt_voltage;

    uint8_t err;

    uint8_t stop;
} out_data_t;

typedef struct __attribute__((__packed__))
{
    uint8_t start;
    uint16_t accel;
    uint16_t brake;
    uint8_t stop;
} in_data_t;

#define power_on() (P_EN_GPIO_Port->ODR |= P_EN_Pin)
#define power_off() (P_EN_GPIO_Port->ODR &= ~P_EN_Pin)

uint32_t millis = 0;

struct
{
    uint32_t release_timeout;
    uint32_t power_off_timeout;

    int break_power;

    int set_power;

    int power_full;
    int power_mult;

    int voltage;
    int current;

    int HAL_ticks_per_sec;
    int freq;

    int ticks_per_change;
    int current_ticks;
    int mode;

    out_data_t out_data;
    int out_data_len;

    in_data_t in_data;
    int in_data_len;
} motor;

uint16_t adc_data[2];

void calculate_power()
{
    motor.power_mult += (motor.set_power * EXP_FILTER_RANGE - motor.power_mult) / EXP_FILTER_RANGE;
    motor.power_full = motor.power_mult / EXP_FILTER_RANGE;

    int curr_d = motor.current - (MAX_CURRENT / (motor.mode + 1));
    if (curr_d > 0)
    {
        motor.power_full -= curr_d;
        if (motor.power_full < 0)
            motor.power_full = 0;
    }
}

void system_power_off()
{
    // so, we need to power off

    // off all irq
    __disable_irq();

    // off motor
    set_chs(0, 0, 0);
    on_off_chs(0);

    // release power switch
    power_off();

    while (1) // wait for power off
        ;
}

void btn_task()
{
    static bool power_on_press = true;
    static bool last_state = 0;
    static uint32_t state_ms = 0;
    static int press_count = 0;
    static uint32_t press_ms = 0;
    static uint8_t released = 0;

    if ((BTN_GPIO_Port->IDR & BTN_Pin) == 0) // btn is press
    {
        if (last_state == 0) // in press moment event
        {

            state_ms = 0;
            last_state = 1;
        }
        state_ms++;
        press_ms++;

        if (released && press_ms > 2000)
            system_power_off();
    }
    else // btn release
    {
        if (last_state == 1) // in release moment event
        {
            press_count++;
            state_ms = 0;
            last_state = 0;
            released = 1;
        }

        if (state_ms < 500)
            state_ms++;
        else // no more presses
        {
            if (power_on_press)
            {
                power_on_press = false;
            }
            else
            {
                switch (press_count)
                {
                case 0: // nothing to do
                    break;

                case 1:
                    if (press_ms < 2000) // short press
                    {
                        motor.out_data.LED ^= 1;
                    }
                    break;
                case 2:
                    motor.mode++;
                    motor.mode %= 3;
                    break;

                default:
                    break;
                }
            }
            press_count = 0;
            press_ms = 0;
        }
    }
}

void tick_stepper()
{
    int power_before = motor.current_ticks * motor.power_full / motor.ticks_per_change;
    if (power_before > motor.power_full)
        power_before = motor.power_full;

    switch (PHASE)
    {
    case PHASE1:
        set_chs(0, MAX_PWM, motor.power_full);
        on_off_chs(CH3 | CH2N);
        break;

    case PHASE2:
        set_chs(MAX_PWM, power_before, motor.power_full);
        on_off_chs(CH2 | CH3 | CH1N);
        break;

    case PHASE3:
        set_chs(MAX_PWM, motor.power_full, 0);
        on_off_chs(CH2 | CH1N);
        break;

    case PHASE4:
        set_chs(power_before, motor.power_full, MAX_PWM);
        on_off_chs(CH1 | CH2 | CH3N);
        break;

    case PHASE5:
        set_chs(motor.power_full, 0, MAX_PWM);
        on_off_chs(CH1 | CH3N);
        break;

    case PHASE6:
        set_chs(motor.power_full, MAX_PWM, power_before);
        on_off_chs(CH1 | CH3 | CH2N);
        break;

    default: // wrong hals
        set_chs(0, 0, 0);
        on_off_chs(0);
        break;
    }
}

void tick_iqr()
{
    motor.current_ticks++;

    if (motor.current_ticks > MAX_TICKS_PER_CHANGE)
    {
        motor.current_ticks = MAX_TICKS_PER_CHANGE;
        motor.ticks_per_change = MAX_TICKS_PER_CHANGE;
    }

    if (motor.break_power == 0)
    {
        if (motor.power_full > 0)
            tick_stepper();
        else
            on_off_chs(0); // off all
    }
}

void main_systick()
{
    millis++;
    btn_task();
    calculate_power();

    if (millis % 1000 == 0)
    {
        motor.freq = motor.HAL_ticks_per_sec * 10 / HALL_CHANGES_PER_CIRCLE;
        motor.HAL_ticks_per_sec = 0;
    }
    if (millis % 100 == 0)
    {
        if (motor.break_power > 0)
            LED_TOGGLE;
        else
            LED_OFF;
    }

    if (motor.release_timeout > 0)
        motor.release_timeout--;
    else
    {
        motor.set_power = 0;
        motor.power_full = 0;
        on_off_chs(0);
    }

    if (motor.power_off_timeout > 0)
        motor.power_off_timeout--;
    else
    {
        // system_power_off();
    }
}

void do_break(int val)
{
    val *= 50;
    if (val > MAX_PWM)
        val = MAX_PWM;

    set_chs(val, val, val);
    on_off_chs(CH1N | CH2N | CH3N);
}

void readed(uint8_t byte)
{
    uint8_t *data = (void *)&motor.in_data;

    if (byte == IN_START)
        motor.in_data_len = 0;

    data[motor.in_data_len++] = byte;

    if (motor.in_data_len == sizeof(in_data_t))
    {
        if (byte == IN_STOP)
        {
            motor.out_data_len = 0;

            if ((motor.in_data.accel < IN_MIN / 2) || (motor.in_data.brake < IN_MIN / 2))
                motor.out_data.err |= (1 << 1);
            else
                motor.out_data.err &= (1 << 1);

            power_on();
            motor.power_off_timeout = TICKS_TO_POWEROFF;
            motor.release_timeout = TICKS_TO_RELEASE;

            if (motor.in_data.brake > IN_MIN)
            {
                motor.set_power = 0;
                motor.power_full = 0;

                motor.break_power = MAX_PWM * (motor.in_data.brake - IN_MIN) / (IN_MAX - IN_MIN);
                if (motor.break_power > MAX_PWM)
                    motor.break_power = MAX_PWM;
                do_break(motor.break_power);
            }
            else
            {
                motor.break_power = 0;
                if (motor.in_data.accel > IN_MIN)
                {
                    motor.power_full = 0;
                    motor.set_power = (MAX_PWM) * (motor.in_data.accel - IN_MIN) / (IN_MAX - IN_MIN);

                    if (motor.set_power > MAX_PWM)
                        motor.set_power = MAX_PWM;
                }
                else
                {
                    motor.set_power = 0;
                    motor.power_full = 0;
                }
            }
            // parse
        }
        motor.in_data_len = 0;
    }
}

void hal_change()
{
    motor.HAL_ticks_per_sec++;

    motor.ticks_per_change = motor.current_ticks;
    motor.current_ticks = 0;
}

void tx_task()
{
    if (motor.out_data_len == 0)
    {
        motor.out_data.start = OUT_START;
        motor.out_data.stop = OUT_STOP;

        motor.out_data.batt_voltage = motor.voltage;
        motor.out_data.current = motor.current;
        motor.out_data.freq = motor.freq / 10;
        motor.out_data.mode = motor.mode;
        motor.out_data.set_current = motor.set_power;

        motor.out_data.err &= ~(1 << 1);

        if (PHASE == PHASE0 || PHASE == PHASE111)
            motor.out_data.err |= (1 << 1);
    }

    if (motor.out_data_len < sizeof(out_data_t) && (USART1->ISR & USART_ISR_TXE) > 0)
    {
        USART1->TDR = *(((uint8_t *)&motor.out_data) + (motor.out_data_len++));
    }
}

void my_main()
{

    set_chs(0, 0, 0);
    on_off_chs(0);

    motor.mode = 1;
    motor.release_timeout = TICKS_TO_RELEASE;
    motor.power_off_timeout = TICKS_TO_POWEROFF;
    motor.out_data_len = sizeof(out_data_t);

    while (1)
    {
        motor.voltage = adc_data[0] / 63;

        if (motor.voltage < 30)
            motor.out_data.err |= 1;
        else
            motor.out_data.err &= ~(1);

        if (adc_data[1] < 1010)
            motor.current = 0;
        else
        {
            motor.current = (adc_data[1] - 1010) * 40;
        }

        tx_task();
    }
}
