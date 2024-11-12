#include "main.h" 
class BLDCDriver6PWM {
public:
    // Pin initialisation
    BLDCDriver6PWM(GPIO_TypeDef* phA_h_Port, uint16_t phA_h_Pin,
                   GPIO_TypeDef* phA_l_Port, uint16_t phA_l_Pin,
                   GPIO_TypeDef* phB_h_Port, uint16_t phB_h_Pin,
                   GPIO_TypeDef* phB_l_Port, uint16_t phB_l_Pin,
                   GPIO_TypeDef* phC_h_Port, uint16_t phC_h_Pin,
                   GPIO_TypeDef* phC_l_Port, uint16_t phC_l_Pin,
                   GPIO_TypeDef* en_Port, uint16_t en_Pin);
void enable();
    void disable();
    int init();
    void setPwm(float Ua, float Ub, float Uc);
    void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);

private:
    GPIO_TypeDef* pwmA_h_Port; uint16_t pwmA_h_Pin;
    GPIO_TypeDef* pwmA_l_Port; uint16_t pwmA_l_Pin;
    GPIO_TypeDef* pwmB_h_Port; uint16_t pwmB_h_Pin;
    GPIO_TypeDef* pwmB_l_Port; uint16_t pwmB_l_Pin;
    GPIO_TypeDef* pwmC_h_Port; uint16_t pwmC_h_Pin;
    GPIO_TypeDef* pwmC_l_Port; uint16_t pwmC_l_Pin;
    GPIO_TypeDef* enable_Port; uint16_t enable_Pin;

    float voltage_power_supply = DEF_POWER_SUPPLY;
    float voltage_limit = NOT_SET;
    float dead_zone = 0.02f;
    TIM_HandleTypeDef htim_pwm;  // Timer for PWM
    PhaseState phase_state[3] = {PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF};
};

// Initialisation of GPIO

int BLDCDriver6PWM::init() {
    // Configuration of GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // Configure each GPIO
    GPIO_InitStruct.Pin = pwmA_h_Pin;
    HAL_GPIO_Init(pwmA_h_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pwmA_l_Pin;
    HAL_GPIO_Init(pwmA_l_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pwmB_h_Pin;
    HAL_GPIO_Init(pwmB_h_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pwmB_l_Pin;
    HAL_GPIO_Init(pwmB_l_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pwmC_h_Pin;
    HAL_GPIO_Init(pwmC_h_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pwmC_l_Pin;
    HAL_GPIO_Init(pwmC_l_Port, &GPIO_InitStruct);

    // Configure a timer for the PWM
    
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_3);

    return 1;  
}

void BLDCDriver6PWM::enable() {
    HAL_GPIO_WritePin(enable_Port, enable_Pin, GPIO_PIN_SET);
    setPwm(0, 0, 0);  // Initialise the values of PWM t0 0
}
void BLDCDriver6PWM::disable() {
    setPwm(0, 0, 0);
    HAL_GPIO_WritePin(enable_Port, enable_Pin, GPIO_PIN_RESET);
}
//set voltage to the PWM PIN

void BLDCDriver6PWM::setPwm(float Ua, float Ub, float Uc) {
    Ua = _constrain(Ua, 0, voltage_limit);
    Ub = _constrain(Ub, 0, voltage_limit);
    Uc = _constrain(Uc, 0, voltage_limit);

    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(&htim_pwm, TIM_CHANNEL_1, dc_a * htim_pwm.Init.Period);
    __HAL_TIM_SET_COMPARE(&htim_pwm, TIM_CHANNEL_2, dc_b * htim_pwm.Init.Period);
    __HAL_TIM_SET_COMPARE(&htim_pwm, TIM_CHANNEL_3, dc_c * htim_pwm.Init.Period);
}
