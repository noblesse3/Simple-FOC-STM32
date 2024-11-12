#ifndef BLDCDriver6PWM_h
#define BLDCDriver6PWM_h

#include "stm32g4xx_hal.h"   // Import STM32 HAL-Bibliothek
#include "main.h"            // Hauptheader für die Konfiguration
#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 * 6-PWM BLDC Driver Class
 */
class BLDCDriver6PWM : public BLDCDriver {
public:

BLDCDriver6PWM(GPIO_TypeDef* phA_h_Port, uint16_t phA_h_Pin,
                   GPIO_TypeDef* phA_l_Port, uint16_t phA_l_Pin,
                   GPIO_TypeDef* phB_h_Port, uint16_t phB_h_Pin,
                   GPIO_TypeDef* phB_l_Port, uint16_t phB_l_Pin,
                   GPIO_TypeDef* phC_h_Port, uint16_t phC_h_Pin,
                   GPIO_TypeDef* phC_l_Port, uint16_t phC_l_Pin,
                   GPIO_TypeDef* en_Port = nullptr, uint16_t en_Pin = NOT_SET);

// Motor hardware init function
    int init() override;
    // Motor disable function
    void disable() override;
    // Motor enale function
    void enable() override;

// Hardware-Variables 
    GPIO_TypeDef* pwmA_h_Port; uint16_t pwmA_h_Pin;
    GPIO_TypeDef* pwmA_l_Port; uint16_t pwmA_l_Pin;
    GPIO_TypeDef* pwmB_h_Port; uint16_t pwmB_h_Pin;
    GPIO_TypeDef* pwmB_l_Port; uint16_t pwmB_l_Pin;
    GPIO_TypeDef* pwmC_h_Port; uint16_t pwmC_h_Pin;
    GPIO_TypeDef* pwmC_l_Port; uint16_t pwmC_l_Pin;
    GPIO_TypeDef* enable_Port; uint16_t enable_Pin;

    float dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low ) for each pwm cycle [0,1]
    PhaseState phase_state[3]; //!< Phase state (active/disabled)

    /**
     * Phasenspannungen auf die Hardware setzen
     * @param Ua - Phase A Spannung
     * @param Ub - Phase B Spannung
     * @param Uc - Phase C Spannung
     */
    void setPwm(float Ua, float Ub, float Uc) override;

    /**
     * Phasenstatus setzen
     * @param sa - Phase A aktiv / deaktiviert
     * @param sb - Phase B aktiv / deaktiviert
     * @param sc - Phase C aktiv / deaktiviert
     */
    void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

private:
    // Initialisierungsmethode für 6-PWM-Treiber
    TIM_HandleTypeDef htim_pwm;  // Timer für PWM
};

#endif
