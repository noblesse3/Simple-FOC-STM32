
#ifndef GENERIC_CS_LIB_H
#define GENERIC_CS_LIB_H

#include "stm32g4xx_hal.h"          // Replaces Arduino.h to use STM32 HAL functions
#include "../common/foc_utils.h"     // FOC-specific utility functions
#include "../common/time_utils.h"    // Timing utility functions
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"

// Definition of the GenericCurrentSense class
class GenericCurrentSense : public CurrentSense {
public:
    /**
      Constructor for the GenericCurrentSense class
    */
    GenericCurrentSense(PhaseCurrent_s (*readCallback)() = nullptr, void (*initCallback)() = nullptr);

    // Implementing functions for the CurrentSense interface
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    int driverAlign(float align_voltage, bool modulation_centered) override;

    PhaseCurrent_s (*readCallback)() = nullptr; //!< Function pointer for sensor reading
    void (*initCallback)() = nullptr; //!< Function pointer for sensor initialization

private:
    /**
     *  Function to calibrate zero offsets for the ADC
     */
    void calibrateOffsets();
    float offset_ia; //!< Zero-current A voltage value (ADC center reading)
    float offset_ib; //!< Zero-current B voltage value (ADC center reading)
    float offset_ic; //!< Zero-current C voltage value (ADC center reading)

};

#endif
