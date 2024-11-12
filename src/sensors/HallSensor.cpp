#include "HallSensor.h"
#include "stm32g4xx_hal.h"  // Import STM32 HAL
#include "main.h"

// Konstruktor: Initialisiert die Hardware-Pins und Parameter für STM32

HallSensor::HallSensor(GPIO_TypeDef* hallA_Port, uint16_t hallA_Pin,
                       GPIO_TypeDef* hallB_Port, uint16_t hallB_Pin,
                       GPIO_TypeDef* hallC_Port, uint16_t hallC_Pin,
                       int pp)
    : pinA_Port(hallA_Port), pinA_Pin(hallA_Pin),
      pinB_Port(hallB_Port), pinB_Pin(hallB_Pin),
      pinC_Port(hallC_Port), pinC_Pin(hallC_Pin), pole_pairs(pp) {

    cpr = pole_pairs * 6;  // Umdrehungen pro elektrischer Zyklus
    pullup = Pullup::USE_EXTERN;  // Externer Pullup als Standard
}

// Interrupt Callback-Funktionen für Kanal A, B und C
void HallSensor::handleA() {
    A_active = HAL_GPIO_ReadPin(pinA_Port, pinA_Pin);
    updateState();
}

void HallSensor::handleB() {
    B_active = HAL_GPIO_ReadPin(pinB_Port, pinB_Pin);
    updateState();
}

void HallSensor::handleC() {
    C_active = HAL_GPIO_ReadPin(pinC_Port, pinC_Pin);
    updateState();
}

// Aktualisiert den Zustand und den Sektor nach einem Interrupt
void HallSensor::updateState() {
    int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);

    // Vermeidung von Störungen: Keine Änderung
    if (new_hall_state == hall_state) return;

    long new_pulse_timestamp = HAL_GetTick();
    hall_state = new_hall_state;

    int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
    int8_t electric_sector_dif = new_electric_sector - electric_sector;

    if (electric_sector_dif > 3) {
        direction = Direction::CCW;
        electric_rotations += direction;
    } else if (electric_sector_dif < (-3)) {
        direction = Direction::CW;
        electric_rotations += direction;
    } else {
        direction = (new_electric_sector > electric_sector) ? Direction::CW : Direction::CCW;
    }

    electric_sector = new_electric_sector;

    if (direction == old_direction) {
        pulse_diff = new_pulse_timestamp - pulse_timestamp;
    } else {
        pulse_diff = 0;
    }

    pulse_timestamp = new_pulse_timestamp;
    total_interrupts++;
    old_direction = direction;

    if (onSectorChange != nullptr) onSectorChange(electric_sector);
}

// Funktion, um einen Callback für Sektorwechsel zu setzen
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
    onSectorChange = _onSectorChange;
}

// Update-Funktion für den Sensor
void HallSensor::update() {
    if (use_interrupt) {
        __disable_irq();
    } else {
        A_active = HAL_GPIO_ReadPin(pinA_Port, pinA_Pin);
        B_active = HAL_GPIO_ReadPin(pinB_Port, pinB_Pin);
        C_active = HAL_GPIO_ReadPin(pinC_Port, pinC_Pin);
        updateState();
    }

    angle_prev_ts = pulse_timestamp;
    long last_electric_rotations = electric_rotations;
    int8_t last_electric_sector = electric_sector;
    if (use_interrupt) __enable_irq();

    angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI;
    full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
}

// Berechnung des Drehwinkels
float HallSensor::getSensorAngle() {
    return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI;
}

// Berechnung der Drehgeschwindigkeit
float HallSensor::getVelocity() {
    __disable_irq();
    long last_pulse_timestamp = pulse_timestamp;
    long last_pulse_diff = pulse_diff;
    __enable_irq();

    if (last_pulse_diff == 0 || ((long)(HAL_GetTick() - last_pulse_timestamp) > last_pulse_diff * 2)) {
        return 0;
    } else {
        return direction * (_2PI / (float)cpr) / (last_pulse_diff / 1000000.0f);
    }
}

// Initialisierung der Hardware-Pins und Berechnungsvariablen
void HallSensor::init() {
    electric_rotations = 0;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = (pullup == Pullup::USE_INTERN) ? GPIO_PULLUP : GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = pinA_Pin;
    HAL_GPIO_Init(pinA_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pinB_Pin;
    HAL_GPIO_Init(pinB_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pinC_Pin;
    HAL_GPIO_Init(pinC_Port, &GPIO_InitStruct);

    A_active = HAL_GPIO_ReadPin(pinA_Port, pinA_Pin);
    B_active = HAL_GPIO_ReadPin(pinB_Port, pinB_Pin);
    C_active = HAL_GPIO_ReadPin(pinC_Port, pinC_Pin);
    updateState();

    pulse_timestamp = HAL_GetTick();
}

// Funktion zum Aktivieren von Hardware-Interrupts
void HallSensor::enableInterrupts(void (*doA)(), void (*doB)(), void (*doC)()) {
    if (doA != nullptr) HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Beispiel für EXTI15-10
    if (doB != nullptr) HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Beispiel für EXTI15-10
    if (doC != nullptr) HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Beispiel für EXTI15-10

    use_interrupt = true;
}

