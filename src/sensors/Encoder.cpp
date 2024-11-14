#include "stm32g4xx_hal.h"
#include "Encoder.h"

// Definition der Konstanten und GPIO Pins für den Encoder
#define _2PI 6.28318530718f
#define ENCODER_PIN_A GPIO_PIN_0   // A Pin für Encoder
#define ENCODER_PIN_B GPIO_PIN_1   // B Pin für Encoder
#define ENCODER_PIN_INDEX GPIO_PIN_2 // Index Pin für Encoder
#define ENCODER_GPIO_PORT GPIOA

// Encoder-Datenstruktur zur Speicherung des Zustands
typedef struct {
    int pulse_counter;
    float cpr;               // Counts per rotation (Impulse pro Umdrehung)
    int A_active;
    int B_active;
    int I_active;
    int quadrature;          // Quadraturmodus (z. B. ON/OFF)
    long pulse_timestamp;
    float angle_prev;
    float pulse_per_second;
    float prev_timestamp_us;
} Encoder_t;

Encoder_t encoder;

// Handler für den A-Kanal
void Encoder_HandleA() {
    int A = HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, ENCODER_PIN_A);
    if (A != encoder.A_active) {
        encoder.pulse_counter += (encoder.A_active == encoder.B_active) ? 1 : -1;
        encoder.pulse_timestamp = HAL_GetTick();
        encoder.A_active = A;
    }
}

// Handler für den B-Kanal
void Encoder_HandleB() {
    int B = HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, ENCODER_PIN_B);
    if (B != encoder.B_active) {
        encoder.pulse_counter += (encoder.A_active != encoder.B_active) ? 1 : -1;
        encoder.pulse_timestamp = HAL_GetTick();
        encoder.B_active = B;
    }
}

// Handler für den Index-Kanal
void Encoder_HandleIndex() {
    if (HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, ENCODER_PIN_INDEX) == GPIO_PIN_SET) {
        encoder.pulse_counter = (int)(encoder.pulse_counter / encoder.cpr) * encoder.cpr;
    }
}

// Funktion zur Berechnung des Winkels
float Encoder_GetAngle() {
    return _2PI * (encoder.pulse_counter / encoder.cpr);
}

// Funktion zur Berechnung der Geschwindigkeit
float Encoder_GetVelocity() {
    // Kopiere Variablen in kurzer Sperrsektion
    long timestamp_us = HAL_GetTick();
    float Ts = (timestamp_us - encoder.prev_timestamp_us) * 1e-3; // Delta-Zeit in Sekunden

    // Kopieren der aktuellen Pulse und Zeitstempel
    int copy_pulse_counter = encoder.pulse_counter;
    long copy_pulse_timestamp = encoder.pulse_timestamp;

    float Th = (timestamp_us - copy_pulse_timestamp) * 1e-3;
    int dN = copy_pulse_counter - encoder.pulse_counter;

    float dt = Ts + encoder.prev_timestamp_us - Th;
    encoder.pulse_per_second = (dN != 0 && dt > Ts / 2) ? dN / dt : encoder.pulse_per_second;

    // Geschwindigkeit berechnen in Rad/s
    float velocity = encoder.pulse_per_second / encoder.cpr * _2PI;

    // Variablen für den nächsten Durchlauf speichern
    encoder.prev_timestamp_us = timestamp_us;
    encoder.prev_timestamp_us = Th;
    return velocity;
}

// Initialisierung der EXTI (External Interrupts) für den Encoder
void Encoder_EnableInterrupts(void (*doA)(), void (*doB)(), void (*doIndex)()) {
    // Konfiguration der EXTI-Interrupts für die Pins A, B und Index
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    if (doIndex) {
        HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    }
}

// Callback-Funktion für EXTI-Interrupts, um die jeweiligen Handler aufzurufen
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENCODER_PIN_A) {
        Encoder_HandleA();
    } else if (GPIO_Pin == ENCODER_PIN_B) {
        Encoder_HandleB();
    } else if (GPIO_Pin == ENCODER_PIN_INDEX) {
        Encoder_HandleIndex();
    }
}
// Initialisierung des Encoders
void Encoder_Init(float ppr) {
    // Setzen der Variablen für Impulszähler und Zeitstempel
    encoder.pulse_counter = 0;
    encoder.cpr = ppr * 4;   // Quadrierung der PPR (Counts per rotation)
    encoder.prev_timestamp_us = HAL_GetTick(); // Startzeitstempel in Mikrosekunden
    
    // GPIO Konfiguration für Encoder-Pins (A, B, Index)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ENCODER_PIN_A | ENCODER_PIN_B | ENCODER_PIN_INDEX;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // Kein Pull-up, da externe Pull-ups bevorzugt werden
    HAL_GPIO_Init(ENCODER_GPIO_PORT, &GPIO_InitStruct);
}

