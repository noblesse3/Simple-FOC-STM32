#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "stm32g4xx_hal.h"
#include <math.h>

#define _2PI 6.28318530718f

/**
 *  Quadraturmodus-Konfigurationsstruktur
 */
enum Quadrature : uint8_t {
    ON    = 0x00, //!<  Quadraturmodus aktivieren: CPR = 4xPPR
    OFF   = 0x01  //!<  Quadraturmodus deaktivieren: CPR = PPR
};

class Encoder {
 public:
    /**
    Konstruktor der Encoder-Klasse
    @param encA  Encoder A-Pin
    @param encB  Encoder B-Pin
    @param ppr   Impulse pro Umdrehung (CPR = PPR * 4, wenn Quadratur aktiv)
    @param index Index-Pin-Nummer (optional)
    */
    Encoder(int encA, int encB, float ppr, int index = 0);

    /** Initialisiere die Encoder-Pins */
    void init();
    /**
     *  Funktion zur Aktivierung der Hardware-Interrupts für die Encoder-Kanäle
     *  mit bereitgestellten Callback-Funktionen. Wenn kein Callback bereitgestellt wird,
     *  wird der Interrupt nicht aktiviert.
     * 
     * @param doA Zeiger auf die A-Kanal-Interrupt-Handler-Funktion
     * @param doB Zeiger auf die B-Kanal-Interrupt-Handler-Funktion
     * @param doIndex Zeiger auf die Index-Kanal-Interrupt-Handler-Funktion
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr, void (*doIndex)() = nullptr);

    // Encoder-Interrupt-Callback-Funktionen
    /** A-Kanal Callback-Funktion */
    void handleA();
    /** B-Kanal Callback-Funktion */
    void handleB();
    /** Index-Kanal Callback-Funktion */
    void handleIndex();

    // Pins A, B und Index
    int pinA; //!< Hardware-Pin A des Encoders
    int pinB; //!< Hardware-Pin B des Encoders
    int index_pin; //!< Index-Pin

    // Encoder-Konfiguration
    Quadrature quadrature; //!< Konfiguration zum Aktivieren/Deaktivieren des Quadraturmodus
    float cpr; //!< Encoder CPR-Wert

    // Berechnung des aktuellen Winkels (rad)
    float getSensorAngle();
    // Berechnung der aktuellen Winkelgeschwindigkeit (rad/s)
    float getVelocity();
    void update();

    /**
     * gibt 0 zurück, wenn keine Suche nach absolutem Nullpunkt erforderlich ist
     * 0 - Encoder ohne Index
     * 1 - Encoder mit Index
     */
    int needsSearch();

 private:
    int hasIndex(); //!< Funktion, die 1 zurückgibt, wenn der Encoder einen Index-Pin hat, sonst 0.

    // Encoder-Zustandsvariablen
    volatile long pulse_counter; //!< Aktueller Impulszähler
    volatile long pulse_timestamp; //!< Letzter Impulszeitstempel in us
    volatile int A_active; //!< Aktiver Zustand des A-Kanals
    volatile int B_active; //!< Aktiver Zustand des B-Kanals
    volatile int I_active; //!< Aktiver Zustand des Index-Kanals
    volatile bool index_found = false; //!< Flag, das anzeigt, ob der Index gefunden wurde

    // Variablen zur Geschwindigkeitsberechnung
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
};

// Konstruktorinitialisierung
Encoder::Encoder(int encA, int encB, float ppr, int index) 
    : pinA(encA), pinB(encB), index_pin(index), quadrature(Quadrature::ON), cpr(ppr * 4) {
    pulse_counter = 0;
    pulse_timestamp = HAL_GetTick();
}

// Encoder initialisieren
void Encoder::init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // A, B und Index-Pins als Eingänge konfigurieren
    GPIO_InitStruct.Pin = pinA | pinB | index_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Startwerte für Zustandsvariablen
    A_active = HAL_GPIO_ReadPin(GPIOA, pinA);
    B_active = HAL_GPIO_ReadPin(GPIOA, pinB);
    if (hasIndex()) I_active = HAL_GPIO_ReadPin(GPIOA, index_pin);
}

// Interrupt-Handler für Kanal A
void Encoder::handleA() {
    int A = HAL_GPIO_ReadPin(GPIOA, pinA);
    if (A != A_active) {
        pulse_counter += (A == B_active) ? 1 : -1;
        pulse_timestamp = HAL_GetTick();
        A_active = A;
    }
}

// Interrupt-Handler für Kanal B
void Encoder::handleB() {
    int B = HAL_GPIO_ReadPin(GPIOA, pinB);
    if (B != B_active) {
        pulse_counter += (A_active != B) ? 1 : -1;
        pulse_timestamp = HAL_GetTick();
        B_active = B;
    }
}

// Interrupt-Handler für den Index-Kanal
void Encoder::handleIndex() {
    if (hasIndex()) {
        int I = HAL_GPIO_ReadPin(GPIOA, index_pin);
        if (I && !I_active) {
            index_found = true;
            pulse_counter = round((float)pulse_counter / cpr) * cpr;
        }
        I_active = I;
    }
}

// Berechnet den Winkel des Encoders in Radiant
float Encoder::getSensorAngle() {
    return _2PI * (pulse_counter / cpr);
}

// Berechnet die Winkelgeschwindigkeit des Encoders
float Encoder::getVelocity() {
    long now = HAL_GetTick();
    float dt = (now - prev_timestamp_us) * 1e-3;
    if (dt <= 0 || dt > 0.5f) dt = 1e-3;

    long pulse_diff = pulse_counter - prev_pulse_counter;
    pulse_per_second = pulse_diff / dt;

    prev_timestamp_us = now;
    prev_pulse_counter = pulse_counter;
    return pulse_per_second / cpr * _2PI;
}

// Prüft, ob ein Index-Pin vorhanden ist
int Encoder::hasIndex() {
    return index_pin != 0;
}

// Aktiviert Interrupts für die Encoder-Pins
void Encoder::enableInterrupts(void (*doA)(), void (*doB)(), void (*doIndex)()) {
    if (doA) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
    if (doB) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
    if (hasIndex() && doIndex) attachInterrupt(digitalPinToInterrupt(index_pin), doIndex, CHANGE);
}

#endif

