#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H

#include "stm32g4xx_hal.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// Elektrische Sektoren-Sequenz für Hall-Sensoren
const int8_t ELECTRIC_SECTORS[8] = { -1, 0, 4, 5, 2, 1, 3, -1 };

class HallSensor : public Sensor {
public:
    /**
    Konstruktor für die HallSensor-Klasse
    @param hallA_Port Port für Hall-Sensor A
    @param hallA_Pin Pin für Hall-Sensor A
    @param hallB_Port Port für Hall-Sensor B
    @param hallB_Pin Pin für Hall-Sensor B
    @param hallC_Port Port für Hall-Sensor C
    @param hallC_Pin Pin für Hall-Sensor C
    @param pp Polpaare
    */
    HallSensor(GPIO_TypeDef* hallA_Port, uint16_t hallA_Pin,
               GPIO_TypeDef* hallB_Port, uint16_t hallB_Pin,
               GPIO_TypeDef* hallC_Port, uint16_t hallC_Pin, int pp);

    /** Initialisierung der Hall-Sensor Pins */
    void init();

    /**
     * Aktiviert Hardware-Interrupts für die Hall-Sensor-Kanäle mit den bereitgestellten Callback-Funktionen
     * @param doA Zeiger auf die Interrupt-Handler-Funktion für Kanal A
     * @param doB Zeiger auf die Interrupt-Handler-Funktion für Kanal B
     * @param doC Zeiger auf die Interrupt-Handler-Funktion für Kanal C
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doC)() = nullptr);

    // Callback-Funktionen für Hall-Sensor-Interrupts
    void handleA();  // Kanal A
    void handleB();  // Kanal B
    void handleC();  // Kanal C

    // Hardware-Pins für A, B und C Kanäle
    GPIO_TypeDef* pinA_Port;
    uint16_t pinA_Pin;
    GPIO_TypeDef* pinB_Port;
    uint16_t pinB_Pin;
    GPIO_TypeDef* pinC_Port;
    uint16_t pinC_Pin;
    int use_interrupt;  // true, wenn Interrupts aktiviert sind

    // Hall-Sensor-Konfiguration
    Pullup pullup;   // interne oder externe Pullups
    int cpr;         // CPR-Wert des Hall-Sensors

    // Abstrakte Funktionen der Sensor-Klasse
    void update() override;                // Interrupt-sichere Aktualisierung
    float getSensorAngle() override;       // Winkel in Rad
    float getVelocity() override;          // Winkelgeschwindigkeit in Rad/s

    // Drehrichtung: CW (+1) oder CCW (-1)
    Direction direction;
    Direction old_direction;

    void attachSectorCallback(void (*onSectorChange)(int a) = nullptr);

    // Aktueller 3-Bit-Zustand der Hall-Sensoren
    volatile int8_t hall_state;
    // Aktueller Sektor des Sensors (jeder Sektor 60° elektrisch)
    volatile int8_t electric_sector;
    // Anzahl der elektrischen Umdrehungen
    volatile long electric_rotations;
    // Anzahl der Interrupts (hilfreich zur Identifikation von Störungen)
    volatile long total_interrupts;

    // Maximalgeschwindigkeit (rad/s) zur Ausreißer-Filterung
    float velocity_max = 1000.0f;

private:
    Direction decodeDirection(int oldState, int newState);
    void updateState();

    volatile unsigned long pulse_timestamp;  // Zeitstempel des letzten Impulses in µs
    volatile int A_active;  // Aktiver Zustand des A-Kanals
    volatile int B_active;  // Aktiver Zustand des B-Kanals
    volatile int C_active;  // Aktiver Zustand des C-Kanals

    // Funktionszeiger für Callback bei Sektoränderung
    void (*onSectorChange)(int sector) = nullptr;

    volatile long pulse_diff;
};

#endif

