#ifndef GENERIC_CURRENT_SENSE_H
#define GENERIC_CURRENT_SENSE_H

#include "stm32g4xx_hal.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// Struktur zur Speicherung der Phasenströme
typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrent_s;

class GenericCurrentSense {
public:
    /**
     * Konstruktor der Klasse GenericCurrentSense
     * @param readCallback Zeiger auf die Funktion, die den Phasenstrom liest
     * @param initCallback Zeiger auf die Funktion, die die Initialisierung durchführt
     */
    GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)() = nullptr);

    /** Initialisierung des Sensors */
    int init();

    /** Kalibriert die Nullpunkte der ADC-Offsets */
    void calibrateOffsets();

    /** Liest die Ströme aller drei Phasen */
    PhaseCurrent_s getPhaseCurrents();

    /**
     * Richten Sie den Stromsensor mit dem Motortreiber aus
     * @param voltage Spannung zum Ausrichten (optional)
     * @param modulation_centered Zentrierte Modulation verwenden
     * @return Status-Flag: 1 = Erfolg, 0 = Fehler
     */
    int driverAlign(float voltage, bool modulation_centered = true);

private:
    // Funktionszeiger für den ADC-Lesevorgang und die Initialisierung
    PhaseCurrent_s (*readCallback)();
    void (*initCallback)();

    // Offset-Werte für jede Phase (Nullpunktkalibrierung)
    float offset_ia = 0;
    float offset_ib = 0;
    float offset_ic = 0;

    // Initialisierungsstatus
    bool initialized = false;
};

// Implementierung der Methoden

GenericCurrentSense::GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)()) {
    if (readCallback != nullptr) this->readCallback = readCallback;
    if (initCallback != nullptr) this->initCallback = initCallback;
}

// Sensor-Initialisierungsfunktion
int GenericCurrentSense::init() {
    if (initCallback != nullptr) initCallback(); // Konfiguration der ADC-Variablen
    calibrateOffsets();                          // Kalibrierung der Nullpunkte
    initialized = true;                          // Setze initialisiert-Flag
    return 1;                                    // Erfolg zurückgeben
}

// Kalibrierung der ADC-Offsets
void GenericCurrentSense::calibrateOffsets() {
    const int calibration_rounds = 1000;

    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;

    // Lesevorgang des ADCs für die Nullpunkt-Kalibrierung (1000 Runden)
    for (int i = 0; i < calibration_rounds; i++) {
        PhaseCurrent_s current = readCallback();
        offset_ia += current.a;
        offset_ib += current.b;
        offset_ic += current.c;
        HAL_Delay(1);  // STM32 HAL Delay
    }

    // Mittelwertberechnung der Offsets
    offset_ia /= calibration_rounds;
    offset_ib /= calibration_rounds;
    offset_ic /= calibration_rounds;
}

// Liest die Phasenströme und subtrahiert die Offset-Werte
PhaseCurrent_s GenericCurrentSense::getPhaseCurrents() {
    PhaseCurrent_s current = readCallback();
    current.a = (current.a - offset_ia);
    current.b = (current.b - offset_ib);
    current.c = (current.c - offset_ic);
    return current;
}

// Richten Sie den Stromsensor mit dem Motortreiber aus
int GenericCurrentSense::driverAlign(float voltage, bool modulation_centered) {
    _UNUSED(voltage);  // Entfernt Warnung für ungenutzten Parameter
    if (!initialized) return 0;
    return 1;
}

#endif

