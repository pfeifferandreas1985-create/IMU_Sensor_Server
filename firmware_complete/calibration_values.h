#ifndef CALIBRATION_VALUES_H
#define CALIBRATION_VALUES_H

// Kalibriert am: 2026-02-19 (aus host_app/calibration.json)
// GYRO BIAS: Statischer Drift des Gyroskops im Ruhezustand
const float GYRO_BIAS[] = { 0.038530452614483006f, 0.0045577907049396986f, -0.024831108072830354f };

// MAG HARD IRON: Konstante Magnetfeldverschiebung (z.B. durch Leiterbahnen)
const float MAG_OFFSET[] = { -0.2875000014901161f, 0.0625000149011612f, 0.04649999737739563f };

// MAG SOFT IRON: Achsenverzerrung (noch nicht kalibriert - Kalibrierung ausstehend)
const float MAG_SCALE[]  = { 1.0f, 1.0f, 1.0f };

#endif
