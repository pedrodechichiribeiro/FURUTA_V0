#pragma once

// ============================================================
// Vetor de estado do Pendulo de Furuta
//
// x = [phi, phiDot, beta, betaDot]^T
//
// Unidades:
//   phi      [rad]
//   phiDot   [rad/s]
//   beta     [rad]
//   betaDot  [rad/s]
// ============================================================

struct StateVector
{
    float phi;
    float phiDot;
    float beta;
    float betaDot;
};
