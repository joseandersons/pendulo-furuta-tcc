#pragma once

// Modelo identificado por mínimos quadrados.
// Estado: [theta_error_rad, alpha_rad, theta_dot_rad_s, alpha_dot_rad_s]
// Entrada: vel_rad_s
// Ts identificado: 0.002000000 s

static const float AD_ID[4][4] = {
    { 0.994971436f, 0.67938062f, -0.0119430358f, -0.0123646761f },
    { 0.00014146821f, 0.951258642f, 0.00109336589f, 5.16299585e-07f },
    { -0.0043113746f, 2.08668484f, 0.459885478f, -0.0357371467f },
    { 0.00979572761f, -3.39612192f, 0.0734681758f, 0.848329706f },
};

static const float BD_ID[4] = { 0.0351874979f, 0.00125554806f, 0.55057278f, 0.0775760197f };

static const float C_BIAS_ID[4] = { 0f, 0f, 0f, 0f };
