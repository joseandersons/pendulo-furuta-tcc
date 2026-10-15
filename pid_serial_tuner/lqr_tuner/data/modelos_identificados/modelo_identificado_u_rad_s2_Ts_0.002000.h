#pragma once

// Modelo identificado por mínimos quadrados.
// Estado: [theta_error_rad, alpha_rad, theta_dot_rad_s, alpha_dot_rad_s]
// Entrada: u_rad_s2
// Ts identificado: 0.002000000 s

static const float AD_ID[4][4] = {
    { 0.842290696f, 50.7519345f, -0.698730355f, 8.6075834f },
    { 0.000635649776f, 0.785239277f, 0.00360258374f, -0.0253286293f },
    { -6.19623425e-05f, -0.897662348f, 0.594307299f, 0.759854199f },
    { -0.000355882575f, -0.283829987f, 0.0435331643f, 1.56271391f },
};

static const float BD_ID[4] = { 0.225232029f, -0.000671460271f, 0.0169092194f, 0.018137144f };

static const float C_BIAS_ID[4] = { 0f, 0f, 0f, 0f };
