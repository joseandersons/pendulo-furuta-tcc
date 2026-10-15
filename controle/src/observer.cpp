#include "observer.h"
#include "ang.h"

#include <Arduino.h>
#include <math.h>

static LQRState g_xhat{};
static bool g_ready = false;

// h_k no relatório: variável auxiliar do observador reduzido
static float g_h[2] = {
  0.0f,
  0.0f
};

static float Lred[2][2] = {
  {13.8058f, 0.0f},
  {0.0f, 5.0110f}
};

static float Hobs[2][2] = {
  {0.9724f, 0.0f},
  {0.0f, 0.9900f}
};

static float Gobs[2][2] = {
  {-0.3812f, 0.0f},
  {0.0f, 0.0218f}
};

static float Fobs[2] = {
  0.001972f,
  0.000913f
};

static float pending_Lred[2][2] = {
  {13.8058f, 0.0f},
  {0.0f, 5.0110f}
};

static float pending_Hobs[2][2] = {
  {0.9724f, 0.0f},
  {0.0f, 0.9900f}
};

static float pending_Gobs[2][2] = {
  {-0.3812f, 0.0f},
  {0.0f, 0.0218f}
};

static float pending_Fobs[2] = {
  0.001972f,
  0.000913f
};

static bool pending_has_Lred = false;
static bool pending_has_Hobs = false;
static bool pending_has_Gobs = false;
static bool pending_has_Fobs = false;

static bool finite4(float a, float b, float c, float d) {
  return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d);
}

// Relação do relatório:
// x_u_hat = h + L_red * x_m
//
// Isolando h:
// h = x_u_hat - L_red * x_m
static void observer_rebuild_h() {
  g_h[0] = g_xhat.theta_dot - (
    Lred[0][0] * g_xhat.theta +
    Lred[0][1] * g_xhat.alpha
  );

  g_h[1] = g_xhat.alpha_dot - (
    Lred[1][0] * g_xhat.theta +
    Lred[1][1] * g_xhat.alpha
  );
}

bool observer_stage_Lred(
  float m00,
  float m01,
  float m10,
  float m11
) {
  if (!finite4(m00, m01, m10, m11)) {
    return false;
  }

  pending_Lred[0][0] = m00;
  pending_Lred[0][1] = m01;
  pending_Lred[1][0] = m10;
  pending_Lred[1][1] = m11;

  pending_has_Lred = true;
  return true;
}

bool observer_stage_Hobs(
  float m00,
  float m01,
  float m10,
  float m11
) {
  if (!finite4(m00, m01, m10, m11)) {
    return false;
  }

  pending_Hobs[0][0] = m00;
  pending_Hobs[0][1] = m01;
  pending_Hobs[1][0] = m10;
  pending_Hobs[1][1] = m11;

  pending_has_Hobs = true;
  return true;
}

bool observer_stage_Gobs(
  float m00,
  float m01,
  float m10,
  float m11
) {
  if (!finite4(m00, m01, m10, m11)) {
    return false;
  }

  pending_Gobs[0][0] = m00;
  pending_Gobs[0][1] = m01;
  pending_Gobs[1][0] = m10;
  pending_Gobs[1][1] = m11;

  pending_has_Gobs = true;
  return true;
}

bool observer_stage_Fobs(
  float f0,
  float f1
) {
  if (!isfinite(f0) || !isfinite(f1)) {
    return false;
  }

  pending_Fobs[0] = f0;
  pending_Fobs[1] = f1;

  pending_has_Fobs = true;
  return true;
}

bool observer_stage_Lr(
  float m00,
  float m01,
  float m10,
  float m11
) {
  return observer_stage_Lred(m00, m01, m10, m11);
}

bool observer_stage_F(
  float m00,
  float m01,
  float m10,
  float m11
) {
  return observer_stage_Hobs(m00, m01, m10, m11);
}

bool observer_stage_G(
  float m00,
  float m01,
  float m10,
  float m11
) {
  return observer_stage_Gobs(m00, m01, m10, m11);
}

bool observer_stage_H(
  float h0,
  float h1
) {
  return observer_stage_Fobs(h0, h1);
}

bool observer_apply_staged() {
  if (
    !pending_has_Lred ||
    !pending_has_Hobs ||
    !pending_has_Gobs ||
    !pending_has_Fobs
  ) {
    return false;
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      Lred[i][j] = pending_Lred[i][j];
      Hobs[i][j] = pending_Hobs[i][j];
      Gobs[i][j] = pending_Gobs[i][j];
    }

    Fobs[i] = pending_Fobs[i];
  }

  if (g_ready) {
    observer_rebuild_h();
  }

  pending_has_Lred = false;
  pending_has_Hobs = false;
  pending_has_Gobs = false;
  pending_has_Fobs = false;

  return true;
}

void observer_reset() {
  g_xhat.theta = 0.0f;
  g_xhat.alpha = 0.0f;
  g_xhat.theta_dot = 0.0f;
  g_xhat.alpha_dot = 0.0f;

  g_h[0] = 0.0f;
  g_h[1] = 0.0f;

  g_ready = false;
}

void observer_init(float theta, float alpha, float theta_dot, float alpha_dot) {
  alpha = wrapPi(alpha);

  g_xhat.theta = theta;
  g_xhat.alpha = alpha;
  g_xhat.theta_dot = theta_dot;
  g_xhat.alpha_dot = alpha_dot;

  observer_rebuild_h();

  g_ready = true;
}

bool observer_ready() {
  return g_ready;
}

LQRState observer_get_state() {
  return g_xhat;
}

LQRState observer_update(float theta_meas, float alpha_meas, float u_prev_rad_s2) {
  alpha_meas = wrapPi(alpha_meas);

  if (!g_ready) {
    observer_init(theta_meas, alpha_meas, 0.0f, 0.0f);
  }

  // x_m = [theta, alpha]^T
  float xm[2] = {
    theta_meas,
    alpha_meas
  };


  // x_u_hat = h + L_red * x_m
  float theta_dot_est =
    g_h[0] +
    Lred[0][0] * xm[0] +
    Lred[0][1] * xm[1];

  float alpha_dot_est =
    g_h[1] +
    Lred[1][0] * xm[0] +
    Lred[1][1] * xm[1];

  // h_{k+1}
  float hn[2];

  // h_{k+1} = H_obs * h_k + G_obs * x_m,k + F_obs * u_k
  for (int i = 0; i < 2; i++) {
    hn[i] = Fobs[i] * u_prev_rad_s2;

    for (int j = 0; j < 2; j++) {
      hn[i] += Hobs[i][j] * g_h[j];
      hn[i] += Gobs[i][j] * xm[j];
    }
  }

  if (
    !isfinite(theta_dot_est) ||
    !isfinite(alpha_dot_est) ||
    !isfinite(hn[0]) ||
    !isfinite(hn[1])
  ) {
    observer_reset();
    observer_init(theta_meas, alpha_meas, 0.0f, 0.0f);
    return g_xhat;
  }

  // Atualiza h_k para o próximo ciclo
  g_h[0] = hn[0];
  g_h[1] = hn[1];

  // x_ctrl = [theta, alpha, theta_dot_hat, alpha_dot_hat]^T
  g_xhat.theta = theta_meas;
  g_xhat.alpha = alpha_meas;
  g_xhat.theta_dot = theta_dot_est;
  g_xhat.alpha_dot = alpha_dot_est;

  return g_xhat;
}