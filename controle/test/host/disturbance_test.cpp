#include "disturbance.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static int64_t now = 1000000;
static std::vector<std::string> drain() {
  std::vector<std::string> result;
  char text[220];
  while (disturbance_pop_message(text, sizeof(text))) result.emplace_back(text);
  return result;
}
static bool contains(const std::vector<std::string>& lines, const char* text) {
  for (auto& s : lines) if (s.find(text) != std::string::npos) return true;
  return false;
}
static DistSnapshot tick(bool allowed = true, float alpha = 0, float alpha_dot = 0) {
  now += 2000;
  return disturbance_update(now, allowed, alpha, disturbance_generation(), alpha_dot);
}
static void command(const char* s) { assert(disturbance_command(s, now)); }
int main() {
  assert(disturbance_snapshot(now).value == 0);
  command("DIST PULSE 3 100");
  assert(contains(drain(), "DIST ERR MODE"));
  tick();
  const char* invalid[] = {"DIST PULSE nan 100", "DIST PULSE inf 100",
    "DIST PULSE -inf 100", "DIST PULSE 1e999 100", "DIST PULSE 3x 100",
    "DIST PULSE 3 100x", "DIST PULSE 3 100.0", "DIST PULSE 3 -1",
    "DIST PULSE 3 99999999999999999999999", "DIST PULSE 1000.1 100",
    "DIST PULSE -1000.1 100", "DIST PULSE 3 19", "DIST PULSE 3 3001",
    "DIST PULSE 3 100 extra", "DIST PULSE 3", "DIST CANCEL extra", "DIST? extra"};
  for (auto s : invalid) {
    command(s);
    assert(contains(drain(), "DIST ERR"));
    assert(disturbance_snapshot(now).value == 0);
  }
  command("DIST PULSE -3 100");
  assert(contains(drain(), "DIST OK PULSE"));
  assert(disturbance_snapshot(now).state == DistState::PENDING);
  assert(disturbance_snapshot(now).value == 0);
  command("DIST PULSE 3 100");
  assert(contains(drain(), "DIST ERR BUSY"));
  auto p = tick();
  assert(p.value == -3 && p.state == DistState::ACTIVE);
  const auto first_id = p.id;
  assert(contains(drain(), "DIST EVENT START"));
  for (int i = 0; i < 49; ++i) assert(tick().value == -3);
  assert(tick().value == 0); // exact 100ms half-open interval
  assert(contains(drain(), "DIST EVENT END"));
  command("DIST PULSE 10 20");
  p = tick();
  assert(p.id != first_id && p.value == 10);
  command("DIST CANCEL");
  assert(!disturbance_motor_active(p.id, now));
  assert(contains(drain(), "DIST EVENT CANCEL"));
  for (auto reason : {"STOP", "START_RESET", "CALIBRATION", "RESET"}) {
    tick(); command("DIST PULSE 3 100"); p = tick();
    const auto old_generation = disturbance_generation();
    disturbance_reset(now, reason);
    assert(!disturbance_motor_active(p.id, now));
    // A control tick already executing before STOP cannot rearm the generator.
    disturbance_update(now, true, 0, old_generation);
    command("DIST PULSE 3 100");
    assert(contains(drain(), "DIST ERR MODE"));
  }
  tick(); command("DIST PULSE 3 100"); tick();
  assert(tick(false).value == 0);
  assert(contains(drain(), "DIST EVENT MODE"));
  tick(true, 0.2f); command("DIST PULSE 3 100");
  assert(contains(drain(), "DIST ERR ANGLE"));
  tick(); command("DIST PULSE 3 100");
  assert(tick(true, 0.2f).value == 0);
  assert(contains(drain(), "DIST EVENT ANGLE"));
  tick(); command("DIST PULSE 3 200"); p = tick();
  now += 21000;
  assert(!disturbance_motor_active(p.id, now));
  assert(contains(drain(), "DIST EVENT STALE_CONTROL"));
  tick(); command("DIST PULSE 0 20"); p = tick();
  assert(p.state == DistState::ACTIVE && p.value == 0);
  for (int i = 0; i < 10; ++i) tick();
  assert(contains(drain(), "DIST EVENT END"));
  // Monotonic 64-bit time does not roll over at micros()' 32-bit boundary.
  now = (1LL << 32) - 4000; tick(); command("DIST PULSE 3 20"); tick();
  for (int i = 0; i < 10; ++i) tick();
  assert(contains(drain(), "DIST EVENT END"));
  command("DIST PULSE 3 200"); p = tick();
  for (int i = 0; i < 100; ++i) command("DIST?");
  command("DIST CANCEL");
  assert(!disturbance_motor_active(p.id, now));
  assert(contains(drain(), "DIST EVENT CANCEL"));
  drain();
  // KICK: waits for a calm pendulum, accelerates until alpha0/2, brakes for
  // the same time and ends with value 0 (LQR resumes).
  tick(); command("DIST KICK 4 100");
  assert(contains(drain(), "DIST OK KICK"));
  assert(tick(true, 0, 0.5f).state == DistState::PENDING);   // ~29 deg/s: not calm
  p = tick(true, 0, 0);
  assert(p.state == DistState::ACTIVE && p.kick && p.value == 100);
  for (int i = 0; i < 5; ++i) assert(tick(true, 0.01f).value == 100);  // 0.57 deg
  assert(tick(true, 0.036f).value == -100);                          // 2.06 deg >= 4/2
  int braking = 0;
  while (tick(true, 0.07f).value == -100) ++braking;
  assert(braking >= 5 && braking <= 7);
  assert(contains(drain(), "DIST EVENT END"));
  for (auto bad : {"DIST KICK 20 100", "DIST KICK 0.1 100", "DIST KICK 4 5", "DIST KICK 4 1001"}) {
    command(bad);
    assert(contains(drain(), "DIST ERR RANGE"));
  }
  command("DIST KICK -4 100");
  for (int i = 0; i < 1600; ++i) assert(tick(true, 0, 0.5f).value == 0);
  assert(contains(drain(), "DIST EVENT TIMEOUT"));
  std::cout << "PASS: parser, lifecycle, exact expiry, cancellation, mode, stale tick, overflow, rollover, kick\n";
}
