#pragma once

void control_task_start();

void control_request_start();
void control_stop();

void control_set_theta_target_rad(float theta_target_rad);
float control_get_theta_target_rad();
float control_get_theta_ref_rad();
void control_reset_theta_reference();