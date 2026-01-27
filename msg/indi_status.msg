# msg/indi_status.msg

uint64 timestamp        # time since system start (microseconds)

float32[3] desired_alpha
float32[3] omega_dot
float32[3] residual_torque
float32[3] final_torque
float32[3] last3
float32[4] last_motor_thrusts
float32[4] motor_thrusts_n    # Actual force in Newtons
float32[4] motor_pwm_norm     # Normalized signal sent to ESC
float32[4] indi_torques
