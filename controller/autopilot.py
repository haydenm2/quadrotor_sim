# import sys
# import numpy as np
# sys.path.append('..')
# import parameters.control_parameters as AP
# import parameters.quadrotor_parameters as MAV
# from tools.transfer_function import transfer_function
# from tools.wrap import wrap
# from tools.tools import RotationBody2Vehicle
# from controller.h_infinity_controller import pid_control, pi_control, pd_control_with_rate
# from controller.hybrid_lqr_tecs_control import lqr_control, tecs_control
# from message_types.msg_state import msg_state
#
#
# class autopilot:
#     def __init__(self, ts_control):
#         self.lateral_control_type = 'PID'  # 'LQR' or 'PID'
#         self.longitudinal_control_type = 'TECS'  # 'TECS', 'LQR', or 'PID'
#
#         if self.lateral_control_type == 'LQR':
#             self.lateral_control = lqr_control(AP.A_lat_lqr, AP.B_lat_lqr, AP.Q_lat, AP.R_lat, AP.limit_lat_lqr, ts_control)
#
#         if self.longitudinal_control_type == 'TECS':
#             self.longitudinal_control = tecs_control(AP.K_tecs, AP.limit_tecs, ts_control, MAV)
#         elif self.longitudinal_control_type == 'LQR':
#             self.longitudinal_control = lqr_control(AP.A_lon_lqr, AP.B_lon_lqr, AP.Q_lon, AP.R_lon, AP.limit_lon_lqr, ts_control)
#
#         self.roll_from_aileron = pd_control_with_rate(
#                         kp=AP.roll_kp,
#                         kd=AP.roll_kd,
#                         limit=np.radians(45))
#         self.course_from_roll = pi_control(
#                         kp=AP.course_kp,
#                         ki=AP.course_ki,
#                         Ts=ts_control,
#                         limit=np.radians(30))
#         self.sideslip_from_rudder = pi_control(
#                         kp=AP.sideslip_kp,
#                         ki=AP.sideslip_ki,
#                         Ts=ts_control,
#                         limit=np.radians(45))
#         self.yaw_damper = transfer_function(
#                         num=np.array([[AP.yaw_damper_kp, 0]]),
#                         den=np.array([[1, 1/AP.yaw_damper_tau_r]]),
#                         Ts=ts_control)
#
#         # instantiate lateral controllers
#         self.pitch_from_elevator = pd_control_with_rate(
#                         kp=AP.pitch_kp,
#                         kd=AP.pitch_kd,
#                         limit=np.radians(45))
#         self.altitude_from_pitch = pi_control(
#                         kp=AP.altitude_kp,
#                         ki=AP.altitude_ki,
#                         Ts=ts_control,
#                         limit=np.radians(30))
#         self.airspeed_from_throttle = pi_control(
#                         kp=AP.airspeed_throttle_kp,
#                         ki=AP.airspeed_throttle_ki,
#                         Ts=ts_control,
#                         limit=1.0)
#         # common
#         self.commanded_state = msg_state()
#         self.delta = np.array([[MAV.delta_a_star, MAV.delta_e_star, MAV.delta_r_star, MAV.delta_t_star]])  #TODO: ensure start at equilibrium
#
#     def update(self, cmd, state):
#         # LATERAL CONTROLLER SELECTION
#         if self.lateral_control_type == 'LQR':
#             # LQR lateral autopilot
#             x_lat = np.array([[np.sin(state.beta) * state.Va, state.p, state.r, state.phi,
#                                state.chi]]).T  # using beta as an estimate for v
#             chi_c = cmd.course_command
#             chi = wrap(state.chi, cmd.course_command)
#             e_I_lat = chi - chi_c
#             x_lat[4, 0] = chi - chi_c
#             u_lateral = self.lateral_control.update(x_lat, e_I_lat)
#             delta_a = u_lateral.item(0)
#             delta_r = u_lateral.item(1)
#             phi_c = 0
#         elif self.lateral_control_type == 'PID':
#             # PID lateral autopilot
#             chi_c = wrap(cmd.course_command, state.chi)
#             phi_c = cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi)
#             delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
#             delta_r = self.yaw_damper.update(state.r)
#
#         # LONGITUDINAL CONTROLLER SELECTION
#         if self.longitudinal_control_type == 'TECS':
#             # TECS longitudinal autopilot
#             T = self.calculate_thrust(state, self.delta)
#             D = self.calculate_drag(state, self.delta)
#             x_tecs = np.array([[state.h, state.Vg, state.theta, state.q, T, D, state.alpha]]).T
#             command = np.array([[cmd.altitude_command], [cmd.airspeed_command]])
#             [u_longitudinal, theta_c] = self.longitudinal_control.update(x_tecs, command)
#             delta_e = u_longitudinal.item(0)
#             delta_t = u_longitudinal.item(1)
#         elif self.longitudinal_control_type == 'LQR':
#             # LQR longitudinal autopilot
#             x_lon = np.array([[np.sin(state.alpha) * state.Va, np.sin(state.alpha) * state.Va, state.q, state.theta,
#                                state.h]]).T  # u, w, q, theta, h
#             h_c = cmd.altitude_command
#             Va_c = cmd.airspeed_command
#             Va = state.Va
#             h = state.h
#             x_lon[0, 0] = np.sin(state.alpha) * Va - np.sin(state.alpha) * Va_c
#             x_lon[4, 0] = h - h_c
#             e_I_lon = np.array([[h - h_c],
#                                 [Va - Va_c]])
#             u_longitudinal = self.longitudinal_control.update(x_lon, e_I_lon)
#             delta_e = u_longitudinal.item(0)  # + MAV.delta_e_star
#             delta_t = u_longitudinal.item(1)  # + MAV.delta_t_star
#             theta_c = MAV.alpha_star
#         elif self.longitudinal_control_type == 'PID':
#             # PID longitudinal autopilot
#             h_c = cmd.altitude_command
#             theta_c = self.altitude_from_pitch.update(h_c, state.h)
#             delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
#             delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
#
#         # construct output and commanded states
#         self.delta = np.array([[delta_a], [delta_e], [delta_r], [delta_t]])
#         self.commanded_state.h = cmd.altitude_command
#         self.commanded_state.Va = cmd.airspeed_command
#         self.commanded_state.phi = phi_c
#         self.commanded_state.theta = theta_c
#         self.commanded_state.chi = cmd.course_command
#         return self.delta, self.commanded_state
#
#     def saturate(self, input, low_limit, up_limit):
#         if input <= low_limit:
#             output = low_limit
#         elif input >= up_limit:
#             output = up_limit
#         else:
#             output = input
#         return output
#
#     def calculate_thrust(self, state, delta):
#         delta_t = delta.item(3)
#         if delta_t < 0:
#             delta_t = 0.0
#         V_in = MAV.V_max*delta_t
#
#         # Quadratic formula to solve for motor speed
#         a1 = MAV.rho * MAV.D_prop**5 / ((2.0*np.pi)**2) * MAV.C_Q0
#         b1 = MAV.rho * MAV.D_prop**4 / (2.0*np.pi) * MAV.C_Q1 * state.Va + (MAV.KQ**2)/MAV.R_motor
#         c1 = MAV.rho * MAV.D_prop**3 * MAV.C_Q2 * state.Va**2 - MAV.KQ / MAV.R_motor * V_in + MAV.KQ * MAV.i0
#
#         # Consider only positive root
#         Omega_op = (-b1 + np.sqrt(b1**2 - 4 * a1 * c1)) / (2.0 * a1)
#
#         # compute advance ratio
#         J_op = 2 * np.pi * state.Va / (Omega_op * MAV.D_prop)
#
#         # compute non-dimensionalized coefficients of thrust and torque
#         C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0
#
#         # add thrust and torque due to propeller
#         n = Omega_op / (2.0 * np.pi)
#         T = MAV.rho * n**2 * MAV.D_prop**4 * C_T
#         return T
#
#     def calculate_drag(self, state, delta):
#         rho = MAV.rho
#         S = MAV.S_wing
#         a = state.alpha
#         c = MAV.c
#         q = state.q
#         Va = state.Va
#
#         # control surface offsets
#         delta_e = delta.item(1)
#
#         # drag coefficients
#         C_D_q = MAV.C_D_q
#         C_L_q = MAV.C_L_q
#         C_D_de = MAV.C_D_delta_e
#         C_L_de = MAV.C_L_delta_e
#
#         C_D = lambda alpha: MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha*alpha)**2/(np.pi*MAV.e*MAV.AR)
#         sig = lambda alpha: (1 + np.exp(-MAV.M*(alpha-MAV.alpha0))+np.exp(MAV.M*(alpha+MAV.alpha0)))/((1+np.exp(-MAV.M*(alpha-MAV.alpha0)))*(1+np.exp(MAV.M*(alpha+MAV.alpha0))))
#         C_L = lambda alpha: (1-sig(alpha))*(MAV.C_L_0+MAV.C_L_alpha*alpha)+sig(alpha)*(2*np.sign(alpha)*np.sin(alpha)**2*np.cos(alpha))
#         C_X = lambda alpha: -C_D(alpha)*np.cos(alpha) + C_L(alpha)*np.sin(alpha)
#         C_X_q = lambda alpha: -C_D_q*np.cos(alpha) + C_L_q*np.sin(alpha)
#         C_X_de = lambda alpha: -C_D_de*np.cos(alpha) + C_L_de*np.sin(alpha)
#         T_D = (0.5 * rho * Va**2 * S)*(C_X(a) + C_X_q(a)*(c/(2*Va))*q + C_X_de(a)*delta_e)
#         return T_D
