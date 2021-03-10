# import sys
# sys.path.append('..')
# import numpy as np
# from chap4.mav_dynamics import mav_dynamics
# from chap5.compute_models import compute_ss_model
# from chap5.trim import compute_trim
# import parameters.quadrotor_parameters as MAV
# import parameters.simulation_parameters as SIM
# from tools.tools import Euler2Quaternion
#
# gravity = MAV.gravity
# #sigma =
# Va0 = MAV.Va0
# Vg = MAV.Va0
#
# #----------roll loop------------- (UAV book pg. 98)
# wn_phi = 10.0
# zeta_phi = 0.707
#
# a_phi_1 = -0.5*MAV.rho*Va0**2*MAV.S_wing*MAV.b*MAV.C_p_p*MAV.b/(2.0*Va0)
# a_phi_2 = 0.5*MAV.rho*Va0**2*MAV.S_wing*MAV.b*MAV.C_p_delta_a
# roll_kp = wn_phi**2/a_phi_2
# roll_kd = (2.*zeta_phi*wn_phi - a_phi_1)/a_phi_2
#
# #----------course loop------------- (UAV book pg. 101)
# W_phi_chi = 20.0
# wn_chi = wn_phi/W_phi_chi
# zeta_chi = 0.707
#
# course_kp = 2.*zeta_chi*wn_chi*Va0/gravity  #should be Vg instead of Va0
# course_ki = wn_chi**2*Va0/gravity  #should be Vg instead of Va0
#
# #----------sideslip loop-------------
# #----------sideslip loop------------- (UAV book pg. 103)
# wn_beta = 0.4
# zeta_beta = 0.6
#
# a_b_1 = -MAV.rho*Va0*MAV.S_wing/(2*MAV.mass)*MAV.C_Y_beta
# a_b_2 = MAV.rho*Va0*MAV.S_wing/(2*MAV.mass)*MAV.C_Y_delta_r
# sideslip_ki = -wn_beta**2/a_b_2
# sideslip_kp = -(2.*zeta_beta*wn_beta - a_b_1)/a_b_2
#
# #----------yaw damper------------- (UAV book pg. ??)
# yaw_damper_tau_r = 0.05
# yaw_damper_kp = 0.5
#
# #----------pitch loop------------- (UAV book pg. 105)
# wn_theta = 20.0
# zeta_theta = 1.707
#
# a_t_1 = -MAV.rho*Va0**2*MAV.c*MAV.S_wing/(2.*MAV.Jy)*MAV.C_m_q*MAV.c/(2.0*Va0)
# a_t_2 = -MAV.rho*Va0**2*MAV.c*MAV.S_wing/(2.*MAV.Jy)*MAV.C_m_alpha
# a_t_3 = MAV.rho*Va0**2*MAV.c*MAV.S_wing/(2.*MAV.Jy)*MAV.C_m_delta_e
# pitch_kp = (wn_theta**2 - a_t_2)/a_t_3
# pitch_kd = (2.*zeta_theta*wn_theta - a_t_1)/a_t_3
# K_theta_DC = pitch_kp*a_t_3/(a_t_2 + pitch_kp*a_t_3)
#
# #----------altitude loop------------- (UAV book pg. 107)
# W_h_theta = 50.0
# wn_h = wn_theta/W_h_theta
# zeta_h = 1.0
#
# altitude_kp = 2.*zeta_h*wn_h/(K_theta_DC*Va0)
# altitude_ki = wn_h**2/(K_theta_DC*Va0)
# # altitude_zone =
#
# #---------airspeed hold using throttle--------------- (UAV book pg. 109)
# wn_v = 5.0
# zeta_v = 2.0
#
# delta_t_star = MAV.delta_t_star
# delta_e_star = MAV.delta_e_star
# alpha_star = MAV.alpha_star
# a_v_1 = MAV.rho*MAV.Va0*MAV.S_prop/MAV.mass*(MAV.C_D_0 + MAV.C_D_alpha*alpha_star + MAV.C_D_delta_e*delta_e_star) + MAV.rho*MAV.S_prop/MAV.mass*MAV.C_prop*Va0
# a_v_2 = MAV.rho*MAV.S_prop/MAV.mass*MAV.C_prop*MAV.k_motor**2*delta_t_star
# airspeed_throttle_kp = wn_v**2/a_v_2
# airspeed_throttle_ki = (2.*zeta_v*wn_v - a_v_1)/a_v_2
#
# # --------------------------------------------------------------------------
# # ------------------------ LQR PARAMETERS ----------------------------------
# # --------------------------------------------------------------------------
# mav = mav_dynamics(SIM.ts_simulation)
# Va = 25.
# gamma = 0.*np.pi/180.
# R = np.inf
# trim_state, trim_input = compute_trim(mav, Va, gamma, R)
# A_lon, B_lon, A_lat, B_lat = compute_ss_model(mav, trim_state, trim_input, euler=True)
# # big value implies more cost for variable, low means low cost
# # Q_lat = np.diag((1.0/1.0, 1.0/1.0, 1.0/1.0, 1.0/2.0, 1.0/2.0, 1.0/40.0))  # v, p, r, phi, psi, psi_int
# Q_lat = np.diag((1.0/1.0, 1.0/1.0, 1.0/1.0, 0.0/2.0, 1.0/10.0, 1.0/100.0))  # v, p, r, phi, psi, psi_int
# R_lat = np.diag((50.0/1.0, 50.0/1.0))  # da, dr
# H_lat = np.array([[0, 0, 0, 0, 1]])
# A_lat_lqr = np.block([[A_lat, np.zeros((len(A_lat), len(H_lat)))],
#                       [H_lat, np.zeros((len(H_lat), len(H_lat)))]])
# B_lat_lqr = np.block([[B_lat],
#                       [np.zeros((len(H_lat), len(B_lat[0])))]])
# limit_lat_lqr = np.array([[np.radians(45.)], [-np.radians(45.)], [np.radians(45.)], [-np.radians(45.)]])
#
# #                u_tilde,     w,       q,        theta,   h_tilde, h_int,   Va_int
# Q_lon = np.diag((10000.0/1.0, 0.0/1.0, 1.0/10.0, 0.0/1.0, 20.0/1.0, 1.0/1.0, 10.0/1.0))  # u, w, q, theta, h, h_int, Va_int
# #                de,        dt
# R_lon = np.diag((600.0/1.0, 500.0/1.0))  # de, dt
#
# H_lon = np.array([[0, 0, 0, 0, 1], [1/Va, 1/Va, 0, 0, 0]])
# A_lon_lqr = np.block([[A_lon, np.zeros((len(A_lon), len(H_lon)))],
#                       [H_lon, np.zeros((len(H_lon), len(H_lon)))]])
# B_lon_lqr = np.block([[B_lon],
#                       [np.zeros((len(H_lon), len(B_lon[0])))]])
# limit_lon_lqr = np.array([[np.radians(45.)], [-np.radians(45.)], [1.0], [0.0]])  # +delta_a, -delta_a, +delta_r, -delta_r
#
# # --------------------------------------------------------------------------
# # ------------------------ TECS PARAMETERS ---------------------------------
# # --------------------------------------------------------------------------
# # 0 < k_T <= k_D
# K_tecs = np.array([[1.0, 3.6, 0.4, 0.4]])  #k_T, k_D, k_h, k_Va
# limit_tecs = np.array([[np.radians(45.)], [-np.radians(45.)], [1.0], [0.0]])  # +delta_e, -delta_e, +delta_t, -delta_t
# thrust_throttle_kp = 0.025
# thrust_throttle_ki = 0.02
# fpa_elevator_kp = -3.5
# fpa_elevator_ki = -2.0
# fpa_elevator_kd = -0.2