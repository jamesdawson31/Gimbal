import numpy as np
import matplotlib.pyplot as plt
from Auxiliary_Functions import *
from Gain_Scheduling import *

# def rpm2radps(rpm):
#     return np.pi/30 * rpm

# def r_square_wave(t, freq):
#     T = 1 / freq

#     if (t % T) < (T / 2):
#         return 1
#     else:
#         return 0

# t = np.linspace(0, 1, 100000)
# wave = np.zeros_like(t)

# for i in range(len(t)):
#     wave[i] = r_square_wave(t[i], freq=10)


# a = np.array([1, 2, 3, 4, 5])
# b = a[:4]
# print(b)

# std = np.array([0.1, 1, 10])
# noise = np.random.normal(0, std)
# print(noise)


# plt.plot(t, wave)
# plt.show()


# theta_rpm_set = np.arange(-2500, 2501, 500)
# print(theta_rpm_set)
# dir = "C:\\Users\\james.dawson\\Documents\\Projects\\Gimbal\\Dynamics & Control\\Gains"
# Ka_list, L_list = load_gains(theta_rpm_set, dir)
# Ka_neg2500 = Ka_list[0]
# Ka_neg2000 = Ka_list[1]

# print("Ka for -2500 rpm:")
# print(Ka_neg2500)
# print("\nKa for -2000 rpm:")
# print(Ka_neg2000)

# test_theta_rpm = -2250
# gain_scheduling(test_theta_rpm, theta_rpm_set, Ka_list, L_list, dir)
# Ka_interp, L_interp = gain_scheduling(test_theta_rpm, theta_rpm_set, Ka_list, L_list, dir)
# print("\nInterpolated Ka for theta_rpm =", test_theta_rpm)
# print(Ka_interp)
# print("\nInterpolated L for theta_rpm =", test_theta_rpm)
# print(L_interp)


# a = np.arange(0, 6*np.pi+0.1, 0.1)
# b = a % (2*np.pi)
# plt.plot(a, b)
# plt.show()

# for i in range(len(theta_rpm_set)):
#     gains = np.load(f"{dir}/LQIG_Gains_{theta_rpm_set[i]}rpm.npz")
#     Ka = gains['Ka']
#     L = gains['L']

#     print("\nTheta RPM:", theta_rpm_set[i])
#     print("Ka:")
#     print(Ka)
#     print("L:")
#     print(L)

# K = np.array([
#     [1, 2, 3, 4, 5],
#     [6, 7, 8, 9, 10]
# ])

# K_I = K[:, 4]

# Kaw = 1 * K_I
# u = np.array([10, 100])
# print(Kaw @ u)

u = np.array([1000, -10000])
u_sat = np.clip(u, -12, 12)

print(u_sat)