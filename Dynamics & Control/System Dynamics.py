import numpy as np
import control as ct 
import matplotlib.pyplot as plt
import sympy as sp

# Define the system parameters
p = 11                              # Number of pole pairs
Rp = 5.6                            # Phase resistance (Ohms)
Lp = 1e-3                           # Phase inductance (H)                      ASSUMED
load_torque_1A = 800 * 9.81e-5      # Load torque @ 1A (Nm)
lam = 2*load_torque_1A/(3*p*1)      # Flux linkage     (Wb)
b = 0.01                            # Viscous friction coefficient (Nms/rad)    ASSUMED
Js = 1e-4                           # Moment of inertia (kgm^2)                 ASSUMED
T_load = 0                          # Load torque (Nm)                        ASSUMED

print(f"Flux linkage: {lam:.6f} Wb")

# Define symbolic state variables
id_dot, iq_dot, theta_m_2dot, theta_m_dot, id, iq, theta_m = sp.symbols(
    "id_dot iq_dot theta_m_2dot theta_m_dot id iq theta_m")

# Define symbolic control variables
ud, uq = sp.symbols("ud uq")

# Define nonlinear dynamics
id_dot = -(Rp/Lp)*id + (1/Lp)*ud + p*theta_m_dot*iq
iq_dot = -(Rp/Lp)*iq - (p*lam/Lp)*theta_m_dot + (1/Lp)*uq - p*theta_m_dot*id
theta_m_2dot = (3*p*lam/(2*Js))*iq - (b/Js)*theta_m_dot - T_load/Js
theta_m_dot = theta_m_dot

