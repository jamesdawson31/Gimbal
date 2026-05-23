import numpy as np
import control as ct 
import matplotlib.pyplot as plt
import sympy as sp

def rpm2radps(rpm):
    return np.pi/30 * rpm

# Define the system parameters
p = 11                              # Number of pole pairs
Rp = 5.6                            # Phase resistance (Ohms)
Lp = 200e-6                         # Phase inductance (H)                      ASSUMED
load_torque_1A = 800 * 9.81e-5      # Load torque @ 1A (Nm)
lam = 2*load_torque_1A/(3*p*1)      # Flux linkage     (Wb)
b = 5e-4                            # Viscous friction coefficient (Nms/rad)    ASSUMED
Js = 5e-5                           # Moment of inertia (kgm^2)                 ASSUMED
T_load = 0                          # Load torque (Nm)                          ASSUMED

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

# Create symbolic state vector
x = sp.Matrix([
    id,
    iq,
    theta_m_dot,
    theta_m
])

# Create symbolic control vector
u = sp.Matrix([
    ud,
    uq
])

# Create symbloc state derivative vector
x_dot = sp.Matrix([
    id_dot,
    iq_dot,
    theta_m_2dot,
    theta_m_dot
])

# Compute Jacobians
Jx = x_dot.jacobian(x)
Ju = x_dot.jacobian(u)

sp.pprint(Jx)
sp.pprint(Ju)

# Define operating point
theta_m_dot_op = rpm2radps(60)
iq_op = (2*b/(3*p*lam))*theta_m_dot_op
id_op = 0
theta_m_op = 0
uq_op = ((2*Rp*b/(3*p*lam)) + p*lam)*theta_m_dot_op
ud_op = -p*Lp*theta_m_dot_op*iq_op

# Evaluate Jacobians for linearised state-space matrices A and B
A = Jx.subs({
    id: id_op,
    iq: iq_op,
    theta_m_dot: theta_m_dot_op,
    theta_m: theta_m_op,
    ud: ud_op,
    uq: uq_op
})

B = Ju.subs({
    id: id_op,
    iq: iq_op,
    theta_m_dot: theta_m_dot_op,
    theta_m: theta_m_op,
    ud: ud_op,
    uq: uq_op
})

sp.pprint(A)
sp.pprint(B)

