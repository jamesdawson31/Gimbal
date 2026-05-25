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

## Define system dynamics and state-space representation
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

print("\nOperating Point Values:")
print(f"theta_m_dot_op = {float(theta_m_dot_op):.6f}")
print(f"iq_op          = {float(iq_op):.6f}")
print(f"id_op          = {float(id_op):.6f}")
print(f"theta_m_op     = {float(theta_m_op):.6f}")
print(f"uq_op          = {float(uq_op):.6f}")
print(f"ud_op          = {float(ud_op):.6f}")

# Evaluate Jacobians for linearised state-space matrices A and B
print("\nLinearised State-Space Matrices:")
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

# Convert to numpy arrays
A = np.array(A).astype(np.float64)
B = np.array(B).astype(np.float64)

# Calculate controllability matrix and rank
Ctrb = ct.ctrb(A, B)
rank = np.linalg.matrix_rank(Ctrb)
n_states = A.shape[0]

print(f"\nControllability Matrix (shape: {Ctrb.shape}):")
print(Ctrb)
print(f"Rank: {rank}, n_states: {n_states}")
if rank == n_states:
    print("\nSystem is controllable!")
else:
    print("\nSystem is NOT controllable")

## LQR Controller Design
# Define weighting matrices Q and R based on maximum expected values of states and inputs
id_max = 0.1
iq_max = 5
theta_m_dot_max = rpm2radps(2000)
theta_m_max = np.inf
Q = np.diag([1/id_max**2, 1/iq_max**2, 1/theta_m_dot_max**2, 1/theta_m_max**2])
ud_max = 12
uq_max = 12
R = np.diag([1/ud_max**2, 1/uq_max**2])

# Compute LQR gain matrix K
K, S, E = ct.lqr(A, B, Q, R)
print("\nLQR Gain Matrix K:")
print(K)

## Kalman Filter Design
# Define measurement matrix C and D
C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 0, 1]])
D = np.zeros((3, 2))

# Define process disturbance noise covariance W and measurement noise covariance V
W = np.diag([1e-6, 1e-6, 1e-6, 1e-6])
V = np.diag([1e-4, 1e-4, 1e-4])

# Increase robustness of Kalman filter by inflating process noise covariance W
mu = 0
W = W + mu*B@B.T

# Calculate observer gain matrix L using dual LQR design
L, S_kalman, E_kalman = ct.lqr(A.T, C.T, W, V)
L = L.T
print("\nKalman Filter Gain Matrix L:")
print(L)

# Check gain and phase margins
controller = ct.ss(A - B@K - L@C, L, K, D)
plant = ct.ss(A, B, C, D)

loop = controller * plant
ct.bode(loop, dB=True)
plt.show()