import numpy as np
import control as ct 
import matplotlib.pyplot as plt
import sympy as sp
from scipy.integrate import solve_ivp

def rpm2radps(rpm):
    return np.pi/30 * rpm

def LQR_input(x, x_op, u_op, K):
    return u_op - K @ (x - x_op)

def LQI_input(xa, Ka):
    return - Ka @ xa

def r_square_wave(t, freq, rpm):
    T = 1 / freq

    if (t % T) > (T / 2):
        return rpm2radps(rpm)
    else:
        return 0

# Define the system parameters
p = 11                              # Number of pole pairs
Rp = 5.6                            # Phase resistance (Ohms)
Lp = 200e-6                         # Phase inductance (H)                      ASSUMED
load_torque_1A = 800 * 9.81e-5      # Load torque @ 1A (Nm)
lam = 2*load_torque_1A/(3*p*1)      # Flux linkage     (Wb)
b = 5e-4                            # Viscous friction coefficient (Nms/rad)    ASSUMED
Js = 5e-5                           # Moment of inertia (kgm^2)                 ASSUMED
T_load = 0                          # Load torque (Nm)                          ASSUMED

# Reference parameters for simulation
square_wave_freq = 15                # Frequency of reference square wave (Hz)
square_wave_rpm = 60                # Amplitude of reference square wave (rpm)

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
x_op = np.array([id_op, iq_op, theta_m_dot_op, theta_m_op])
uq_op = ((2*Rp*b/(3*p*lam)) + p*lam)*theta_m_dot_op
ud_op = -p*Lp*theta_m_dot_op*iq_op
u_op = np.array([ud_op, uq_op])

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

## Augmented system design for LQI controller
# Ci to only regulate theta_m_dot
Ci = np.array([[0, 0, 1, 0]])

# Augmented A matrix (Aa = [[A, 0], [-Ci, 0]])
Aa = np.block([
    [A, np.zeros((4,1))],
    [-Ci, 0]
])

# Augmented B matrix (Ba = [[B], [0]])
Ba = np.vstack((B, np.zeros((1,2))))

print("\nAugmented State-Space Matrices for LQI:")
print("Aa:")
print(Aa)
print("Ba:")
print(Ba)

## LQI Controller Design
# Define weighting matrices Qa and Ra based on maximum expected values of states and inputs
id_max = 0.1
iq_max = 5
theta_m_dot_max = rpm2radps(2000)
theta_m_max = np.inf
q_max = 0.1
Qa = np.diag([1/id_max**2, 1/iq_max**2, 1/theta_m_dot_max**2, 1/theta_m_max**2, 1/q_max**2])
ud_max = 12
uq_max = 12
Ra = np.diag([1/ud_max**2, 1/uq_max**2])

# Compute LQR gain matrix K
Ka, S, E = ct.lqr(Aa, Ba, Qa, Ra)
print("\nLQI Gain Matrix Ka:")
print(Ka)

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


## Simulate dynamics
def nonlinear_dynamics(t, x, Ka, L):
    # Unpack true states
    xa = x[:5]
    id, iq, theta_m_dot, theta_m, q = xa
    
    # Unpack estimated states
    x_hat = x[5:]
    id_hat, iq_hat, theta_m_dot_hat, theta_m_hat = x_hat

    # Output measurement
    y = C @ xa

    # Estimate state x_hat using Kalman filter


    # Compute control input using state feedback
    # Add gain interpolation later
    u = LQI_input(xa, Ka)
    ud = u[0]
    uq = u[1]

    # Compute state derivatives
    id_dot = -(Rp/Lp)*id + (1/Lp)*ud + p*theta_m_dot*iq
    iq_dot = -(Rp/Lp)*iq - (p*lam/Lp)*theta_m_dot + (1/Lp)*uq - p*theta_m_dot*id
    theta_m_2dot = (3*p*lam/(2*Js))*iq - (b/Js)*theta_m_dot - T_load/Js
    q_dot = r_square_wave(t, freq=square_wave_freq, rpm=square_wave_rpm) - theta_m_dot

    return np.array([id_dot, iq_dot, theta_m_2dot, theta_m_dot, q_dot])

# Initial states
id_0 = 0
iq_0 = 0
theta_m_dot_0 = 0#theta_m_dot_op
theta_m_0 = 0
q_0 = 0

x_0 = np.array([id_0, iq_0, theta_m_dot_0, theta_m_0, q_0])

# Control input
u = np.array([ud_op, uq_op])

# Simulation time
t_start = 0
t_end = 0.5
num_points = 1000
t_eval = np.linspace(t_start, t_end, num_points)

# Record other variables for visualisation
ud_sim = np.zeros(num_points)
uq_sim = np.zeros(num_points)

# Run simulation
sol = solve_ivp(
    nonlinear_dynamics,
    (t_start, t_end),
    x_0,
    args=(Ka,),
    t_eval=t_eval,
    method='RK45'
)

# Unpack results
t = sol.t
id_sim = sol.y[0]
iq_sim = sol.y[1]
theta_m_dot_sim = sol.y[2]
theta_m_sim = sol.y[3]

## Convert back to the abc frame for visualisation
# Define transformation matrices for Clarke and Park transforms
def T_Clarke():
    return (2/3) * np.array([
        [1, -0.5, -0.5],
        [0, np.sqrt(3)/2, -np.sqrt(3)/2]
    ])

def T_Clarke_inv():
    return np.array([
        [1, 0],
        [-0.5,  np.sqrt(3)/2],
        [-0.5, -np.sqrt(3)/2]
    ])

def T_Park(theta):
    return np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)]
    ])

def T_Park_inv(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

i_dq = np.column_stack((id_sim, iq_sim))
i_abc = np.zeros((len(t), 3))
u_abc = np.zeros((len(t), 3))
ref = np.zeros_like(t)

for i in range(len(t)):
    # Get current state
    theta_m_i = theta_m_sim[i]

    # Reconstruct reference
    ref[i] = r_square_wave(t[i], freq=square_wave_freq, rpm=square_wave_rpm)

    # Reconstruct ud and uq for visualisation
    # u_sim = u_op - K @ (sol.y[:, i] - x_op)
    u_sim = LQI_input(sol.y[:, i], Ka)
    ud_sim[i] = u_sim[0]
    uq_sim[i] = u_sim[1]

    # Inverse Clarke and Park transforms to get abc currents
    i_abc[i] = T_Clarke_inv() @ T_Park_inv(p*theta_m_i) @ np.array(i_dq[i])
    u_abc[i] = T_Clarke_inv() @ T_Park_inv(p*theta_m_i) @ u_sim


# Unpack phase currents and voltages
ia = i_abc[:, 0]
ib = i_abc[:, 1]
ic = i_abc[:, 2]
ua = u_abc[:, 0]
ub = u_abc[:, 1]
uc = u_abc[:, 2]


## Plot results
plt.figure(figsize=(10,8))

# id
plt.subplot(4,2,1)
plt.plot(t, id_sim)
plt.axhline(id_op, color='r', linestyle='--', label='id_op')
plt.ylabel('id (A)')
plt.grid()

# iq
plt.subplot(4,2,2)
plt.plot(t, iq_sim)
plt.axhline(iq_op, color='r', linestyle='--', label='iq_op')
plt.ylabel('iq (A)')
plt.grid()

# theta_m_dot
plt.subplot(4,2,3)
plt.plot(t, theta_m_dot_sim, label='Rotor speed')
plt.plot(t, ref, 'k--', label='Reference')
# plt.axhline(theta_m_dot_op, color='r', linestyle='--', label='theta_m_dot_op')
plt.ylabel('ωm (rad/s)')
plt.legend()
plt.grid()

# theta_m
plt.subplot(4,2,4)
plt.plot(t, theta_m_sim)
plt.ylabel('θm (rad)')
plt.grid()

# ud
plt.subplot(4,2,5)
plt.plot(t, ud_sim)
plt.axhline(ud_op, color='r', linestyle='--', label='ud_op')
plt.ylabel('ud (V)')
plt.grid()

# uq
plt.subplot(4,2,6)
plt.plot(t, uq_sim)
plt.axhline(uq_op, color='r', linestyle='--', label='uq_op')
plt.ylabel('uq (V)')
plt.grid()

# ia, ib, ic
plt.subplot(4,2,7)
plt.plot(t, ia, label='ia')
plt.plot(t, ib, label='ib')
plt.plot(t, ic, label='ic')
plt.legend()
plt.ylabel('Phase Current (A)')
plt.grid()

# ua, ub, uc
plt.subplot(4,2,8)
plt.plot(t, u_abc[:, 0], label='ua')
plt.plot(t, u_abc[:, 1], label='ub')
plt.plot(t, u_abc[:, 2], label='uc')
plt.legend()
plt.ylabel('Phase Voltage (V)')
plt.xlabel('Time (s)')
plt.grid()

plt.tight_layout()
plt.show()