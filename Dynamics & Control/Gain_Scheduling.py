import numpy as np
import sympy as sp
import control as ct
from Auxiliary_Functions import *

def linearise(theta_rpm, params):
    # Unpack parameters
    p = params['p']
    Rp = params['Rp']
    Lp = params['Lp']
    lam = params['lam']
    b = params['b']
    Js = params['Js']
    T_load = params['T_load']

    # Define symbolic state variables
    id_dot, iq_dot, theta_m_2dot, theta_m_dot, id, iq, theta_m = sp.symbols(
        "id_dot iq_dot theta_m_2dot theta_m_dot id iq theta_m"
    )

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
    theta_m_dot_op = rpm2radps(theta_rpm)
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
    
    return A, B

def design_lqig_controller(A, B, C, D, Ci, Qa, Ra, W, V, mu):
    # Augmented A matrix (Aa = [[A, 0], [-Ci, 0]])
    Aa = np.block([
        [A, np.zeros((4,1))],
        [-Ci, 0]
    ])

    # Augmented B matrix (Ba = [[B], [0]])
    Ba = np.vstack((B, np.zeros((1,2))))

    # print("\nAugmented State-Space Matrices for LQI:")
    # print("Aa:")
    # print(Aa)
    # print("Ba:")
    # print(Ba)

    # Compute LQR gain matrix Ka
    Ka, S, E = ct.lqr(Aa, Ba, Qa, Ra)

    # Increase robustness of Kalman filter by inflating process noise covariance W
    W = W + mu*B@B.T

    # Calculate observer gain matrix L using dual LQR design
    L, S_kalman, E_kalman = ct.lqr(A.T, C.T, W, V)
    L = L.T

    return Ka, L


def create_lookup_table(theta_rpm_set, C, D, Ci, Qa, Ra, W, V, mu, params, dir):
    for theta_rpm in theta_rpm_set:
        # Calculate all necessary matrices for LQIG controller
        A, B = linearise(theta_rpm, params)
        Ka, L = design_lqig_controller(A, B, C, D, Ci, Qa, Ra, W, V, mu)

        # Save to .npz files
        filename = f"{dir}/LQIG_Gains_{theta_rpm}rpm.npz"
        np.savez(filename, Ka=Ka, L=L)

def load_gains(theta_rpm_set, dir):
    Ka_list = []
    L_list = []

    for theta_rpm in theta_rpm_set:
        gains = np.load(f"{dir}/LQIG_Gains_{theta_rpm}rpm.npz")
        Ka_list.append(gains['Ka'])
        L_list.append(gains['L'])

    return Ka_list, L_list


def gain_scheduling(theta_rpm, theta_rpm_set, Ka_list, L_list, dir):
    Ka_interp = np.zeros_like(Ka_list[0])
    L_interp = np.zeros_like(L_list[0])

    if theta_rpm < theta_rpm_set[0]:
        Ka_interp = Ka_list[0]
        L_interp = L_list[0]
    elif theta_rpm > theta_rpm_set[-1]:
        Ka_interp = Ka_list[-1]
        L_interp = L_list[-1]
    else:
        for i in range(len(theta_rpm_set)-1):
            if theta_rpm_set[i] <= theta_rpm <= theta_rpm_set[i+1]:
                # Linear interpolation
                alpha = (theta_rpm - theta_rpm_set[i]) / (theta_rpm_set[i+1] - theta_rpm_set[i])
                # print(f"alpha: {alpha:.4f} for theta_rpm: {theta_rpm} between {theta_rpm_set[i]} and {theta_rpm_set[i+1]}")
                alpha = np.clip(alpha, 0, 1)  # Ensure alpha is between 0 and 1
                Ka_interp = (1-alpha)*Ka_list[i] + alpha*Ka_list[i+1]
                L_interp = (1-alpha)*L_list[i] + alpha*L_list[i+1]
                break

    return Ka_interp, L_interp