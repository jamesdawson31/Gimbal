import numpy as np

def rpm2radps(rpm):
    return np.pi/30 * rpm

def radps2rpm(radps):
    return radps * 30/np.pi

def rad2rev(rad):
    return rad / (2*np.pi)

def rev2rad(rev):
    return rev * (2*np.pi)

def LQI_input(xa, Ka):
    return - Ka @ xa

def r_square_wave(t, freq, rpm):
    T = 1 / freq

    if (t % T) > (T / 2):
        return rpm2radps(rpm)
    else:
        return 0
    
def r_sine_wave(t, freq, rpm):
    return rpm2radps(rpm) * np.sin(2*np.pi*freq*t)

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

def calculate_SVPWM(uq, ud, theta_e, Vdc):
    # Create d-q frame vector
    u_dq = np.array([ud, uq])

    # Inverse Park and Clarke transforms
    u_abc = T_Clarke_inv() @ T_Park_inv(theta_e) @ u_dq

    # Extracting each phase
    u_a = u_abc[0]
    u_b = u_abc[1]
    u_c = u_abc[2]

    # Common-mode voltage offset
    u_offset = -(max(u_a, u_b, u_c) + min(u_a, u_b, u_c))/2

    # Add the offset
    u_a += u_offset
    u_b += u_offset
    u_c += u_offset

    # Calculate duty cycles
    D_a = 0.5 + u_a/Vdc
    D_b = 0.5 + u_b/Vdc
    D_c = 0.5 + u_c/Vdc

    # Calculate average pole voltages
    u_a = D_a * Vdc
    u_b = D_b * Vdc
    u_c = D_c * Vdc

    # Calculate floating motor neutral voltage
    u_neutral = (u_a + u_b + u_c) / 3

    # Calculate average phase-to-neutral voltages
    u_phase_a = u_a - u_neutral
    u_phase_b = u_b - u_neutral
    u_phase_c = u_c - u_neutral

    return np.array([u_phase_a, u_phase_b, u_phase_c])
    