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

def calculate_SVPWM(u_dq, theta_e, Vdc):

    # Inverse Park and Clarke transforms
    u_abc = T_Clarke_inv() @ T_Park_inv(theta_e) @ u_dq

    # Common-mode voltage offset
    u_offset = -(np.max(u_abc) + np.min(u_abc))/2

    # Add common-mode offset
    u_abc_svpwm = u_abc + u_offset

    # Calculate duty cycles
    D = 0.5 + u_abc_svpwm/Vdc

    # Calculate average pole voltages
    u_pole = D * Vdc

    # Calculate floating motor neutral voltage
    u_neutral = np.mean(u_pole)

    # Calculate average phase-to-neutral voltages
    u_phase = u_pole - u_neutral

    return D, u_pole, u_phase


def calculate_u_sat(u_cmd, error, Vmax):
    # Apply actuator saturation effects to ensure the control signal is physically possible
    u_mag = np.linalg.norm(u_cmd)

    # Apply voltage vector saturation
    if u_mag > Vmax:

        u_sat = (Vmax/u_mag) * u_cmd

        # Conditional integration anti-windup
        # Only integrate if the error helps move the controller out of saturation
        print(u_cmd - u_sat)
        print(np.array([0, error]))

        if np.dot(u_cmd - u_sat, np.array([0, error])) > 0:
            q_dot = 0
        else:
            q_dot = error

    # Non-saturated
    else:
        u_sat = u_cmd.copy()
        q_dot = error

    return u_sat, q_dot