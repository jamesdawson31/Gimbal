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