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