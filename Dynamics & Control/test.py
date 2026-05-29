import numpy as np
import matplotlib.pyplot as plt

def rpm2radps(rpm):
    return np.pi/30 * rpm

def r_square_wave(t, freq):
    T = 1 / freq

    if (t % T) < (T / 2):
        return 1
    else:
        return 0

t = np.linspace(0, 1, 100000)
wave = np.zeros_like(t)

for i in range(len(t)):
    wave[i] = r_square_wave(t[i], freq=10)

plt.plot(t, wave)
plt.show()
