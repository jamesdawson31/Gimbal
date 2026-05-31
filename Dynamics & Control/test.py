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


a = np.array([1, 2, 3, 4, 5])
b = a[:4]
print(b)

std = np.array([0.1, 1, 10])
noise = np.random.normal(0, std)
print(noise)


plt.plot(t, wave)
plt.show()
