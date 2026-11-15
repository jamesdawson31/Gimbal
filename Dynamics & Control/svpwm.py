import numpy as np
import matplotlib.pyplot as plt
from Auxiliary_Functions import *

switching_states = np.array([
    [0,0,0],
    [1,0,0],
    [1,0,1],
    [1,1,0],
    [0,1,0],
    [0,1,1],
    [0,0,1],
    [1,1,1]
])

def calculate_basic_vector(state, V_dc):
    V_abc = V_dc * state

    V_alpha_beta = T_Clarke() @ V_abc

    return V_alpha_beta



if __name__ == "__main__":
    V_dc = 1

    state = switching_states[1]

    basic_vectors = np.zeros((8, 2))

    for i in range(basic_vectors.shape[0]):
        basic_vectors[i] = calculate_basic_vector(switching_states[i], V_dc)

    # basic_vectors = V_dc * T_Clarke() @ switching_states

    print(switching_states)
    print(basic_vectors)

    print(T_Park(np.pi/3))

    plt.figure(figsize=(7, 7))

    for i in range(basic_vectors.shape[0]):
        Valpha, Vbeta = basic_vectors[i]

        plt.quiver(
            0, 0, Valpha, Vbeta,
            angles="xy", scale_units="xy", scale=1,
            color="tab:blue"
        )

        plt.text(Valpha * 1.08, Vbeta * 1.08, str(switching_states[i]),
                ha="center", va="center")

    plt.axhline(0, color="gray", linewidth=0.8)
    plt.axvline(0, color="gray", linewidth=0.8)
    plt.xlim([-V_dc, V_dc])
    plt.ylim([-V_dc, V_dc])
    plt.xlabel("V_alpha (V)")
    plt.ylabel("V_beta (V)")
    plt.title("Three-Phase Inverter Switching Vectors")
    # plt.axis("equal")
    plt.grid(True)
    plt.show()
