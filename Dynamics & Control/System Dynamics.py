import numpy as np
import control as ct 
import matplotlib.pyplot as plt

# Define the system parameters
p = 11                              # Number of pole pairs
Rp = 5.6                            # Phase resistance (Ohms)
Lp = 1e-3                               # Phase inductance (H)                      ASSUMED
load_torque_1A = 800 * 9.81e-5      # Load torque @ 1A (Nm)
lam = 2*load_torque_1A/(3*p*1)      # Flux linkage     (Wb)
b = 0.01                            # Viscous friction coefficient (Nms/rad)    ASSUMED
Js = 1e-4                           # Moment of inertia (kgm^2)                 ASSUMED


print(f"Flux linkage: {lam:.6f} Wb")