import numpy as np

def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi