import numpy as np


def quintic_scalar(q0, qf, T, n=200):
    """
    5. derece polinom
    başlangıç ve bitiş hız/ivme = 0
    """

    t = np.linspace(0, T, n)

    a0 = q0
    a1 = 0
    a2 = 0
    a3 = 10*(qf - q0)/(T**3)
    a4 = -15*(qf - q0)/(T**4)
    a5 = 6*(qf - q0)/(T**5)

    q = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
    qd = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
    qdd = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3

    return t, q, qd, qdd


def quintic_joint_trajectory(q0_vec, qf_vec, T, n=200):
    """
    q0_vec = [d1_0,Q2_0, Q3_0, Q4_0]
    qf_vec = [d1_1,Q2_1,  Q3_1, Q4_1]
    """

    t, d1, d1d, d1dd = quintic_scalar(q0_vec[0], qf_vec[0], T, n)
    _, Q2, Q2d, Q2dd = quintic_scalar(q0_vec[1], qf_vec[1], T, n)
    _, Q3, Q3d, Q3dd = quintic_scalar(q0_vec[2], qf_vec[2], T, n)
    _, Q4, Q4d, Q4dd = quintic_scalar(q0_vec[3], qf_vec[3], T, n)

    return t, d1, Q2, Q3, Q4