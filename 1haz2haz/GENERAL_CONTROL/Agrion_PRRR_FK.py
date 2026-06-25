# Agrion_PRRR_FK.py

import numpy as np
from config_init import L2, L3, L_GRIP, H1, H2

class Agrion_PRRR_FK:
    @staticmethod
    def solve(d1, Q2, Q3, Q4):
        d1 = float(d1); Q2 = float(Q2); Q3 = float(Q3); Q4 = float(Q4)

        phi = Q2 + Q3 + Q4

        xw = float(L2)*np.cos(Q2) + float(L3)*np.cos(Q2 + Q3)
        yw = float(L2)*np.sin(Q2) + float(L3)*np.sin(Q2 + Q3)

        x = xw + float(L_GRIP)*np.cos(phi)
        y = yw + float(L_GRIP)*np.sin(phi)
        z = d1 

        return float(x), float(y), float(z)