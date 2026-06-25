# Agrion_PRRR_IK.py

import numpy as np
from config_init import L2, L3, L_GRIP, H1, H2


class Agrion_PRRR_IK:
    @staticmethod
    def solve(Xd, Yd, Zd, phi, elbow="up"):
        """
        Inputs:
          Xd, Yd, Zd : target end-effector position (mm)
          phi        : end-effector orientation (rad)
          elbow      : "up" or "down"

        Returns:
          d1, Q2, Q3, Q4
        """

        # 1) Prismatic
        d1 = float(Zd) # For now there is no offset in Z.

        # 2) Wrist center
        Xw = float(Xd) - float(L_GRIP) * np.cos(phi)
        Yw = float(Yd) - float(L_GRIP) * np.sin(phi)

        rw = Xw * Xw + Yw * Yw

        # 3) Q3
        denom = 2.0 * float(L2) * float(L3)
        cosQ3_raw = (rw - float(L2) ** 2 - float(L3) ** 2) / denom

        # ✅ Eski davranış: clip ile devam et (hata fırlatma yok)
        cosQ3 = np.clip(cosQ3_raw, -1.0, 1.0)

        # floating point güvenliği
        sin2 = max(0.0, float(1.0 - cosQ3 * cosQ3))
        sin_abs = np.sqrt(sin2)

        sinQ3 = sin_abs if str(elbow).lower() == "up" else -sin_abs
        Q3 = np.arctan2(sinQ3, cosQ3)

        # 4) Q1
        beta = np.arctan2(Yw, Xw)

        k1 = float(L2) + float(L3) * cosQ3
        k2 = float(L3) * sinQ3
        gamma = np.arctan2(k2, k1)

        Q2 = beta - gamma

        # 5) Q4
        Q4 = float(phi) - (Q2 + Q3)

        return float(d1), float(Q2), float(Q3), float(Q4) 