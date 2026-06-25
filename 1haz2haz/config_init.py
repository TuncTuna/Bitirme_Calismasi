"""
Robot Physical Parameters
PRRR Configuration
All units in mm and radians
"""

# CONFIG_INIT.PY

# =================================================
# UART / SERİ PORT AYARLARI
# =================================================
# Jetson Orin Nano'da mevcut portlar:
#   /dev/ttyUSB0  → FTDI FT232R USB-UART (STM32 haberleşme) ← kullanılan
#   /dev/ttyACM0  → STM32 ST-Link (sadece debug/program)
#   /dev/ttyTHS1  → Jetson native UART1 (GPIO pinleri, kullanılmıyor)
#   /dev/ttyTHS2  → Jetson native UART2 (GPIO pinleri, kullanılmıyor)
UART_PORT = 'COM18'
UART_BAUD = 115200

import numpy as np
from GENERAL_CONTROL.motion_control import AABB

# -------------------------------------------------
# LINK LENGTHS (mm)
# -------------------------------------------------
L2 = 320
L3 = 280
L_GRIP = 200

# -------------------------------------------------
# HEIGHT OFFSETS
# -------------------------------------------------
H1 = 5
H2 = 5

# -------------------------------------------------
# Initial Position (Home)
# -------------------------------------------------
HOME_Position = {
    'x': 339.41,
    'y': -376.568,
    'z': 0.0,
    'phi': 45.0
}

# -------------------------------------------------
# Waiting Position
# -------------------------------------------------
WAIT_Position = {
    'x': 800.0,
    'y': 0.0,
    'z': -150.0,
    'phi': 0.0
}

# -------------------------------------------------
# JOINT LIMITS
# -------------------------------------------------
LIMITS = {
    'x': [-800, 800],
    'y': [-800, 800],
    'z': [-360, 360],
    'phi': [-180, 180]
}# Workspace limits can be defined if needed

D1_MIN = -30
D1_MAX = 165.0

Q2_MIN = -np.pi
Q2_MAX =  np.pi

Q3_MIN = -np.pi
Q3_MAX =  np.pi

Q4_MIN = -np.pi
Q4_MAX =  np.pi




# =================================================
# COLLISION ENVIRONMENT
# =================================================
# AABB(xmin, xmax, ymin, ymax, zmin, zmax)
# Tüm değerler mm

OBSTACLES = [

    # Masa üzerindeki kutu
    AABB(
        xmin=150, xmax=300,
        ymin=-100, ymax=100,
        zmin=0,   zmax=120
    ),

    # Sol tarafta bir kolon
    AABB(
        xmin=-350, xmax=-250,
        ymin=-200, ymax=200,
        zmin=0,    zmax=300
    ),

    # Ön tarafta dar engel
    AABB(
        xmin=50,  xmax=120,
        ymin=250, ymax=350,
        zmin=0,   zmax=200
    )
]


# =================================================
# Planner Varsayılanları (isteğe bağlı)
# =================================================
STEP_MM = 5.0
H_CLEAR = 120.0
Z_SAFE  = 250.0


# =================================================
# CAMERA → ROBOT FRAME TRANSFORM
# =================================================
# Kameranın kendi frame'i (RealSense / OpenCV standardı):
#   +X_kam : kameranın sağı
#   +Y_kam : aşağı (gravite yönü)
#   +Z_kam : kameradan uzaklaşan derinlik (ileri)
#
# Robot frame'i (IK/FK içinde kullanılan):
#   +X_rob : robotun "ileri" baktığı yön (Q1=0 yönü)
#   +Y_rob : sağ-el kuralı (Q1 pozitif yönü)
#   +Z_rob : yukarı
#
# Eksen eşleştirmesi:
#   X_rob =  +Z_kam
#   Y_rob =  -X_kam
#   Z_rob =  -Y_kam
#
# Kameranın robot tabanına göre montaj konumu (mm):
#   Robot (0,0,0) noktasının üzerinde, sadece Z ekseninde yüksekte.
#     0 mm önde    (X_rob ekseninde)
#     0 mm yan     (Y_rob ekseninde)
#     ? mm yukarı  (Z_rob ekseninde) — kamerayı yerleştirince ölç ve gir
# -------------------------------------------------

# Kamera lensi → Robot taban (mm). Kalibrasyon değişirse buradan ayarla.
CAM_OFFSET_X = 0.0
CAM_OFFSET_Y = 0.0
CAM_OFFSET_Z = -105.0    # ← kamerayı yerleştirince ölç ve buraya gir (mm)


def cam_to_robot(x_cam_mm, y_cam_mm, z_cam_mm):
    """
    Kamera frame'inde mm cinsinden verilen bir noktayı,
    robot frame'inde mm cinsinden konuma çevirir.

    Giriş : (x_cam, y_cam, z_cam)  — kamera koordinatları, mm
    Çıkış : (x_rob, y_rob, z_rob)  — robot koordinatları, mm
    """
    x_rob = ( +float(z_cam_mm) ) + CAM_OFFSET_X
    y_rob = ( -float(x_cam_mm) ) + CAM_OFFSET_Y
    z_rob = ( -float(y_cam_mm) ) + CAM_OFFSET_Z
    return x_rob, y_rob, z_rob
