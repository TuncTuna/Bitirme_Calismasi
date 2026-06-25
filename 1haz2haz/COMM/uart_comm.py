# COMM/uart_comm.py
import serial
import serial.tools.list_ports
import struct
import time
from enum import IntEnum

# ── Frame constants ───────────────────────────────────────────
START       = b'\xFF'
STOP        = b'\xFE'
FRAME_SIZE  = 25          # was 24; +1 for the mode byte


# ── Mode definitions ──────────────────────────────────────────
class RobotMode(IntEnum):
    MANUAL   = 0x01   # Python sends angles continuously
    TEACHPAD = 0x02   # STM32 records positions internally
    AUTO     = 0x03   # STM32 runs stored trajectory


# ── CRC-16 ────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ── Frame builder ─────────────────────────────────────────────
def build_frame(
    dist1:       float,
    theta2:      float,
    theta3:      float,
    theta4:      float,
    gripper_cmd: float,
    mode:        RobotMode = RobotMode.MANUAL,
) -> bytes:
    """
    Frame layout (25 bytes):
      [0]     0xFF          START
      [1]     mode          RobotMode byte
      [2-21]  5 × float32   dist1, theta2, theta3, theta4, gripper_cmd
      [22-23] uint16 LE     CRC-16 over bytes [1-21] (mode + payload)
      [24]    0xFE          STOP
    """
    mode_byte = bytes([int(mode)])
    payload   = struct.pack('<5f', dist1, theta2, theta3, theta4, gripper_cmd)

    crc_input = mode_byte + payload
    crc       = crc16(crc_input)
    crc_bytes = struct.pack('<H', crc)

    frame = START + mode_byte + payload + crc_bytes + STOP

    if len(frame) != FRAME_SIZE:
        raise ValueError(f"Frame length {len(frame)} != {FRAME_SIZE}")

    return frame


# ── Port yardımcıları ─────────────────────────────────────────
def find_ftdi_port() -> str | None:
    """
    Bağlı portlar arasında FTDI FT232 (Vendor ID 0403) arar.
    Jetson'da ttyUSB0 olarak görünür.
    Bulamazsa None döner.
    """
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x0403:   # FTDI
            return port.device
    return None


def open_port(port: str | None = None, baud: int = 115200, timeout: float = 1.0) -> serial.Serial:
    """
    Belirtilen portu açar. port=None ise FTDI adaptörü otomatik bulunur.
    Jetson'da STM32 ST-Link (ttyACM0) değil, FTDI (ttyUSB0) kullanılır.
    """
    if port is None:
        port = find_ftdi_port()
        if port is None:
            raise IOError(
                "FTDI USB-UART adaptörü bulunamadı.\n"
                "  Bağlı portlar: " +
                ", ".join(p.device for p in serial.tools.list_ports.comports()) +
                "\n  STM32'yi FTDI adaptörü üzerinden bağlayın (/dev/ttyUSB0)."
            )
        print(f"[UART] FTDI port otomatik bulundu: {port}")

    ser = serial.Serial(
        port     = port,
        baudrate = baud,
        bytesize = serial.EIGHTBITS,
        parity   = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        timeout  = timeout,
        # Jetson'da DTR/RTS sinyalleri STM32'yi resetlememeli
        dsrdtr   = False,
        rtscts   = False,
    )
    # STM32 Arduino gibi reset atmaz → kısa bekleme yeterli
    time.sleep(0.1)
    print(f"[UART] Port açıldı: {port}  baud={baud}")
    return ser


def close_port(ser: serial.Serial) -> None:
    if ser and ser.is_open:
        ser.close()
        print("[UART] Port kapatıldı.")


# ── Send ──────────────────────────────────────────────────────
def send_frame(
    ser,
    dist1,
    theta2,
    theta3,
    theta4,
    gripper_cmd: float     = 0.0,
    mode:        RobotMode = RobotMode.MANUAL,
    debug:       bool      = False,
) -> bytes:
    frame = build_frame(
        float(dist1), float(theta2),
        float(theta3), float(theta4),
        float(gripper_cmd),
        mode,
    )
    ser.write(frame)
    ser.flush()

    if debug:
        print(f"[UART TX] mode={mode.name}  len={len(frame)}  hex={frame.hex()}")

    return frame


# ── Send Position → then same position with Gripper ───────────
def send_with_gripper(
    ser,
    dist1,
    theta2,
    theta3,
    theta4,
    gripper_cmd: float     = 0.0,
    mode:        RobotMode = RobotMode.MANUAL,
    delay:       float     = 0.05,
    debug:       bool      = False,
) -> None:
    """
    İki adımda gönderim:
      1) Konum paketi  → gripper_cmd = 0.0  (hareket)
      2) Aynı konum   → gripper_cmd = gerçek değer  (gripper komutu)
    """
    send_frame(ser, dist1, theta2, theta3, theta4,
               gripper_cmd=0.0, mode=mode, debug=debug)

    time.sleep(delay)

    if debug:
        print(f"[UART TX] gripper paketi gönderiliyor: grip={float(gripper_cmd):.2f}")

    send_frame(ser, dist1, theta2, theta3, theta4,
               gripper_cmd=float(gripper_cmd), mode=mode, debug=debug)


# ── DONE bekleme (STM → Jetson) ───────────────────────────────
def wait_for_done(ser, timeout: float = 1.0, debug: bool = False) -> bool:
    """
    STM'den 'DONE' satırını bekler.
    timeout süresi içinde gelirse True, gelmezse False döner.
    (Kısa timeout ile çağrılır; üst katman E-stop'a duyarlı şekilde döngüye alır.)
    """
    if ser is None:
        return False
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline()                 # ser.timeout zaten 1.0 s
        if not line:
            continue
        text = line.decode(errors="ignore").strip()
        if debug:
            print(f"[UART RX] {text!r}")
        if text == "DONE":
            return True
    return False
