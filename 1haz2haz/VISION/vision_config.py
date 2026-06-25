# VISION/vision_config.py

import json
from pathlib import Path

_CONFIG_PATH = Path(__file__).resolve().parent.parent / "vision_config.json"

DEFAULT_CONFIG = {
    "position_A": {"x": 300.0, "y": 0.0,   "z": 150.0, "phi": 0.0},
    "position_B": {"x": 100.0, "y": 400.0, "z": 150.0, "phi": 0.0},
    "scan_timeout":  2.5,
    "confidence":    0.65,
    "run_mode":      "full_auto",
}


def load_config() -> dict:
    """
    JSON'dan config oku.
    Dosya yoksa veya bozuksa DEFAULT_CONFIG döner.
    Eksik anahtarlar default ile tamamlanır.
    """
    if not _CONFIG_PATH.exists():
        return DEFAULT_CONFIG.copy()

    try:
        with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        print(f"[vision_config] Okuma hatası, default kullanılıyor: {e}")
        return DEFAULT_CONFIG.copy()

    cfg = DEFAULT_CONFIG.copy()
    cfg.update(data)
    return cfg


def save_config(cfg: dict) -> bool:
    """
    Tüm config'i JSON'a yaz.
    Başarılıysa True, hata olursa False döner.
    """
    try:
        with open(_CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        return True
    except IOError as e:
        print(f"[vision_config] Yazma hatası: {e}")
        return False


def save_all(pos_a: dict, pos_b: dict,
             scan_timeout: float, confidence: float,
             run_mode: str) -> bool:
    """
    GUI'deki 'Kaydet' butonuna basınca çağrılır.
    A/B konumları + tarama süresi + güven eşiği + mod → JSON'a yazar.

    pos_a / pos_b formatı:
        {"x": float, "y": float, "z": float, "phi": float}
    run_mode:
        "full_auto" veya "step_by_step"
    """
    cfg = load_config()
    cfg["position_A"]    = pos_a
    cfg["position_B"]    = pos_b
    cfg["scan_timeout"]  = scan_timeout
    cfg["confidence"]    = confidence
    cfg["run_mode"]      = run_mode
    return save_config(cfg)
