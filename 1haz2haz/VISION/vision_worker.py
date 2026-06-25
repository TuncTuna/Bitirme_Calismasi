# VISION/vision_worker.py

import time
import uuid
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

try:
    import cv2
    import pyrealsense2 as rs
    from ultralytics import YOLO
    _LIBS_AVAILABLE = True
except ImportError:
    _LIBS_AVAILABLE = False
    print("[VisionWorker] cv2/pyrealsense2/ultralytics bulunamadı → MOCK MODE")


# =============================================================================
# SABITLER
# =============================================================================
MODEL_PATH        = '/home/koubots/bitirme_projesi/domates-3/runs/segment/domates_modelim/weights/best.engine'
BUFFER_SIZE       = 15
TIMEOUT           = 1.5
CONFIRMATION_TIME = 1.2

COLOR_CONFIG = {
    'fully': {'color': (0,   0,   255), 'label': 'OLGUN'},
    'half':  {'color': (0,   255, 255), 'label': 'YARI'},
    'green': {'color': (0,   255, 0),   'label': 'HAM'},
}


# =============================================================================
# VisionWorker
# =============================================================================
class VisionWorker(QThread):
    frame_ready     = pyqtSignal(object)
    targets_updated = pyqtSignal(list)
    status_changed  = pyqtSignal(str, str)
    error_occurred  = pyqtSignal(str)

    def __init__(self, confidence: float = 0.65, mock_mode: bool = True):
        super().__init__()
        self.confidence = confidence
        self.mock_mode  = mock_mode or not _LIBS_AVAILABLE
        self._running   = False
        self._locked    = False
        self._memory: dict = {}

    def stop(self):
        self._running = False

    def set_locked(self, locked: bool):
        self._locked = locked
        if not locked:
            self._memory.clear()

    def set_confidence(self, value: float):
        self.confidence = value

    def run(self):
        self._running = True
        if self.mock_mode:
            self._run_mock()
        else:
            self._run_realsense()

    # ── Gerçek kamera ────────────────────────────────────────────
    def _run_realsense(self):
        try:
            pipeline = rs.pipeline()
            config   = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            spatial      = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            temporal     = rs.temporal_filter()
            hole_filling = rs.hole_filling_filter(1)

            pipeline.start(config)
            align = rs.align(rs.stream.color)
            model = YOLO(MODEL_PATH, task='segment')
            names = model.names

        except Exception as e:
            self.error_occurred.emit(f"Kamera başlatılamadı: {e}")
            self._running = False
            return

        self.status_changed.emit("TARANIYOR", "#00c2a8")

        try:
            while self._running:
                frames         = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame    = aligned_frames.get_depth_frame()
                color_frame    = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                color_image  = np.asanyarray(color_frame.get_data())
                intrinsics   = color_frame.profile.as_video_stream_profile().intrinsics
                current_time = time.time()

                results = model.predict(
                    color_image, conf=self.confidence,
                    verbose=False, device=0,
                )

                if results[0].masks is not None:
                    depth_frame = spatial.process(depth_frame)
                    depth_frame = temporal.process(depth_frame)
                    depth_frame = hole_filling.process(depth_frame)
                    depth_array = np.asanyarray(depth_frame.get_data())

                    res = results[0]
                    for i in range(len(res.masks.xy)):
                        class_id   = int(res.boxes.cls[i])
                        raw_name   = names[class_id].lower()
                        short_name = (
                            'fully' if 'fully' in raw_name else
                            'half'  if 'half'  in raw_name else
                            'green'
                        )

                        cx, cy, w, h = map(int, res.boxes.xywh[i])

                        roi   = depth_array[
                            max(0,cy-2):min(480,cy+2),
                            max(0,cx-2):min(640,cx+2)
                        ]
                        valid = roi[roi > 0]
                        if len(valid) == 0:
                            continue

                        dist = np.median(valid) * 0.001
                        if not (0.1 < dist < 2.0):
                            continue

                        xyz    = rs.rs2_deproject_pixel_to_point(intrinsics,[cx,cy],dist)
                        coords = [xyz[0]*100, xyz[1]*100, xyz[2]*100]

                        if not self._locked:
                            self._update_memory(short_name, coords, current_time)

                        cfg  = COLOR_CONFIG[short_name]
                        x1,y1 = cx-w//2, cy-h//2
                        x2,y2 = cx+w//2, cy+h//2
                        cv2.rectangle(color_image,(x1,y1),(x2,y2),cfg['color'],1)
                        cv2.circle(color_image,(cx,cy),3,cfg['color'],-1)

                        tid = self._find_id(short_name, coords)
                        if tid:
                            avg   = np.mean(self._memory[tid]['history'],axis=0)
                            label = f"{cfg['label']} {avg[2]:.1f}cm"
                            cv2.putText(color_image,label,(x1,y1-5),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.35,cfg['color'],1,cv2.LINE_AA)

                if not self._locked:
                    self._cleanup(current_time)
                    stable = self._get_stable(current_time)
                    self.targets_updated.emit(stable)

                self.frame_ready.emit(color_image.copy())

        except Exception as e:
            self.error_occurred.emit(f"Kamera hatası: {e}")
        finally:
            pipeline.stop()
            self.status_changed.emit("KAMERA KAPALI", "#6a6a8a")

    # ── Mock modu ────────────────────────────────────────────────
    def _run_mock(self):
        """
        Robot limitleri içinde geçerli koordinatlar (mm cinsinden değil, cm):
        x: 0-575mm → 0-57.5cm
        y: -575-575mm → -57.5-57.5cm
        z: -30-175mm → -3-17.5cm
        """
        self.status_changed.emit("TARANIYOR (MOCK)", "#f0a040")

        mock_targets = [
            {'id': str(uuid.uuid4())[:8], 'class': 'fully',
             'coords': np.array([30.0,  5.0, 10.0])},   # 300mm, 50mm, 100mm
            {'id': str(uuid.uuid4())[:8], 'class': 'fully',
             'coords': np.array([35.0, -5.0,  8.0])},   # 350mm, -50mm, 80mm
            {'id': str(uuid.uuid4())[:8], 'class': 'half',
             'coords': np.array([25.0,  0.0, 12.0])},   # 250mm, 0mm, 120mm
        ]

        # ID'leri sabit tut (her frame'de yeni uuid üretme)
        frame_count = 0
        while self._running:
            img = np.zeros((480,640,3), dtype=np.uint8)
            img[:] = (20, 20, 30)

            cv2_available = False
            try:
                import cv2 as _cv2
                cv2_available = True
            except ImportError:
                pass

            if cv2_available:
                _cv2.putText(img, "MOCK MODE - Gercek kamera yok",
                            (100,240), _cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (100,100,200), 2)
                for idx, t in enumerate(mock_targets):
                    cfg = COLOR_CONFIG[t['class']]
                    cx  = 150 + idx * 160
                    cy  = 240
                    _cv2.rectangle(img,(cx-40,cy-40),(cx+40,cy+40),cfg['color'],2)
                    _cv2.putText(img, cfg['label'],
                                (cx-35,cy-45),
                                _cv2.FONT_HERSHEY_SIMPLEX,
                                0.4,cfg['color'],1)

            self.frame_ready.emit(img)

            if not self._locked:
                stable = [
                    {
                        'id':       t['id'],
                        'class':    t['class'],
                        'coords':   t['coords'],
                        'priority': 0 if t['class'] == 'fully' else 1,
                    }
                    for t in mock_targets
                ]
                stable.sort(key=lambda x: (x['priority'], x['coords'][2]))
                self.targets_updated.emit(stable)

            frame_count += 1
            time.sleep(0.033)

    # ── Yardımcı metodlar ────────────────────────────────────────
    def _find_id(self, short_name: str, coords: list) -> str | None:
        for tid, data in self._memory.items():
            if data['class'] != short_name:
                continue
            prev    = data['history'][-1]
            dist_sq = sum((coords[j]-prev[j])**2 for j in range(3))
            if dist_sq < 6.25:
                return tid
        return None

    def _update_memory(self, short_name: str, coords: list, current_time: float):
        found_id = self._find_id(short_name, coords)

        if found_id is None:
            # UUID kullan — ID çakışması yok
            found_id = str(uuid.uuid4())[:8]
            self._memory[found_id] = {
                'start_time': current_time,
                'history':    [],
                'class':      short_name,
                'last_seen':  current_time,
            }

        self._memory[found_id]['last_seen'] = current_time
        self._memory[found_id]['class']     = short_name
        self._memory[found_id]['history'].append(coords)

        if len(self._memory[found_id]['history']) > BUFFER_SIZE:
            self._memory[found_id]['history'].pop(0)

    def _cleanup(self, current_time: float):
        expired = [
            tid for tid, data in self._memory.items()
            if current_time - data['last_seen'] > TIMEOUT
        ]
        for tid in expired:
            del self._memory[tid]

    def _get_stable(self, current_time: float) -> list:
        stable = []
        for tid, data in self._memory.items():
            if current_time - data['start_time'] < CONFIRMATION_TIME:
                continue
            avg      = np.mean(data['history'], axis=0)
            priority = 0 if data['class'] == 'fully' else 1
            stable.append({
                'id':       tid,
                'class':    data['class'],
                'coords':   avg,
                'priority': priority,
            })
        stable.sort(key=lambda x: (x['priority'], x['coords'][2]))
        return stable
