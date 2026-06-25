# MOTION_CONTROL.PY

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

Vec3 = np.ndarray  # (3,)

@dataclass(frozen=True)
class AABB:
    xmin: float; xmax: float
    ymin: float; ymax: float
    zmin: float; zmax: float

    def inflate(self, m: float) -> "AABB":
        return AABB(self.xmin-m, self.xmax+m,
                    self.ymin-m, self.ymax+m,
                    self.zmin-m, self.zmax+m)

def aabb_intersect(a: AABB, b: AABB) -> bool:
    return not (a.xmax < b.xmin or a.xmin > b.xmax or
                a.ymax < b.ymin or a.ymin > b.ymax or
                a.zmax < b.zmin or a.zmin > b.zmax)

def sphere_intersect_aabb(center: Vec3, r: float, box: AABB) -> bool:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    px = min(max(cx, box.xmin), box.xmax)
    py = min(max(cy, box.ymin), box.ymax)
    pz = min(max(cz, box.zmin), box.zmax)
    d2 = (cx-px)**2 + (cy-py)**2 + (cz-pz)**2
    return d2 <= r*r

def lerp_path(p0: Vec3, p1: Vec3, step_mm: float = 5.0) -> list[Vec3]:
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    dist = float(np.linalg.norm(p1 - p0))
    n = max(2, int(dist / step_mm) + 1)
    return [p0 + (p1 - p0) * t for t in np.linspace(0, 1, n)]

def carried_aabb_at_pose(ee_pos: Vec3, size_xyz: tuple[float,float,float], offset_xyz=(0,0,0)) -> AABB:
    sx, sy, sz = size_xyz
    ox, oy, oz = offset_xyz
    x, y, z = float(ee_pos[0]+ox), float(ee_pos[1]+oy), float(ee_pos[2]+oz)
    return AABB(x - sx/2, x + sx/2, y - sy/2, y + sy/2, z - sz/2, z + sz/2)

@dataclass
class CollisionConfig:
    ee_radius: float = 25.0
    margin: float = 5.0
    carried_size: tuple[float,float,float] = (80, 50, 40)
    carried_offset: tuple[float,float,float] = (0, 0, 20)

@dataclass
class PlanConfig:
    step_mm: float = 5.0
    h_clear: float = 50.0
    z_safe: float = 100.0

def collides_pre(ee_pos: Vec3, obstacles: list[AABB], ccfg: CollisionConfig) -> bool:
    for obs in obstacles:
        if sphere_intersect_aabb(ee_pos, ccfg.ee_radius, obs.inflate(ccfg.margin)):
            return True
    return False

def collides_post(ee_pos: Vec3, obstacles: list[AABB], ccfg: CollisionConfig) -> bool:
    carried = carried_aabb_at_pose(ee_pos, ccfg.carried_size, ccfg.carried_offset).inflate(ccfg.margin)
    for obs in obstacles:
        obs_i = obs.inflate(ccfg.margin)
        if sphere_intersect_aabb(ee_pos, ccfg.ee_radius, obs_i):
            return True
        if aabb_intersect(carried, obs_i):
            return True
    return False

def _check_segment(cur: Vec3, wp: Vec3, obstacles: list[AABB], ccfg: CollisionConfig, pcfg: PlanConfig, post: bool):
    seg = lerp_path(cur, wp, step_mm=pcfg.step_mm)
    for p in seg:
        hit = collides_post(p, obstacles, ccfg) if post else collides_pre(p, obstacles, ccfg)
        if hit:
            return False, []
    return True, seg

def plan_pick(start_xyz: Vec3, pick_xyz: Vec3, obstacles: list[AABB],
              ccfg: CollisionConfig | None = None, pcfg: PlanConfig | None = None) -> tuple[bool, dict]:
    """
    PICK:
      - approach (pre)
      - descend (pre)
      - retreat (post)  # çünkü grasp sonrası artık parça var
      - lift to z_safe (post)
    """
    ccfg = ccfg or CollisionConfig()
    pcfg = pcfg or PlanConfig()

    start = np.array(start_xyz, dtype=float)
    pick = np.array(pick_xyz, dtype=float)
    approach = pick + np.array([0, 0, pcfg.h_clear], dtype=float)
    retreat = approach.copy()
    lift = np.array([retreat[0], retreat[1], max(pcfg.z_safe, retreat[2])], dtype=float)

    path: list[Vec3] = []
    cur = start

    ok, seg = _check_segment(cur, approach, obstacles, ccfg, pcfg, post=False)
    if not ok: return False, {"reason": "collision_pick_approach"}
    path += seg; cur = approach

    ok, seg = _check_segment(cur, pick, obstacles, ccfg, pcfg, post=False)
    if not ok: return False, {"reason": "collision_pick_descend"}
    path += seg; cur = pick

    ok, seg = _check_segment(cur, retreat, obstacles, ccfg, pcfg, post=True)
    if not ok: return False, {"reason": "collision_pick_retreat_with_part"}
    path += seg; cur = retreat

    ok, seg = _check_segment(cur, lift, obstacles, ccfg, pcfg, post=True)
    if not ok: return False, {"reason": "collision_pick_lift_with_part"}
    path += seg

    return True, {"path": path}

def plan_place(start_xyz: Vec3, place_xyz: Vec3, obstacles: list[AABB],
               ccfg: CollisionConfig | None = None, pcfg: PlanConfig | None = None) -> tuple[bool, dict]:
    """
    PLACE (holding=True varsayımı):
      - approach (post)
      - descend (post)
      - retreat (post)  # drop sonrası istersen pre’ye döndürebilirsin ama şimdilik güvenli taraf
    """
    ccfg = ccfg or CollisionConfig()
    pcfg = pcfg or PlanConfig()

    start = np.array(start_xyz, dtype=float)
    place = np.array(place_xyz, dtype=float)
    approach = place + np.array([0, 0, pcfg.h_clear], dtype=float)
    retreat = approach.copy()

    path: list[Vec3] = []
    cur = start

    ok, seg = _check_segment(cur, approach, obstacles, ccfg, pcfg, post=True)
    if not ok: return False, {"reason": "collision_place_approach"}
    path += seg; cur = approach

    ok, seg = _check_segment(cur, place, obstacles, ccfg, pcfg, post=True)
    if not ok: return False, {"reason": "collision_place_descend"}
    path += seg; cur = place

    ok, seg = _check_segment(cur, retreat, obstacles, ccfg, pcfg, post=True)
    if not ok: return False, {"reason": "collision_place_retreat"}
    path += seg

    return True, {"path": path}