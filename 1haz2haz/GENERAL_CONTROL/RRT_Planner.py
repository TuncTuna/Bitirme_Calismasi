import numpy as np
import random

class RRT:
    def __init__(self, q_start, q_goal, obstacles, bounds, FK_module, 
                 step_size=0.1, max_iter=2000, goal_sample_rate=0.2):
        self.q_start = np.array(q_start, dtype=float)
        self.q_goal = np.array(q_goal, dtype=float)
        self.obstacles = obstacles
        self.bounds = bounds  # [(min, max), (min, max), ...] 4 eklem için
        self.FK = FK_module
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        
        self.nodes = [self.Node(self.q_start)]

    class Node:
        def __init__(self, q, parent=None):
            self.q = q
            self.parent = parent

    def is_collision_free(self, q):
        # Robotun uç noktasının XYZ'sini FK ile al
        # (Gerekirse FK_module içinde eklem koordinatlarını da kontrol edebilirsin)
        x, y, z = self.FK.solve(q[0], q[1], q[2], q[3])
        pos = np.array([x, y, z])

        for obs in self.obstacles:
            center = np.array(obs["center"])
            dist = np.linalg.norm(pos - center)
            if dist <= obs["radius"]:
                return False # Çarpışma var
        return True

    def plan(self):
        for _ in range(self.max_iter):
            # 1. Örnekleme (Eklem Uzayında)
            if random.random() < self.goal_sample_rate:
                q_rand = self.q_goal
            else:
                q_rand = np.array([random.uniform(b[0], b[1]) for b in self.bounds])

            # 2. En Yakın Düğümü Bul
            nearest_node = min(self.nodes, key=lambda n: np.linalg.norm(n.q - q_rand))

            # 3. Yeni Noktaya Adım At (Steer)
            diff = q_rand - nearest_node.q
            dist = np.linalg.norm(diff)
            
            if dist > 0:
                q_new = nearest_node.q + (diff / dist) * self.step_size
                
                # 4. Çarpışma Kontrolü
                if self.is_collision_free(q_new):
                    new_node = self.Node(q_new, nearest_node)
                    self.nodes.append(new_node)

                    # Hedefe ulaştık mı?
                    if np.linalg.norm(q_new - self.q_goal) < self.step_size:
                        return self.extract_path(new_node)
        
        return [self.q_start, self.q_goal] # Yol bulunamazsa direkt gitmeyi dene (veya hata dön)

    def extract_path(self, node):
        path = []
        while node is not None:
            path.append(node.q)
            node = node.parent
        return path[::-1]
