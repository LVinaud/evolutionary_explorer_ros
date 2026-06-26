#!/usr/bin/env python3
"""Mapa de ocupacao 2D construido em tempo real e planejamento por A estrela.

Diferente de um mapa pre-definido, esta grade comeca VAZIA e e preenchida pelo
robo enquanto explora, marcando como ocupadas as celulas onde o LIDAR detecta
obstaculos. Sobre essa grade construida, a busca A estrela encontra uma rota
livre ate um ponto objetivo, que a maquina de estados segue por waypoints. Isso
evita os minimos locais em que a navegacao puramente reativa prendia.

O modulo nao depende do ROS, o que permite testa-lo isoladamente e reutiliza-lo
num avaliador headless na fase de computacao evolutiva.
"""

import heapq
import math


class OccupancyPlanner:
    """Grade de ocupacao preenchida pelo LIDAR, com planejamento A estrela.

    Celulas comecam livres (desconhecido tratado como livre, de forma otimista,
    para o robo poder planejar rumo a regioes ainda nao vistas). Conforme o
    LIDAR detecta obstaculos, as celulas correspondentes ficam ocupadas, ja
    infladas pelo raio do robo para manter uma folga de seguranca.
    """

    def __init__(self, x_min=-11.0, x_max=11.0, y_min=-6.0, y_max=6.0,
                 resolution=0.20, robot_radius=0.30):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.res = resolution
        self.robot_radius = robot_radius
        self.nx = int(round((x_max - x_min) / resolution)) + 1
        self.ny = int(round((y_max - y_min) / resolution)) + 1
        self.occ = [[False] * self.ny for _ in range(self.nx)]
        self._infl_cells = max(1, int(round(robot_radius / resolution)))

    # ---- conversoes mundo <-> grade ----
    def to_cell(self, x, y):
        return (int(round((x - self.x_min) / self.res)),
                int(round((y - self.y_min) / self.res)))

    def to_world(self, i, j):
        return self.x_min + i * self.res, self.y_min + j * self.res

    def in_bounds(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def is_free(self, i, j):
        return self.in_bounds(i, j) and not self.occ[i][j]

    # ---- preenchimento pelo LIDAR ----
    def mark_obstacle(self, x, y):
        """Marca a celula do ponto (x, y) detectado pelo LIDAR como ocupada,
        inflando pelo raio do robo (em celulas) para manter folga."""
        ci, cj = self.to_cell(x, y)
        r = self._infl_cells
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    i, j = ci + di, cj + dj
                    if self.in_bounds(i, j):
                        self.occ[i][j] = True

    def update_from_scan(self, robot_x, robot_y, robot_yaw,
                         ranges, angle_min, angle_increment,
                         range_min, range_max):
        """Integra uma leitura de LaserScan na grade, no frame do mundo."""
        n = len(ranges)
        for k in range(n):
            r = ranges[k]
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= range_min or r >= range_max:
                continue
            a = robot_yaw + angle_min + k * angle_increment
            self.mark_obstacle(robot_x + r * math.cos(a),
                               robot_y + r * math.sin(a))

    def _nearest_free(self, i, j, max_radius=25):
        if self.is_free(i, j):
            return i, j
        for rr in range(1, max_radius + 1):
            for di in range(-rr, rr + 1):
                for dj in range(-rr, rr + 1):
                    if max(abs(di), abs(dj)) != rr:
                        continue
                    if self.is_free(i + di, j + dj):
                        return i + di, j + dj
        return None

    # ---- busca A estrela ----
    def plan(self, start_xy, goal_xy, simplify=True):
        """Rota de waypoints (x, y) do start ao goal sobre a grade atual, ou []
        se nao houver rota. Start e goal sao deslocados para a celula livre mais
        proxima caso caiam em area ocupada."""
        s = self._nearest_free(*self.to_cell(*start_xy))
        g = self._nearest_free(*self.to_cell(*goal_xy))
        if s is None or g is None:
            return []
        if s == g:
            return [self.to_world(*g)]

        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = [(h(s, g), 0.0, s)]
        came = {}
        gcost = {s: 0.0}
        closed = set()
        while open_heap:
            _, cost, cur = heapq.heappop(open_heap)
            if cur == g:
                return self._reconstruct(came, cur, simplify)
            if cur in closed:
                continue
            closed.add(cur)
            ci, cj = cur
            for di, dj in nbrs:
                ni, nj = ci + di, cj + dj
                if not self.is_free(ni, nj):
                    continue
                if di != 0 and dj != 0:
                    if not self.is_free(ci + di, cj) or not self.is_free(ci, cj + dj):
                        continue
                ng = cost + math.hypot(di, dj)
                if ng < gcost.get((ni, nj), float('inf')):
                    gcost[(ni, nj)] = ng
                    came[(ni, nj)] = cur
                    heapq.heappush(open_heap, (ng + h((ni, nj), g), ng, (ni, nj)))
        return []

    def _reconstruct(self, came, cur, simplify):
        cells = [cur]
        while cur in came:
            cur = came[cur]
            cells.append(cur)
        cells.reverse()
        if simplify:
            cells = self._simplify(cells)
        return [self.to_world(i, j) for i, j in cells]

    def _simplify(self, cells):
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        for k in range(1, len(cells) - 1):
            ax, ay = cells[k - 1]
            bx, by = cells[k]
            cx, cy = cells[k + 1]
            if (bx - ax, by - ay) != (cx - bx, cy - by):
                out.append(cells[k])
        out.append(cells[-1])
        return out
