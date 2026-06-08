#!/usr/bin/env python3
"""Funcoes auxiliares de navegacao (sem dependencia de ROS).

Concentra a matematica de baixo nivel usada pela maquina de estados:
  * setorizacao das leituras do LIDAR (frente / laterais),
  * utilitarios numericos (clamp, wrap de angulo).

Manter este modulo livre de ROS permite:
  * testar a logica isoladamente (pytest), e
  * reutilizar exatamente o mesmo codigo dentro de um avaliador de FITNESS
    headless quando a computacao evolutiva for adicionada (fase 2).
"""

import math
from dataclasses import dataclass


def clamp(value: float, lo: float, hi: float) -> float:
    """Limita ``value`` ao intervalo [lo, hi]."""
    return max(lo, min(hi, value))


def wrap_to_pi(angle: float) -> float:
    """Normaliza um angulo para o intervalo (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def sector_min_distance(ranges, angle_min, angle_increment,
                        center_rad, half_width_rad,
                        range_min=0.0, range_max=float('inf')) -> float:
    """Menor distancia valida dentro de um setor angular do LaserScan.

    Args:
        ranges: lista/array de distancias (sensor_msgs/LaserScan.ranges).
        angle_min: angulo da primeira leitura (rad).
        angle_increment: passo angular entre leituras (rad).
        center_rad: centro do setor desejado (rad), relativo ao frame do lidar.
        half_width_rad: meia-largura do setor (rad).
        range_min/range_max: limites validos do sensor (m).

    Returns:
        A menor distancia valida no setor; se nao houver leitura valida,
        retorna ``range_max`` (interpretado como "caminho livre").
    """
    n = len(ranges)
    if n == 0:
        return range_max

    best = float('inf')
    for i in range(n):
        angle = angle_min + i * angle_increment
        if abs(wrap_to_pi(angle - center_rad)) > half_width_rad:
            continue
        r = ranges[i]
        # descarta inf, nan e leituras fora da faixa valida do sensor
        if r is None or math.isinf(r) or math.isnan(r):
            continue
        if r <= range_min or r > range_max:
            continue
        if r < best:
            best = r

    return best if best != float('inf') else range_max


@dataclass
class ScanSectors:
    """Resumo das distancias minimas por setor (em metros)."""
    front: float
    front_left: float
    front_right: float
    left: float
    right: float

    @property
    def front_blocked_side(self) -> str:
        """Indica para que lado convem desviar: retorna 'left' se ha mais
        espaco a esquerda, 'right' caso contrario."""
        return 'left' if self.front_left >= self.front_right else 'right'


def compute_sectors(ranges, angle_min, angle_increment,
                    front_half_rad, side_half_rad,
                    range_min=0.0, range_max=float('inf')) -> ScanSectors:
    """Calcula as distancias minimas nos setores de interesse.

    Convencao do LIDAR do robo (URDF): 360 amostras, angle_min=0, varrendo no
    sentido anti-horario. Logo: 0 rad = frente, +pi/2 = esquerda,
    -pi/2 (== 3pi/2) = direita.
    """
    return ScanSectors(
        front=sector_min_distance(ranges, angle_min, angle_increment,
                                  0.0, front_half_rad, range_min, range_max),
        front_left=sector_min_distance(ranges, angle_min, angle_increment,
                                       math.radians(45), front_half_rad,
                                       range_min, range_max),
        front_right=sector_min_distance(ranges, angle_min, angle_increment,
                                        math.radians(-45), front_half_rad,
                                        range_min, range_max),
        left=sector_min_distance(ranges, angle_min, angle_increment,
                                 math.radians(90), side_half_rad,
                                 range_min, range_max),
        right=sector_min_distance(ranges, angle_min, angle_increment,
                                  math.radians(-90), side_half_rad,
                                  range_min, range_max),
    )
