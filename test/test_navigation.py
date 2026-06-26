"""Testes unitarios dos modulos ROS-free (navigation e mission_params).

Estes modulos nao dependem do ROS, o que permite testa-los isoladamente e,
na fase 2, reutiliza-los num avaliador de fitness headless.
"""

import math

from evolutionary_explorer_ros import navigation as nav
from evolutionary_explorer_ros.mission_params import MissionParams


def test_clamp():
    assert nav.clamp(5.0, -1.0, 1.0) == 1.0
    assert nav.clamp(-5.0, -1.0, 1.0) == -1.0
    assert nav.clamp(0.3, -1.0, 1.0) == 0.3


def test_wrap_to_pi():
    assert abs(nav.wrap_to_pi(3.0 * math.pi)) - math.pi < 1e-6


def test_sector_min_distance_front():
    # Obstaculo a 0.5 m bem a frente (indice 0) num scan de 360 amostras.
    ranges = [99.0] * 360
    ranges[0] = 0.5
    ranges[1] = 0.6
    ranges[359] = 0.55
    inc = math.radians(1.0)
    d = nav.sector_min_distance(ranges, 0.0, inc, 0.0, math.radians(10),
                                range_min=0.1, range_max=100.0)
    assert abs(d - 0.5) < 1e-6


def test_sector_min_distance_clear():
    # Sem leituras validas -> retorna range_max (caminho livre).
    ranges = [float('inf')] * 360
    d = nav.sector_min_distance(ranges, 0.0, math.radians(1.0), 0.0,
                                math.radians(10), range_min=0.1, range_max=3.5)
    assert d == 3.5


def test_compute_sectors_blocked_side():
    ranges = [99.0] * 360
    # obstaculo a frente-esquerda (~+45 graus, indice ~45)
    for i in range(40, 51):
        ranges[i] = 0.4
    sectors = nav.compute_sectors(ranges, 0.0, math.radians(1.0),
                                  math.radians(35), math.radians(70),
                                  range_min=0.12, range_max=3.5)
    # Mais espaco a direita -> desviar para a direita.
    assert sectors.front_left < sectors.front_right
    assert sectors.front_blocked_side == 'right'


def test_genome_roundtrip_and_bounds():
    p = MissionParams()
    keys = MissionParams.genome_keys()
    bounds = MissionParams.bounds()
    genome = p.to_genome()
    assert len(genome) == len(keys) == len(bounds)

    # Valor acima do limite deve ser clipado ao maximo.
    g = list(genome)
    g[0] = 1e6
    p2 = MissionParams.from_genome(g)
    low, high = bounds[0]
    assert p2.to_genome()[0] == high

    # Campos estruturais (bool/int) sao preservados do base.
    assert p2.start_immediately == p.start_immediately
    assert p2.detect_confirm_frames == p.detect_confirm_frames
