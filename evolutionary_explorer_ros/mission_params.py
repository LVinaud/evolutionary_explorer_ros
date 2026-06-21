#!/usr/bin/env python3
"""Parametros de comportamento da missao (o "genoma" do robo).

Este modulo concentra TODOS os ganhos e limiares que governam a exploracao,
o desvio de obstaculos e a navegacao/posicionamento ate a bandeira. A ideia e
que a logica de controle (maquina de estados) leia exclusivamente daqui, de
forma que:

  * Na entrega atual (Trabalho 1) os valores vem de config/mission_params.yaml
    (parametros ROS 2), podendo ser ajustados sem recompilar.

  * No segundo momento do projeto, um algoritmo de COMPUTACAO EVOLUTIVA podera
    tratar o vetor de parametros como um cromossomo (genoma): basta usar
    ``MissionParams.from_genome(...)`` / ``params.to_genome()`` e os limites em
    ``MissionParams.GENOME_BOUNDS``. Nenhuma mudanca na maquina de estados sera
    necessaria para evoluir o comportamento -- somente os valores mudam.

Mantendo os parametros isolados do codigo de controle garantimos baixo
acoplamento entre "o que o robo faz" (estados/comportamentos) e "como ele faz"
(ganhos numericos), que e exatamente o ponto de injecao da evolucao.
"""

from dataclasses import dataclass, fields
from typing import Dict, List, Tuple


@dataclass
class MissionParams:
    """Conjunto de parametros sintonizaveis da missao.

    Os defaults abaixo formam uma configuracao funcional "feita a mao" para a
    arena padrao (arena_cilindros). O algoritmo evolutivo, no futuro, busca
    valores melhores dentro de ``GENOME_BOUNDS``.
    """

    # ------------------------------------------------------------------ #
    # Loop de controle
    # ------------------------------------------------------------------ #
    control_frequency: float = 20.0     # Hz do loop da maquina de estados
    start_immediately: bool = True      # se False, comeca em AGUARDANDO_COMANDO

    # ------------------------------------------------------------------ #
    # Estado EXPLORANDO
    # Movimento de varredura (serpentina) sobreposto a um avanco constante.
    # ------------------------------------------------------------------ #
    explore_linear_speed: float = 0.30        # m/s ao explorar (reduzido p/ estabilidade)
    explore_serpentine_gain: float = 0.4      # rad/s amplitude da serpentina
    explore_serpentine_period: float = 8.0    # s periodo da serpentina
    # Vies direcional: o robo sabe que a bandeira inimiga esta do outro lado da
    # arena (+x, a partir da base vermelha). Empurra a exploracao p/ esse rumo.
    explore_target_heading: float = 0.0       # rad (0 = +x, lado inimigo)
    explore_heading_kp: float = 0.5           # ganho de ATRACAO p/ o rumo alvo
    repulse_gain: float = 2.0                 # ganho de REPULSAO dos obstaculos
    #   (exploracao = campo potencial: atrai p/ +x e repele de obstaculos ->
    #    o robo CONTORNA os cilindros em vez de empurra-los)
    # Anti-travamento: se o robo nao progride, faz uma manobra de escape.
    stuck_window: float = 6.0                 # s observados p/ detectar travamento
    stuck_min_progress: float = 0.4           # m: deslocamento minimo na janela
    escape_duration: float = 2.5              # s de manobra de escape
    escape_turn_speed: float = 0.9            # rad/s do giro de escape

    # ------------------------------------------------------------------ #
    # Desvio de obstaculos com LIDAR (ativo em EXPLORANDO e NAVEGANDO)
    # ------------------------------------------------------------------ #
    front_sector_half_angle_deg: float = 35.0   # meia-largura do setor frontal
    side_sector_half_angle_deg: float = 70.0    # meia-largura dos setores laterais
    obstacle_block_distance: float = 1.1         # m: dist. frontal que aciona desvio (cedo)
    emergency_stop_distance: float = 0.45        # m: dist. critica (para de avancar)
    avoid_linear_speed: float = 0.10             # m/s durante o desvio
    avoid_angular_speed: float = 0.8             # rad/s durante o desvio

    # ------------------------------------------------------------------ #
    # Estado NAVEGANDO_PARA_BANDEIRA
    # Controle proporcional sobre o deslocamento horizontal da bandeira na
    # imagem (offset normalizado em [-1, 1]).
    # ------------------------------------------------------------------ #
    nav_linear_speed: float = 0.30        # m/s ao navegar para a bandeira
    nav_angular_kp: float = 1.6           # ganho P sobre o offset horizontal
    nav_max_angular_speed: float = 1.0    # rad/s saturacao do giro

    # ------------------------------------------------------------------ #
    # Estado POSICIONANDO_PARA_COLETA
    # ------------------------------------------------------------------ #
    # Gatilho NAV -> POSICIONANDO: a bandeira aparece pequena/de lado na camera,
    # entao usamos PRINCIPALMENTE a distancia frontal do LIDAR (approach_distance)
    # quando a bandeira esta centralizada; a area (approach_area_ratio) e um
    # gatilho secundario (caso o mastro fino escape do LIDAR).
    approach_distance: float = 1.3        # m: dist. frontal p/ NAV->POSICIONANDO
    approach_area_ratio: float = 0.03     # area p/ NAV->POSICIONANDO (entra mais perto)
    # Conclusao do posicionamento: a bandeira "grande o suficiente" na imagem
    # indica proximidade (sinal visual robusto, ao contrario do mastro fino no
    # LIDAR). O LIDAR (emergency_stop_distance) entra apenas como seguranca.
    complete_area_ratio: float = 0.03     # area p/ considerar POSICIONADO (perto)
    target_stop_distance: float = 0.6     # m: dist. frontal de referencia/seguranca
    position_linear_kp: float = 0.6       # ganho da velocidade de aproximacao
    position_angular_kp: float = 2.0      # ganho P de centralizacao fina
    position_steer_kp: float = 1.2        # ganho de esterco na perseguicao suave
    centering_tolerance: float = 0.06     # offset normalizado considerado centrado

    # ------------------------------------------------------------------ #
    # Garra e captura da bandeira (Trabalho 2)
    # ------------------------------------------------------------------ #
    # Poses da garra publicadas em /gripper_controller/commands, NA ORDEM do
    # controlador do professor: [elevacao_haste, braco_DIREITO, braco_ESQUERDO].
    # Faixas: elevacao -1.57..0.2 rad; direito -0.06..0; esquerdo 0..0.06.
    grip_capture_elev: float = 0.0     # haste na altura do mastro p/ agarrar
    grip_open_right: float = -0.06     # braco direito aberto
    grip_open_left: float = 0.06       # braco esquerdo aberto
    grip_closed_right: float = 0.0     # braco direito fechado (prende o mastro)
    grip_closed_left: float = 0.0      # braco esquerdo fechado
    grip_carry_elev: float = -0.4      # haste ao transportar (segura o mastro)
    # Vies lateral de mira na captura: o centro do blob azul (mastro + painel)
    # fica puxado para o lado do painel, entao o robo mira ao lado do mastro.
    # Este vies, somado ao deslocamento medido, corrige a mira para o mastro.
    # Positivo mira mais para a direita; ajustar o sinal/valor olhando a captura.
    grasp_aim_bias: float = 0.12
    # Area da bandeira na imagem para iniciar a captura (bandeira BEM perto).
    grasp_area_ratio: float = 0.05
    # Distancia frontal abaixo da qual, se a bandeira ainda esta pequena na
    # imagem (area < grasp_area_ratio), o que esta a frente NAO e a bandeira e
    # sim um obstaculo (um cilindro entre o robo e a bandeira). Serve para o
    # posicionamento voltar a navegar e contornar, em vez de avancar contra ele.
    grasp_clear_distance: float = 0.5
    # Sequencia temporizada da captura (segundos de cada etapa).
    capture_settle_time: float = 1.5   # haste desce e bracos abrem
    capture_creep_time: float = 3.5    # avanca devagar enfiando o mastro
    capture_creep_speed: float = 0.08  # m/s no avanco de captura
    capture_close_time: float = 1.5    # fecha os bracos no mastro
    capture_raise_time: float = 1.5    # eleva a haste com a bandeira presa

    # ------------------------------------------------------------------ #
    # Retorno a base e deposito (Trabalho 2)
    # ------------------------------------------------------------------ #
    # O robo nasce sobre o centro da base; usamos a pose inicial como destino
    # do deposito, dentro do circulo amarelo demarcado.
    deposit_tolerance: float = 0.5     # m: raio ao redor da pose inicial p/ depositar
    deposit_release_time: float = 1.5  # s abrindo a garra p/ soltar a bandeira
    deposit_backup_time: float = 2.0   # s recuando apos soltar p/ nao reencostar
    deposit_backup_speed: float = 0.12 # m/s no recuo
    # Ao transportar a bandeira presa, o LIDAR enxerga o proprio mastro logo a
    # frente. Ignoramos os retornos mais proximos que isto (no mapa e na
    # seguranca frontal) para o robo nao tratar a propria bandeira como parede.
    carry_blind_range: float = 0.8     # m: alcance frontal ignorado ao carregar
    turn_in_place_threshold: float = 0.7  # rad: acima disso, gira no lugar (nao avanca)

    # ------------------------------------------------------------------ #
    # Navegacao planejada por A estrela sobre a grade construida pelo LIDAR.
    # O robo NAO conhece os obstaculos de antemao: ele monta a grade enquanto
    # explora. A estrela planeja uma rota livre ate um ponto a frente, na
    # direcao desejada (rumo ao inimigo na exploracao, rumo a bandeira vista
    # pela camera na navegacao). Isso evita os minimos locais do metodo reativo.
    # ------------------------------------------------------------------ #
    plan_resolution: float = 0.20    # m por celula da grade de ocupacao
    plan_robot_radius: float = 0.30  # raio do robo p/ inflar os obstaculos
    lookahead_distance: float = 3.0  # m: ponto-alvo a frente, na direcao desejada
    goto_angular_kp: float = 1.8     # ganho de rumo ao seguir um waypoint
    waypoint_tolerance: float = 0.45  # m: distancia p/ considerar waypoint atingido
    replan_period: float = 1.5       # s: re-planeja sobre a grade atualizada

    # ------------------------------------------------------------------ #
    # Deteccao e transicoes da maquina de estados
    # ------------------------------------------------------------------ #
    detect_confirm_frames: int = 1        # frames p/ confirmar deteccao (1 = trava rapido)
    detect_lost_timeout: float = 3.5      # s sem ver a bandeira -> REDETECTANDO
    #   (tolera deteccao intermitente a longa distancia, evitando vai-e-vem)
    redetect_spin_speed: float = 0.7      # rad/s ao girar procurando de novo
    redetect_timeout: float = 8.0         # s sem reencontrar -> volta a EXPLORANDO

    # ================================================================== #
    # Suporte a COMPUTACAO EVOLUTIVA (fase 2)
    # ------------------------------------------------------------------ #
    # O subconjunto de parametros CONTINUOS evoluiveis e seus limites
    # (min, max) ficam em _GENOME_BOUNDS (modulo). Campos booleanos/inteiros
    # sao escolhas estruturais e ficam de fora do genoma continuo. A ordem do
    # dicionario define a ordem do vetor de genoma.
    # ------------------------------------------------------------------ #

    # ------------------------------------------------------------------ #
    # Helpers de genoma
    # ------------------------------------------------------------------ #
    @classmethod
    def genome_keys(cls) -> List[str]:
        """Nomes dos parametros evoluiveis, na ordem do vetor de genoma."""
        return list(_GENOME_BOUNDS.keys())

    @classmethod
    def bounds(cls) -> List[Tuple[float, float]]:
        """Limites (min, max) de cada gene, na mesma ordem de genome_keys()."""
        return list(_GENOME_BOUNDS.values())

    def to_genome(self) -> List[float]:
        """Serializa os parametros evoluiveis num vetor (cromossomo)."""
        return [float(getattr(self, k)) for k in self.genome_keys()]

    @classmethod
    def from_genome(cls, genome: List[float], base: "MissionParams" = None) -> "MissionParams":
        """Constroi um MissionParams a partir de um vetor de genoma.

        Campos nao presentes no genoma (bools/ints) sao herdados de ``base``
        (ou dos defaults). Os valores sao limitados (clipados) aos bounds.
        """
        params = base if base is not None else cls()
        keys = cls.genome_keys()
        bounds = cls.bounds()
        for value, key, (low, high) in zip(genome, keys, bounds):
            setattr(params, key, float(min(max(value, low), high)))
        return params

    # ------------------------------------------------------------------ #
    # Integracao com parametros ROS 2
    # ------------------------------------------------------------------ #
    @classmethod
    def declare_and_read(cls, node) -> "MissionParams":
        """Declara cada campo como parametro ROS 2 no ``node`` (usando os
        defaults da dataclass) e le os valores efetivos (que podem ter sido
        sobrescritos por um arquivo YAML ou pela linha de comando). Retorna a
        instancia preenchida.

        E este o ponto que permite a um driver evolutivo, no futuro, lancar o
        nodo com um arquivo de parametros diferente a cada episodio.
        """
        defaults = cls()
        params = cls()
        for f in fields(cls):
            default_value = getattr(defaults, f.name)
            node.declare_parameter(f.name, default_value)
            value = node.get_parameter(f.name).value
            setattr(params, f.name, value)
        return params


# Limites de evolucao definidos fora da dataclass para nao virarem "campos".
_GENOME_BOUNDS: Dict[str, Tuple[float, float]] = {
    # Exploracao
    'explore_linear_speed':       (0.10, 0.80),
    'explore_serpentine_gain':    (0.00, 1.20),
    'explore_serpentine_period':  (3.00, 15.0),
    'explore_heading_kp':         (0.00, 2.00),
    'repulse_gain':               (0.50, 4.00),
    'escape_turn_speed':          (0.40, 1.50),
    # Desvio de obstaculos
    'front_sector_half_angle_deg': (15.0, 60.0),
    'obstacle_block_distance':    (0.50, 1.50),
    'emergency_stop_distance':    (0.20, 0.60),
    'avoid_linear_speed':         (0.00, 0.30),
    'avoid_angular_speed':        (0.40, 1.80),
    # Navegacao
    'nav_linear_speed':           (0.10, 0.80),
    'nav_angular_kp':             (0.50, 3.50),
    'nav_max_angular_speed':      (0.60, 2.00),
    # Posicionamento
    'approach_distance':          (0.80, 2.00),
    'approach_area_ratio':        (0.005, 0.10),
    'complete_area_ratio':        (0.010, 0.20),
    'target_stop_distance':       (0.40, 1.20),
    'position_linear_kp':         (0.20, 1.50),
    'position_angular_kp':        (0.80, 3.50),
    # Re-deteccao
    'redetect_spin_speed':        (0.30, 1.50),
}
