#!/usr/bin/env python3
"""Nodo CEREBRO da missao: maquina de estados de exploracao/navegacao.

Este e o modulo central do Trabalho 1. Ele coordena toda a missao por meio de
uma MAQUINA DE ESTADOS explicita e publica comandos de velocidade em /cmd_vel.

Fluxo dos estados
-----------------
        +----------------------+
        |  AGUARDANDO_COMANDO   |  (so se start_immediately == False)
        +----------+-----------+
                   | start
                   v
        +----------------------+   bandeira confirmada
        |      EXPLORANDO      +-----------------------+
        |  (varredura + desvio)|                       |
        +----------+-----------+                       v
                   ^                        +----------------------+
                   | timeout de redeteccao  |  BANDEIRA_DETECTADA  |
                   |                        | (calcula pose na cam)|
        +----------+-----------+            +----------+-----------+
        | REDETECTANDO_BANDEIRA|                       |
        | (gira procurando)    |                       v
        +----------+-----------+            +----------------------+
                   ^   bandeira perdida      | NAVEGANDO_PARA_      |
                   +-------------------------+   BANDEIRA           |
                   |                         | (vai + desvia LIDAR) |
                   |                         +----------+-----------+
                   |  bandeira perdida                  | perto e centralizada
                   +------------------------------------+
                                                        v
                                            +----------------------+
                                            | POSICIONANDO_PARA_   |
                                            |   COLETA             |
                                            +----------+-----------+
                                                       | alinhado + distancia ok
                                                       v
                                            +----------------------+
                                            |  MISSAO_CONCLUIDA    |
                                            +----------------------+

Percepcao consumida:
  * /scan            (sensor_msgs/LaserScan) -> desvio de obstaculos
  * /flag/detected   (std_msgs/Bool)         -> bandeira no campo de visao
  * /flag/offset     (std_msgs/Float32)      -> desloc. horizontal [-1,1]
  * /flag/area_ratio (std_msgs/Float32)      -> proximidade (area do blob)
  * /imu             (sensor_msgs/Imu)       -> orientacao (yaw), informativo

Acao:
  * /cmd_vel         (geometry_msgs/Twist)   -> velocidade do robo diferencial

Todos os ganhos/limiares vem de MissionParams (mission_params.py), carregados
de config/mission_params.yaml. Isso mantem a LOGICA (estados) separada dos
VALORES (parametros), permitindo que a fase 2 (computacao evolutiva) evolua o
comportamento apenas trocando os parametros, sem tocar nesta maquina de estados.
"""

import math
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R

from .mission_params import MissionParams
from . import navigation as nav


# Campo de visao horizontal da camera (rad), conforme o URDF (horizontal_fov).
CAMERA_H_FOV = 1.57


class State(Enum):
    """Estados da missao (documentados individualmente nos handlers)."""
    AGUARDANDO_COMANDO = 'AGUARDANDO_COMANDO'
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    NAVEGANDO_PARA_BANDEIRA = 'NAVEGANDO_PARA_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    REDETECTANDO_BANDEIRA = 'REDETECTANDO_BANDEIRA'
    MISSAO_CONCLUIDA = 'MISSAO_CONCLUIDA'


class MissionControl(Node):

    def __init__(self):
        super().__init__('mission_control')

        # ---- Parametros (genoma) ----
        self.p = MissionParams.declare_and_read(self)

        # ---- Publishers ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Estado atual publicado para depuracao/monitoramento (e util como
        # "sensor" de fitness na fase evolutiva).
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        # ---- Subscribers ----
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/flag/detected', self.flag_detected_cb, 10)
        self.create_subscription(Float32, '/flag/offset', self.flag_offset_cb, 10)
        self.create_subscription(Float32, '/flag/area_ratio', self.flag_area_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        # Odometria (ground-truth) para detectar progresso/travamento.
        self.create_subscription(Odometry, '/odom_gt', self.odom_cb, 10)
        # Comando de start opcional (so usado se start_immediately == False)
        self.create_subscription(Bool, '/mission/start', self.start_cb, 10)

        # ---- Estado da percepcao ----
        self.scan = None                  # ultima LaserScan
        self.flag_detected_raw = False    # ultimo /flag/detected
        self.flag_offset = 0.0            # ultimo /flag/offset
        self.flag_area = 0.0              # ultimo /flag/area_ratio
        self.yaw = 0.0                    # orientacao atual (do IMU)
        self.tipped = False               # robo tombado? (seguranca via IMU)
        self.x = 0.0                      # posicao (odometria)
        self.y = 0.0
        self.pos_history = deque()        # historico (t, x, y) p/ detectar travamento
        self.escape_until = 0.0           # tempo ate o qual executa manobra de escape
        self.escape_turn_dir = 1.0        # sentido do giro de escape
        self.detect_streak = 0            # frames consecutivos com deteccao
        self.last_seen_time = None        # instante da ultima deteccao positiva
        self.last_offset_sign = 1.0       # lado onde a bandeira foi vista

        # ---- Estado da maquina ----
        self.state = State.AGUARDANDO_COMANDO
        self.start_requested = bool(self.p.start_immediately)
        self.t0 = self.now()
        self.state_since = self.now()

        # ---- Loop de controle ----
        period = 1.0 / max(1.0, self.p.control_frequency)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            'mission_control iniciado. Estado inicial: '
            f'{self.state.value} (start_immediately={self.p.start_immediately})')

    # ====================================================================== #
    # Utilitarios
    # ====================================================================== #
    def now(self) -> float:
        """Tempo atual em segundos (relogio do ROS / simulado)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def transition_to(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(
                f'[FSM] {self.state.value} -> {new_state.value}')
            self.state = new_state
            self.state_since = self.now()

    def time_in_state(self) -> float:
        return self.now() - self.state_since

    @property
    def flag_visible(self) -> bool:
        """True se a bandeira foi confirmada por frames suficientes."""
        return self.detect_streak >= self.p.detect_confirm_frames

    def time_since_seen(self) -> float:
        if self.last_seen_time is None:
            return float('inf')
        return self.now() - self.last_seen_time

    # ====================================================================== #
    # Callbacks de percepcao
    # ====================================================================== #
    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def flag_detected_cb(self, msg: Bool):
        if msg.data:
            self.detect_streak += 1
            self.last_seen_time = self.now()
        else:
            self.detect_streak = 0
        self.flag_detected_raw = msg.data

    def flag_offset_cb(self, msg: Float32):
        self.flag_offset = msg.data
        if abs(msg.data) > 1e-3:
            self.last_offset_sign = 1.0 if msg.data > 0 else -1.0

    def flag_area_cb(self, msg: Float32):
        self.flag_area = msg.data

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        try:
            roll, pitch, yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            self.yaw = yaw
            # Robo considerado TOMBADO se muito inclinado (|roll| ou |pitch| > ~50 deg).
            self.tipped = abs(roll) > 0.9 or abs(pitch) > 0.9
        except ValueError:
            pass

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def _update_stuck_history(self):
        """Mantem um historico recente de posicoes (janela stuck_window)."""
        now = self.now()
        self.pos_history.append((now, self.x, self.y))
        while self.pos_history and now - self.pos_history[0][0] > self.p.stuck_window:
            self.pos_history.popleft()

    def _is_stuck(self) -> bool:
        """True se o robo ficou confinado a uma regiao pequena na janela
        (deslocamento total < stuck_min_progress) -> preso num loop/beco."""
        if len(self.pos_history) < 2:
            return False
        if self.now() - self.pos_history[0][0] < self.p.stuck_window:
            return False
        xs = [p[1] for p in self.pos_history]
        ys = [p[2] for p in self.pos_history]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return spread < self.p.stuck_min_progress

    def start_cb(self, msg: Bool):
        if msg.data:
            self.start_requested = True

    # ====================================================================== #
    # Sensoriamento de obstaculos (LIDAR)
    # ====================================================================== #
    def read_sectors(self) -> nav.ScanSectors:
        """Setoriza o ultimo LaserScan usando os parametros atuais."""
        if self.scan is None:
            # Sem dados ainda -> considera tudo livre.
            big = 99.0
            return nav.ScanSectors(big, big, big, big, big)
        s = self.scan
        return nav.compute_sectors(
            s.ranges, s.angle_min, s.angle_increment,
            front_half_rad=math.radians(self.p.front_sector_half_angle_deg),
            side_half_rad=math.radians(self.p.side_sector_half_angle_deg),
            range_min=s.range_min, range_max=s.range_max)

    def front_distance(self) -> float:
        """Distancia frontal (setor estreito) usada no posicionamento."""
        if self.scan is None:
            return 99.0
        s = self.scan
        return nav.sector_min_distance(
            s.ranges, s.angle_min, s.angle_increment,
            center_rad=0.0, half_width_rad=math.radians(12.0),
            range_min=s.range_min, range_max=s.range_max)

    def is_blocked(self, sectors: nav.ScanSectors) -> bool:
        """Ha obstaculo proximo a frente OU nas diagonais frontais?

        Considerar as diagonais (front_left/front_right) faz o robo reagir a
        obstaculos que ele clipa de lado, nao so os bem a frente."""
        d = self.p.obstacle_block_distance
        return (sectors.front <= d or
                sectors.front_left <= d * 0.85 or
                sectors.front_right <= d * 0.85)

    def avoidance_twist(self, sectors: nav.ScanSectors) -> Twist:
        """Desvio: GIRA NO LUGAR (sem avancar) ate o caminho liberar.

        Em vez de virar avancando -- o que faz o robo clipar/subir no obstaculo
        e tombar -- ele para e gira para o lado mais aberto. So volta a avancar
        quando a frente estiver livre (ver st_explorando/st_navegando)."""
        twist = Twist()
        turn_dir = 1.0 if sectors.front_blocked_side == 'left' else -1.0
        twist.angular.z = turn_dir * self.p.avoid_angular_speed
        twist.linear.x = 0.0  # nao avanca contra o obstaculo
        return twist

    # ====================================================================== #
    # Loop principal: despacha o handler do estado atual
    # ====================================================================== #
    def control_loop(self):
        # Seguranca: se o robo tombou, para de comandar velocidade (evita ficar
        # "arando" o chao com as rodas e mascarar o problema).
        if self.tipped:
            self.get_logger().warn(
                'Robo TOMBADO (inclinacao excessiva) - parando comandos.',
                throttle_duration_sec=3.0)
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data=self.state.value))
            return

        handlers = {
            State.AGUARDANDO_COMANDO: self.st_aguardando,
            State.EXPLORANDO: self.st_explorando,
            State.BANDEIRA_DETECTADA: self.st_bandeira_detectada,
            State.NAVEGANDO_PARA_BANDEIRA: self.st_navegando,
            State.POSICIONANDO_PARA_COLETA: self.st_posicionando,
            State.REDETECTANDO_BANDEIRA: self.st_redetectando,
            State.MISSAO_CONCLUIDA: self.st_concluida,
        }
        twist = handlers[self.state]()
        self.cmd_pub.publish(twist)
        self.state_pub.publish(String(data=self.state.value))

    # ---------------------------------------------------------------------- #
    # ESTADO: AGUARDANDO_COMANDO
    # Robo parado ate receber o start (via /mission/start ou start_immediately).
    # ---------------------------------------------------------------------- #
    def st_aguardando(self) -> Twist:
        if self.start_requested:
            self.transition_to(State.EXPLORANDO)
        return Twist()  # parado

    # ---------------------------------------------------------------------- #
    # ESTADO: EXPLORANDO
    # Varre a arena (avanco + serpentina) procurando a bandeira, desviando de
    # obstaculos com o LIDAR. Sai quando a bandeira e confirmada.
    # ---------------------------------------------------------------------- #
    def st_explorando(self) -> Twist:
        if self.flag_visible:
            self.transition_to(State.BANDEIRA_DETECTADA)
            return Twist()

        now = self.now()
        self._update_stuck_history()

        # (a) Manobra de escape em andamento: recua COM FIRMEZA (p/ se soltar de
        #     um cilindro em que tenha encravado) e gira para o lado aberto.
        if now < self.escape_until:
            twist = Twist()
            twist.linear.x = -0.22
            twist.angular.z = self.escape_turn_dir * self.p.escape_turn_speed
            return twist

        # (b) Detecta travamento -> inicia escape girando p/ o lado mais aberto.
        if self._is_stuck():
            sectors = self.read_sectors()
            self.escape_turn_dir = 1.0 if sectors.left >= sectors.right else -1.0
            self.escape_until = now + self.p.escape_duration
            self.pos_history.clear()
            self.get_logger().info('Exploracao travada -> manobra de escape.')
            return Twist()

        # (c) Direcao por CAMPO POTENCIAL: o robo e ATRAIDO para o rumo alvo
        #     (+x, lado inimigo) e REPELIDO dos obstaculos ao mesmo tempo, de
        #     forma que CONTORNA os cilindros suavemente enquanto avanca (em vez
        #     de empurra-los). Sempre se move (mais devagar perto de obstaculo;
        #     so gira parado se em distancia critica).
        sectors = self.read_sectors()
        react = self.p.obstacle_block_distance

        def prox(d):
            # 0 (longe) .. 1 (encostando): intensidade da repulsao
            return max(0.0, (react - d) / react) if d < react else 0.0

        # Atracao para o rumo alvo (+x)
        heading_error = nav.wrap_to_pi(self.p.explore_target_heading - self.yaw)
        attract = self.p.explore_heading_kp * heading_error
        # Repulsao lateral: obstaculo a frente-esq -> vira p/ direita (e vice-versa)
        repulse = self.p.repulse_gain * (prox(sectors.front_right) -
                                         prox(sectors.front_left))
        # Obstaculo bem a frente: reforca o giro para o lado MAIS ABERTO
        if sectors.front < react:
            side = 1.0 if sectors.front_left >= sectors.front_right else -1.0
            repulse += side * self.p.repulse_gain * prox(sectors.front)
        # Serpentina leve so com caminho livre (cobertura)
        serp = 0.0
        if not self.is_blocked(sectors):
            phase = 2.0 * math.pi * (now - self.t0) / \
                max(0.1, self.p.explore_serpentine_period)
            serp = self.p.explore_serpentine_gain * math.sin(phase)

        twist = Twist()
        twist.angular.z = nav.clamp(attract + repulse + serp,
                                    -self.p.nav_max_angular_speed,
                                    self.p.nav_max_angular_speed)
        # Velocidade: reduz perto de obstaculo; zera (so gira) se critico
        front_min = min(sectors.front, sectors.front_left, sectors.front_right)
        if front_min <= self.p.emergency_stop_distance:
            twist.linear.x = 0.0
        else:
            scale = (front_min - self.p.emergency_stop_distance) / \
                max(0.01, react - self.p.emergency_stop_distance)
            twist.linear.x = self.p.explore_linear_speed * nav.clamp(scale, 0.25, 1.0)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: BANDEIRA_DETECTADA
    # Bandeira identificada visualmente. Calcula a posicao (bearing) da bandeira
    # em relacao a camera e segue para a navegacao. Estado curto/transicional.
    # ---------------------------------------------------------------------- #
    def st_bandeira_detectada(self) -> Twist:
        if not self.flag_visible:
            self.transition_to(State.REDETECTANDO_BANDEIRA)
            return Twist()

        bearing_deg = math.degrees(self.flag_offset * (CAMERA_H_FOV / 2.0))
        self.get_logger().info(
            f'Bandeira detectada! bearing~{bearing_deg:+.1f} deg, '
            f'offset={self.flag_offset:+.2f}, area={self.flag_area:.3f}')
        self.transition_to(State.NAVEGANDO_PARA_BANDEIRA)
        return Twist()  # breve parada antes de navegar

    # ---------------------------------------------------------------------- #
    # ESTADO: NAVEGANDO_PARA_BANDEIRA
    # Vai em direcao a bandeira (controle P sobre o offset), mas o desvio de
    # obstaculos (LIDAR) tem PRIORIDADE sobre a perseguicao. Sai para
    # posicionamento quando a bandeira esta perto (area) e ~centralizada.
    # ---------------------------------------------------------------------- #
    def st_navegando(self) -> Twist:
        if self.time_since_seen() > self.p.detect_lost_timeout:
            self.transition_to(State.REDETECTANDO_BANDEIRA)
            return Twist()

        front = self.front_distance()
        centered = abs(self.flag_offset) < 0.30

        # Transicao para posicionamento: a bandeira (centralizada) ja esta perto.
        # Usamos a distancia frontal do LIDAR como gatilho principal (a bandeira
        # aparece pequena na imagem) e a area como gatilho secundario.
        if centered and (front <= self.p.approach_distance or
                         self.flag_area >= self.p.approach_area_ratio):
            self.transition_to(State.POSICIONANDO_PARA_COLETA)
            return Twist()

        sectors = self.read_sectors()
        # Desvio de obstaculos: SO desvia se o obstaculo NAO for a bandeira
        # (isto e, a bandeira nao esta centralizada). Se a bandeira esta bem a
        # frente, o "obstaculo" e ela mesma -> seguimos em direcao a ela (o
        # posicionamento cuida de parar na distancia certa).
        if self.is_blocked(sectors) and not centered:
            return self.avoidance_twist(sectors)

        # Caminho livre (ou bandeira a frente): persegue a bandeira.
        twist = Twist()
        twist.linear.x = self.p.nav_linear_speed
        # Reduz a velocidade ao se aproximar para nao "atropelar" a bandeira.
        if front <= self.p.obstacle_block_distance:
            twist.linear.x = min(twist.linear.x, self.p.avoid_linear_speed * 2.0)
        twist.angular.z = nav.clamp(
            -self.p.nav_angular_kp * self.flag_offset,
            -self.p.nav_max_angular_speed, self.p.nav_max_angular_speed)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: POSICIONANDO_PARA_COLETA
    # Ajuste fino: centraliza a bandeira (offset~0) e para na distancia
    # desejada (medida pelo LIDAR frontal). Conclui a missao quando alinhado.
    # ---------------------------------------------------------------------- #
    def st_posicionando(self) -> Twist:
        if self.time_since_seen() > self.p.detect_lost_timeout:
            self.transition_to(State.REDETECTANDO_BANDEIRA)
            return Twist()

        centered = abs(self.flag_offset) < self.p.centering_tolerance
        # "Perto o suficiente" = bandeira grande na imagem (sinal visual robusto;
        # o mastro fino e pouco confiavel no LIDAR a distancia).
        flag_close = self.flag_area >= self.p.complete_area_ratio
        front = self.front_distance()

        if centered and flag_close:
            self.get_logger().info(
                f'Posicionado! area={self.flag_area:.3f}, '
                f'offset={self.flag_offset:+.3f}, dist_frontal={front:.2f} m. '
                'Missao concluida.')
            self.transition_to(State.MISSAO_CONCLUIDA)
            return Twist()

        twist = Twist()
        # Centralizacao fina da bandeira.
        twist.angular.z = nav.clamp(
            -self.p.position_angular_kp * self.flag_offset,
            -self.p.nav_max_angular_speed, self.p.nav_max_angular_speed)
        # Aproxima-se enquanto a bandeira nao esta perto, com seguranca do LIDAR
        # (nao avanca se houver algo a distancia critica). Avanca menos quando
        # ainda nao esta centralizada (alinha primeiro, depois avanca).
        if not flag_close and front > self.p.emergency_stop_distance:
            approach = self.p.position_linear_kp * 0.4
            approach *= max(0.0, 1.0 - abs(self.flag_offset) / 0.5)
            twist.linear.x = nav.clamp(approach, 0.0, self.p.nav_linear_speed)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: REDETECTANDO_BANDEIRA
    # Perdeu a bandeira do campo de visao. Gira no lugar (no sentido em que ela
    # foi vista por ultimo) para reencontra-la. Se nao achar em redetect_timeout,
    # volta a explorar.
    # ---------------------------------------------------------------------- #
    def st_redetectando(self) -> Twist:
        if self.flag_visible:
            self.transition_to(State.NAVEGANDO_PARA_BANDEIRA)
            return Twist()
        if self.time_in_state() > self.p.redetect_timeout:
            self.transition_to(State.EXPLORANDO)
            return Twist()

        twist = Twist()
        twist.angular.z = self.last_offset_sign * self.p.redetect_spin_speed
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: MISSAO_CONCLUIDA
    # Chegou a bandeira inimiga e se posicionou. Para o robo. Faz um breve giro
    # de "comemoracao" (criatividade) nos primeiros segundos.
    # ---------------------------------------------------------------------- #
    def st_concluida(self) -> Twist:
        twist = Twist()
        if self.time_in_state() < 3.0:
            twist.angular.z = 1.5  # comemoracao
        return twist


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante que o robo pare ao encerrar o nodo.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
