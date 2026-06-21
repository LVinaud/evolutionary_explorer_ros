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
from std_msgs.msg import Bool, Float32, String, Float64MultiArray
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R

from .mission_params import MissionParams
from . import navigation as nav
from .planner import OccupancyPlanner


# Campo de visao horizontal da camera (rad), conforme o URDF (horizontal_fov).
CAMERA_H_FOV = 1.57


class State(Enum):
    """Estados da missao (documentados individualmente nos handlers)."""
    AGUARDANDO_COMANDO = 'AGUARDANDO_COMANDO'
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    NAVEGANDO_PARA_BANDEIRA = 'NAVEGANDO_PARA_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    CAPTURANDO_BANDEIRA = 'CAPTURANDO_BANDEIRA'
    RETORNANDO_PARA_BASE = 'RETORNANDO_PARA_BASE'
    POSICIONANDO_PARA_DEPOSITO = 'POSICIONANDO_PARA_DEPOSITO'
    DEPOSITANDO_BANDEIRA = 'DEPOSITANDO_BANDEIRA'
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
        # Comandos da garra (Trabalho 2): vetor [elevacao, direito, esquerdo],
        # na ordem do controlador do professor.
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

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
        self.have_odom = False            # ja recebeu a primeira pose?
        # Pose inicial (centro da base vermelha): referencia do deposito (T2).
        self.start_x = None
        self.start_y = None
        # True enquanto a bandeira esta presa na garra (transporte): faz o robo
        # ignorar a propria bandeira nas leituras do LIDAR.
        self.carrying = False
        self.pos_history = deque()        # historico (t, x, y) p/ detectar travamento
        self.escape_until = 0.0           # tempo ate o qual executa manobra de escape
        self.escape_turn_dir = 1.0        # sentido do giro de escape

        # ---- Mapa de ocupacao (construido pelo LIDAR) + planejador A estrela ----
        self.gplanner = OccupancyPlanner(resolution=self.p.plan_resolution,
                                         robot_radius=self.p.plan_robot_radius)
        self.path = []                    # rota atual (lista de waypoints)
        self.path_goal = None             # objetivo da rota atual
        self.last_plan_time = 0.0
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

    def send_gripper(self, elev: float, right: float, left: float):
        """Publica um comando para a garra na ordem do controlador do professor:
        [elevacao_haste, braco_direito, braco_esquerdo]."""
        msg = Float64MultiArray()
        msg.data = [float(elev), float(right), float(left)]
        self.gripper_pub.publish(msg)

    def open_gripper(self, elev: float):
        """Abre os dois bracos, mantendo a haste na elevacao dada."""
        self.send_gripper(elev, self.p.grip_open_right, self.p.grip_open_left)

    def close_gripper(self, elev: float):
        """Fecha os dois bracos (prende o mastro), na elevacao dada."""
        self.send_gripper(elev, self.p.grip_closed_right, self.p.grip_closed_left)

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
        # Constroi a grade de ocupacao a partir do LIDAR (so com pose valida).
        # Ignora retornos muito proximos para nao mapear o proprio robo/garra.
        # Ao carregar a bandeira, ignora ainda mais perto para nao mapear o
        # proprio mastro preso a frente como se fosse um obstaculo.
        if self.have_odom:
            near = self.p.carry_blind_range if self.carrying else 0.35
            self.gplanner.update_from_scan(
                self.x, self.y, self.yaw, msg.ranges,
                msg.angle_min, msg.angle_increment,
                max(msg.range_min, near), msg.range_max)

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
            roll, pitch, _ = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            # Robo considerado TOMBADO se muito inclinado (|roll| ou |pitch| > ~50 deg).
            self.tipped = abs(roll) > 0.9 or abs(pitch) > 0.9
        except ValueError:
            pass

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        try:
            self.yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        except ValueError:
            pass
        # Guarda a primeira pose como referencia da base (centro do circulo de
        # deposito), conforme permite o enunciado do Trabalho 2.
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
        self.have_odom = True

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

    def _maybe_escape(self):
        """Manobra de escape global p/ tirar o robo de travamentos (minimo
        local do campo potencial ou encravamento em cilindro). Retorna o comando
        de escape (recua firme + gira p/ o lado mais aberto) enquanto a manobra
        dura, inicia uma nova manobra se acabou de travar, ou None se nao ha
        escape a fazer."""
        now = self.now()
        if now < self.escape_until:
            twist = Twist()
            twist.linear.x = -0.22
            twist.angular.z = self.escape_turn_dir * self.p.escape_turn_speed
            return twist
        if self._is_stuck():
            sectors = self.read_sectors()
            self.escape_turn_dir = 1.0 if sectors.left >= sectors.right else -1.0
            self.escape_until = now + self.p.escape_duration
            self.pos_history.clear()
            self.get_logger().info('Robo travado -> manobra de escape.')
            return Twist()
        return None

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

    def front_clear_distance(self) -> float:
        """Distancia frontal para a navegacao de retorno. Ao carregar a
        bandeira, ignora os retornos mais proximos que carry_blind_range, que
        sao o proprio mastro preso, para nao confundi-lo com uma parede."""
        if self.scan is None:
            return 99.0
        s = self.scan
        rmin = self.p.carry_blind_range if self.carrying else s.range_min
        return nav.sector_min_distance(
            s.ranges, s.angle_min, s.angle_increment,
            center_rad=0.0, half_width_rad=math.radians(12.0),
            range_min=rmin, range_max=s.range_max)

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
    # Navegacao: A estrela sobre a grade construida pelo LIDAR (+ fallback)
    # ====================================================================== #
    def _reactive_twist(self, direction: float) -> Twist:
        """Campo potencial reativo rumo a 'direction' (rad, mundo): atrai para a
        direcao e repele dos obstaculos. Usado como fallback quando A estrela
        nao encontra rota na grade atual."""
        sectors = self.read_sectors()
        react = self.p.obstacle_block_distance

        def prox(d):
            return max(0.0, (react - d) / react) if d < react else 0.0

        attract = self.p.explore_heading_kp * nav.wrap_to_pi(direction - self.yaw)
        repulse = self.p.repulse_gain * (prox(sectors.front_right) -
                                         prox(sectors.front_left))
        if sectors.front < react:
            side = 1.0 if sectors.front_left >= sectors.front_right else -1.0
            repulse += side * self.p.repulse_gain * prox(sectors.front)
        twist = Twist()
        twist.angular.z = nav.clamp(attract + repulse,
                                    -self.p.nav_max_angular_speed,
                                    self.p.nav_max_angular_speed)
        front_min = min(sectors.front, sectors.front_left, sectors.front_right)
        if front_min <= self.p.emergency_stop_distance:
            twist.linear.x = 0.0
        else:
            scale = (front_min - self.p.emergency_stop_distance) / \
                max(0.01, react - self.p.emergency_stop_distance)
            twist.linear.x = self.p.nav_linear_speed * nav.clamp(scale, 0.25, 1.0)
        return twist

    def _plan_follow_twist(self, direction: float) -> Twist:
        """Planeja por A estrela ate um ponto a 'lookahead' na direcao dada,
        sobre a grade construida pelo LIDAR, e segue o proximo waypoint. Como a
        grade vai sendo preenchida enquanto o robo anda, a rota e replanejada
        periodicamente. Se nao houver rota, cai no metodo reativo."""
        now = self.now()
        ld = self.p.lookahead_distance
        goal = (self.x + ld * math.cos(direction),
                self.y + ld * math.sin(direction))

        if not self.path or now - self.last_plan_time > self.p.replan_period:
            self.path = self.gplanner.plan((self.x, self.y), goal)
            self.path_goal = goal
            self.last_plan_time = now

        if not self.path:
            return self._reactive_twist(direction)

        # Descarta waypoints ja atingidos.
        while (len(self.path) > 1 and
               math.hypot(self.path[0][0] - self.x,
                          self.path[0][1] - self.y) < self.p.waypoint_tolerance):
            self.path.pop(0)
        wx, wy = self.path[0]
        yaw_err = nav.wrap_to_pi(math.atan2(wy - self.y, wx - self.x) - self.yaw)

        twist = Twist()
        twist.angular.z = nav.clamp(self.p.goto_angular_kp * yaw_err,
                                    -self.p.nav_max_angular_speed,
                                    self.p.nav_max_angular_speed)
        # Avanca, reduzindo se precisa girar muito; para em distancia critica.
        speed = self.p.nav_linear_speed * max(0.2, 1.0 - abs(yaw_err) / 1.2)
        if self.front_distance() <= self.p.emergency_stop_distance:
            speed = 0.0
        twist.linear.x = speed
        return twist

    def _plan_follow_to(self, goal) -> Twist:
        """Como _plan_follow_twist, mas planeja A estrela ate um PONTO absoluto
        (x, y) do mundo, e nao um ponto a frente numa direcao. Usado no retorno
        a base, cujo destino e a pose inicial gravada."""
        now = self.now()
        if not self.path or now - self.last_plan_time > self.p.replan_period:
            self.path = self.gplanner.plan((self.x, self.y), goal)
            self.path_goal = goal
            self.last_plan_time = now

        if not self.path:
            direction = math.atan2(goal[1] - self.y, goal[0] - self.x)
            return self._reactive_twist(direction)

        while (len(self.path) > 1 and
               math.hypot(self.path[0][0] - self.x,
                          self.path[0][1] - self.y) < self.p.waypoint_tolerance):
            self.path.pop(0)
        wx, wy = self.path[0]
        yaw_err = nav.wrap_to_pi(math.atan2(wy - self.y, wx - self.x) - self.yaw)

        twist = Twist()
        twist.angular.z = nav.clamp(self.p.goto_angular_kp * yaw_err,
                                    -self.p.nav_max_angular_speed,
                                    self.p.nav_max_angular_speed)
        # Se o objetivo esta muito fora do rumo (tipico no retorno, que exige
        # dar meia-volta), gira no lugar antes de avancar, em vez de sair
        # avancando para o lado errado.
        if abs(yaw_err) > self.p.turn_in_place_threshold:
            speed = 0.0
        else:
            speed = self.p.nav_linear_speed * max(0.2, 1.0 - abs(yaw_err) / 1.2)
            if self.front_clear_distance() <= self.p.emergency_stop_distance:
                speed = 0.0
        twist.linear.x = speed
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

        # Atualiza o historico de progresso e, nos estados de travessia
        # (explorando ou navegando ate a bandeira), aplica uma manobra de
        # escape se o robo travar. Isso tira o robo de minimos locais do campo
        # potencial e de encravamentos entre cilindros, valendo tanto na
        # exploracao quanto na navegacao ate a bandeira.
        self._update_stuck_history()
        # A manobra de escape (recuar + girar) NAO roda no retorno: com a
        # bandeira presa ela viraria res e giros erraticos. O retorno tem a
        # propria logica de girar no lugar e contornar pela grade.
        if self.state in (State.EXPLORANDO, State.NAVEGANDO_PARA_BANDEIRA):
            escape = self._maybe_escape()
            if escape is not None:
                self.cmd_pub.publish(escape)
                self.state_pub.publish(String(data=self.state.value))
                return

        handlers = {
            State.AGUARDANDO_COMANDO: self.st_aguardando,
            State.EXPLORANDO: self.st_explorando,
            State.BANDEIRA_DETECTADA: self.st_bandeira_detectada,
            State.NAVEGANDO_PARA_BANDEIRA: self.st_navegando,
            State.POSICIONANDO_PARA_COLETA: self.st_posicionando,
            State.CAPTURANDO_BANDEIRA: self.st_capturando,
            State.RETORNANDO_PARA_BASE: self.st_retornando,
            State.POSICIONANDO_PARA_DEPOSITO: self.st_posicionando_deposito,
            State.DEPOSITANDO_BANDEIRA: self.st_depositando,
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
    # Avanca rumo ao lado inimigo (+x) procurando a bandeira. Constroi a grade
    # de ocupacao pelo LIDAR e usa A estrela sobre ela para contornar os
    # obstaculos. Sai quando a bandeira e confirmada pela visao.
    # ---------------------------------------------------------------------- #
    def st_explorando(self) -> Twist:
        if self.flag_visible:
            self.transition_to(State.BANDEIRA_DETECTADA)
            return Twist()
        # Navega rumo ao lado inimigo planejando sobre a grade construida.
        return self._plan_follow_twist(self.p.explore_target_heading)

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
    # Vai em direcao a bandeira combinando atracao para ela (pelo deslocamento
    # na imagem) com repulsao dos obstaculos (LIDAR), de modo a contornar os
    # cilindros a caminho. Sai para posicionamento quando a bandeira esta perto
    # e ~centralizada.
    # ---------------------------------------------------------------------- #
    def st_navegando(self) -> Twist:
        if self.time_since_seen() > self.p.detect_lost_timeout:
            self.transition_to(State.REDETECTANDO_BANDEIRA)
            return Twist()

        # Transicao para posicionamento: a bandeira fica GRANDE na imagem, ou
        # seja, esta perto de verdade. Nao usamos a distancia do LIDAR como
        # gatilho porque um cilindro entre o robo e a bandeira a confundiria com
        # a bandeira e o robo pararia cedo demais.
        if (abs(self.flag_offset) < 0.5 and
                self.flag_area >= self.p.approach_area_ratio):
            self.transition_to(State.POSICIONANDO_PARA_COLETA)
            return Twist()

        # Navega ate a bandeira planejando A estrela sobre a grade construida
        # pelo LIDAR. A direcao alvo e a da bandeira VISTA PELA CAMERA, ou seja,
        # o rumo atual do robo somado ao deslocamento angular da bandeira na
        # imagem. Assim a visao direciona, e o planejador contorna os obstaculos
        # mapeados a caminho, sem prender nos cilindros laterais.
        flag_dir = self.yaw - self.flag_offset * (CAMERA_H_FOV / 2.0)
        return self._plan_follow_twist(flag_dir)

    # ---------------------------------------------------------------------- #
    # ESTADO: POSICIONANDO_PARA_COLETA
    # Alinha o robo com o mastro da bandeira e avanca reto ate ela ficar bem
    # perto (area grande na imagem), em posicao de pega. Usa "girar entao
    # avancar" com zona morta, evitando o vai-e-vem pendular. Quando alinhado e
    # perto o suficiente, passa para a CAPTURA.
    # ---------------------------------------------------------------------- #
    def st_posicionando(self) -> Twist:
        if self.time_since_seen() > self.p.detect_lost_timeout:
            self.transition_to(State.REDETECTANDO_BANDEIRA)
            return Twist()

        # Mira no MASTRO, nao no centro do blob: o painel desloca o centro do
        # blob para o seu lado, entao corrigimos com um vies lateral.
        aim_offset = self.flag_offset + self.p.grasp_aim_bias
        centered = abs(aim_offset) < self.p.centering_tolerance
        ready_to_grab = self.flag_area >= self.p.grasp_area_ratio
        front = self.front_distance()
        # Nao consegue mais avancar (algo, tipicamente a propria bandeira, esta
        # na distancia critica): tambem serve de gatilho para nao travar aqui.
        cant_advance = front <= self.p.emergency_stop_distance

        if centered and (ready_to_grab or cant_advance):
            self.get_logger().info(
                f'Em posicao de pega! area={self.flag_area:.3f}, '
                f'offset={self.flag_offset:+.3f} (mira={aim_offset:+.3f}), '
                f'dist_frontal={front:.2f} m. Iniciando captura.')
            self.transition_to(State.CAPTURANDO_BANDEIRA)
            return Twist()

        # Perseguicao suave: gira proporcional ao deslocamento (ja corrigido para
        # o mastro) E avanca ao mesmo tempo, reduzindo a velocidade quanto mais
        # descentralizado (em vez de "gira-entao-anda", que gera o vai-e-vem
        # pendular). Para o avanco se houver algo na distancia critica.
        twist = Twist()
        twist.angular.z = nav.clamp(
            -self.p.position_steer_kp * aim_offset,
            -self.p.nav_max_angular_speed, self.p.nav_max_angular_speed)
        if front > self.p.emergency_stop_distance:
            slow = max(0.3, 1.0 - abs(aim_offset) / 0.4)
            speed = self.p.position_linear_kp * slow
            twist.linear.x = nav.clamp(speed, 0.0, self.p.nav_linear_speed)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: CAPTURANDO_BANDEIRA
    # Sequencia temporizada da garra (por atrito): abaixa a haste na altura do
    # mastro e abre os bracos, avanca o pouco que falta para enfiar o mastro
    # entre os bracos, fecha os bracos prendendo o mastro e eleva a haste para
    # transportar. Robo so anda na etapa de avanco; nas demais fica parado.
    # ---------------------------------------------------------------------- #
    def st_capturando(self) -> Twist:
        p = self.p
        t = self.time_in_state()
        t_settle = p.capture_settle_time
        t_creep = t_settle + p.capture_creep_time
        t_close = t_creep + p.capture_close_time
        t_raise = t_close + p.capture_raise_time

        twist = Twist()
        if t < t_settle:
            # Abaixa a haste na altura do mastro e abre os bracos.
            self.open_gripper(p.grip_capture_elev)
        elif t < t_creep:
            # Avanca devagar para enfiar o mastro entre os bracos abertos.
            self.open_gripper(p.grip_capture_elev)
            twist.linear.x = p.capture_creep_speed
        elif t < t_close:
            # Fecha os bracos, prendendo o mastro por atrito.
            self.close_gripper(p.grip_capture_elev)
        elif t < t_raise:
            # Eleva a haste com a bandeira presa, pronto para transportar.
            self.close_gripper(p.grip_carry_elev)
        else:
            self.carrying = True  # a partir daqui o LIDAR deve ignorar o mastro
            self.get_logger().info('Bandeira capturada. Retornando a base.')
            self.transition_to(State.RETORNANDO_PARA_BASE)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: RETORNANDO_PARA_BASE
    # Mantem a bandeira presa e elevada e planeja por A estrela ate a pose
    # inicial (centro da base vermelha), contornando obstaculos pela grade do
    # LIDAR. Ao chegar perto da base, passa ao posicionamento de deposito.
    # ---------------------------------------------------------------------- #
    def st_retornando(self) -> Twist:
        # Mantem o mastro preso e elevado durante todo o transporte.
        self.close_gripper(self.p.grip_carry_elev)

        if self.start_x is None:
            return Twist()  # sem referencia de base ainda

        dist = math.hypot(self.start_x - self.x, self.start_y - self.y)
        if dist <= self.p.deposit_tolerance:
            self.transition_to(State.POSICIONANDO_PARA_DEPOSITO)
            return Twist()

        return self._plan_follow_to((self.start_x, self.start_y))

    # ---------------------------------------------------------------------- #
    # ESTADO: POSICIONANDO_PARA_DEPOSITO
    # Ajuste fino sobre o ponto de deposito (a pose inicial, dentro do circulo
    # amarelo). Aproxima-se do centro da base ate ficar bem dentro da tolerancia
    # e entao deposita.
    # ---------------------------------------------------------------------- #
    def st_posicionando_deposito(self) -> Twist:
        self.close_gripper(self.p.grip_carry_elev)
        if self.start_x is None:
            return Twist()

        dx = self.start_x - self.x
        dy = self.start_y - self.y
        dist = math.hypot(dx, dy)
        if dist <= self.p.deposit_tolerance * 0.6:
            self.get_logger().info(
                f'Sobre o ponto de deposito (a {dist:.2f} m do centro da base). '
                'Depositando a bandeira.')
            self.transition_to(State.DEPOSITANDO_BANDEIRA)
            return Twist()

        # Vira para o ponto de deposito e avanca ate o centro.
        yaw_err = nav.wrap_to_pi(math.atan2(dy, dx) - self.yaw)
        twist = Twist()
        twist.angular.z = nav.clamp(self.p.goto_angular_kp * yaw_err,
                                    -self.p.nav_max_angular_speed,
                                    self.p.nav_max_angular_speed)
        if abs(yaw_err) < 0.2:
            twist.linear.x = nav.clamp(self.p.position_linear_kp * 0.4,
                                       0.0, self.p.nav_linear_speed)
        return twist

    # ---------------------------------------------------------------------- #
    # ESTADO: DEPOSITANDO_BANDEIRA
    # Abre a garra para soltar a bandeira no circulo amarelo e recua um pouco
    # para nao reencostar nela nem derruba-la. Em seguida, missao concluida.
    # ---------------------------------------------------------------------- #
    def st_depositando(self) -> Twist:
        p = self.p
        t = self.time_in_state()
        t_release = p.deposit_release_time
        t_backup = t_release + p.deposit_backup_time

        twist = Twist()
        if t < t_release:
            # Abaixa a haste e abre os bracos para liberar a bandeira no chao.
            self.carrying = False  # soltou: volta a enxergar o LIDAR normalmente
            self.open_gripper(p.grip_capture_elev)
        elif t < t_backup:
            # Recua um pouco com a garra aberta para nao reencostar na bandeira.
            self.open_gripper(p.grip_capture_elev)
            twist.linear.x = -p.deposit_backup_speed
        else:
            self.get_logger().info('Bandeira depositada na base. Missao concluida.')
            self.transition_to(State.MISSAO_CONCLUIDA)
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
