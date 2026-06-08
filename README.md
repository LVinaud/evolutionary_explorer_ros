# evolutionary_explorer_ros

**SSC0712 – Programação de Robôs Móveis · Trabalho 1**
Sistema de **Exploração, Navegação e Controle da Missão** com ROS 2.

Um robô diferencial autônomo **explora** a arena, **detecta a bandeira inimiga**
por visão computacional (câmera de segmentação semântica), **navega** até ela
**desviando de obstáculos** com o LIDAR e **se posiciona** de frente para a
bandeira para a coleta. Todo o comportamento é coordenado por uma
**máquina de estados**.

> Este pacote deriva do pacote-base da disciplina
> (`matheusbg8/prm_2026`): o robô, os sensores, os mundos e a infraestrutura de
> *launch* foram reaproveitados e adaptados. **O pacote foi renomeado** para
> `evolutionary_explorer_ros`, o robô para `explorer_robot`, e foram adicionados
> os nós de percepção e de controle da missão.

> **Por que "evolutionary"?** A arquitetura foi projetada para uma **segunda
> fase** em que os parâmetros de comportamento serão **evoluídos por computação
> evolutiva**. Toda a lógica (máquina de estados) é independente dos *valores*
> (ganhos/limiares), centralizados em [`config/mission_params.yaml`](config/mission_params.yaml)
> e na dataclass [`MissionParams`](evolutionary_explorer_ros/mission_params.py).
> Veja a seção [Preparação para computação evolutiva](#preparação-para-computação-evolutiva-fase-2).

---

## 1. Requisitos

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Fortress** (Ignition) + `ros_gz` / `ign_ros2_control`
- Python 3, OpenCV (`python3-opencv`), `cv_bridge`, `scipy`, `numpy`

Instalação das dependências do sistema (caso ainda não tenha o ROS/Gazebo,
veja o script auxiliar [`scripts/install_ros_humble.sh`](scripts/install_ros_humble.sh)):

```bash
# A partir da raiz do workspace, com o ROS já instalado:
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 2. Compilação

```bash
# Coloque este pacote em ~/ros2_ws/src/evolutionary_explorer_ros
cd ~/ros2_ws
colcon build --symlink-install --packages-select evolutionary_explorer_ros
source install/setup.bash
```

## 3. Execução

São necessários **dois terminais** (em ambos, lembre de `source install/setup.bash`).

**Terminal 1 — inicia o mundo no Gazebo:**

```bash
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py
# (opcional) outro mundo:
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py world:=arena_paredes.sdf
```

**Terminal 2 — carrega o robô + controle autônomo da missão:**

```bash
ros2 launch evolutionary_explorer_ros missao.launch.py
```

Isso sobe o robô, os sensores, a ponte Gazebo↔ROS, o RViz, a odometria
*ground-truth*, o `flag_detector` (percepção) e o `mission_control`
(máquina de estados). O robô começa a explorar imediatamente.

#### Argumentos úteis de launch

```bash
# Gazebo sem GUI (headless) — útil p/ testes e p/ episódios da fase evolutiva:
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py headless:=true

# Missão sem RViz e com posição inicial do robô customizada:
ros2 launch evolutionary_explorer_ros missao.launch.py use_rviz:=false \
    spawn_x:=-8.0 spawn_y:=-0.5 spawn_yaw:=0.0

# Usar outro arquivo de parâmetros (ex.: um genoma evoluído na fase 2):
ros2 launch evolutionary_explorer_ros missao.launch.py params_file:=/caminho/genoma.yaml
```

> **Validação:** em simulação (Gazebo Fortress, Humble) o sistema foi executado
> de ponta a ponta, completando a missão (EXPLORANDO → … → POSICIONANDO →
> MISSAO_CONCLUIDA), inclusive o caminho de recuperação REDETECTANDO_BANDEIRA
> quando a bandeira sai do campo de visão.

### Controle manual (alternativa, sem o controle autônomo)

```bash
# Terminal 1: inicia_simulacao.launch.py
# Terminal 2: carrega_robo.launch.py   (só robô + sensores, sem a missão)
# Terminal 3:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Visualizar só o URDF (RViz, sem física)

```bash
ros2 launch evolutionary_explorer_ros teste_urdf.launch.py
```

---

## 4. Arquitetura geral do sistema

```
                 +------------------------+        /scan (LaserScan)
   Gazebo  ----> |     ros_gz_bridge      | -----> /imu  (Imu)
  Fortress       | (carrega_robo.launch)  | -----> /robot_cam/labels_map (Image)
                 +------------------------+ -----> /model/explorer_robot/pose (Pose)
                      |            |
                      v            v
        +--------------------+   +----------------------------+
        |   flag_detector    |   |   ground_truth_odometry    |
        | (camera segment.)  |   |  (/odom_gt + TF odom->base)|
        +---------+----------+   +----------------------------+
                  | /flag/detected (Bool)
                  | /flag/offset   (Float32 [-1,1])
                  | /flag/area_ratio (Float32)
                  v
        +-------------------------------+
        |        mission_control        |  <-- /scan (desvio de obstáculos)
        |   MÁQUINA DE ESTADOS (FSM)    |
        +---------------+---------------+
                        | /cmd_vel (Twist)
                        v
              diff_drive_controller  (ros2_control / Gazebo)
```

### Nós

| Nó | Arquivo | Função |
|----|---------|--------|
| `mission_control` | [`mission_control.py`](evolutionary_explorer_ros/mission_control.py) | **Cérebro**: máquina de estados, navegação reativa, desvio e posicionamento. Publica `/cmd_vel`. |
| `flag_detector` | [`flag_detector.py`](evolutionary_explorer_ros/flag_detector.py) | **Percepção**: detecta a bandeira inimiga na câmera de segmentação e publica detecção/offset/área. |
| `ground_truth_odometry` | [`ground_truth_odometry.py`](evolutionary_explorer_ros/ground_truth_odometry.py) | Publica `/odom_gt` e a TF `odom_gt → base_link` a partir da pose do simulador. |
| `robo_mapper` | [`robo_mapper.py`](evolutionary_explorer_ros/robo_mapper.py) | Mapa de ocupação simples (`/grid_map`) — opcional. |

### Tópicos principais

| Tópico | Tipo | Sentido |
|--------|------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | mission_control → robô |
| `/scan` | `sensor_msgs/LaserScan` | LIDAR → mission_control |
| `/imu` | `sensor_msgs/Imu` | IMU → mission_control |
| `/robot_cam/labels_map` | `sensor_msgs/Image` | câmera segmentação → flag_detector |
| `/robot_cam/colored_map` | `sensor_msgs/Image` | câmera segmentação (modo cor) |
| `/flag/detected` | `std_msgs/Bool` | flag_detector → mission_control |
| `/flag/offset` | `std_msgs/Float32` | desloc. horizontal normalizado `[-1,1]` |
| `/flag/area_ratio` | `std_msgs/Float32` | área do blob / área da imagem |
| `/flag/debug_image` | `sensor_msgs/Image` | detecção desenhada (depuração) |
| `/mission/state` | `std_msgs/String` | estado atual da FSM (monitoramento) |
| `/mission/start` | `std_msgs/Bool` | inicia a missão (se `start_immediately:=false`) |

---

## 5. Máquina de estados (módulo central)

Implementada em [`mission_control.py`](evolutionary_explorer_ros/mission_control.py),
com cada estado documentado no respectivo *handler*.

```
 AGUARDANDO_COMANDO ──start──▶ EXPLORANDO ──bandeira confirmada──▶ BANDEIRA_DETECTADA
                                  ▲   ▲                                    │
                       timeout ───┘   │ bandeira perdida                   ▼
                  REDETECTANDO_BANDEIRA ◀──────────── NAVEGANDO_PARA_BANDEIRA
                                  │                            │ perto + centralizada
                                  │ reencontrou                ▼
                                  └──────────────▶  POSICIONANDO_PARA_COLETA
                                                              │ alinhado + distância ok
                                                              ▼
                                                       MISSAO_CONCLUIDA
```

| Estado | Comportamento | Saída |
|--------|---------------|-------|
| **AGUARDANDO_COMANDO** | Parado até o *start*. | `start` → EXPLORANDO |
| **EXPLORANDO** | Avança em **serpentina** (varredura) procurando a bandeira; **desvia** de obstáculos com o LIDAR. | bandeira confirmada → BANDEIRA_DETECTADA |
| **BANDEIRA_DETECTADA** | Calcula o *bearing* da bandeira em relação à câmera (a partir do offset e do FOV). | → NAVEGANDO_PARA_BANDEIRA / (perdida) → REDETECTANDO |
| **NAVEGANDO_PARA_BANDEIRA** | Controle **P** sobre o offset para se alinhar e avançar; **desvio de obstáculos tem prioridade** (mas o que está à frente quando a bandeira está centralizada é a *própria bandeira*, não um obstáculo a evitar). | perto (LIDAR/área) + centralizada → POSICIONANDO / perdida → REDETECTANDO |
| **POSICIONANDO_PARA_COLETA** | Aproxima-se centralizando a bandeira até ela ficar **grande na imagem** (proximidade por visão — robusto, pois o mastro é fino no LIDAR); o LIDAR atua só como **segurança** anticolisão. | centralizado + bandeira grande (perto) → MISSAO_CONCLUIDA / perdida → REDETECTANDO |
| **REDETECTANDO_BANDEIRA** | Gira no lugar (lado em que a bandeira foi vista) para reencontrá-la. | reencontrou → NAVEGANDO / timeout → EXPLORANDO |
| **MISSAO_CONCLUIDA** | Para; breve giro de "comemoração". | — |

**Robustez:** a perda da bandeira (REDETECTANDO) e o desvio reativo de
obstáculos (prioridade sobre a perseguição) garantem reação correta a eventos,
conforme o critério de *coerência e robustez*.

---

## 6. Detecção visual da bandeira

A câmera é do tipo **segmentação semântica** (definida no URDF). No mundo
`arena_cilindros`, cada objeto recebe uma *label*:

| Objeto | label | Objeto | label |
|--------|-------|--------|-------|
| ground_plane | 5 | red_flag | **20** |
| red_base | 10 | blue_flag | **25** |
| blue_base | 15 | flag_deploy_zone | 28 |
| obstáculos/centro | 30 | paredes | 35 |

O robô nasce no lado **vermelho**, então a **bandeira inimiga é a AZUL
(label 25)** — esse é o `target_label` padrão do `flag_detector`.

O `flag_detector` opera em dois modos (parâmetro `detection_mode`):

- **`labels`** (padrão): lê `/robot_cam/labels_map`, onde o valor do pixel é o
  ID da *label*. Procura `pixel == target_label`. **Robusto** (não depende de
  calibrar cor).
- **`color`**: lê `/robot_cam/colored_map` e procura uma cor BGR exata
  (`target_color_*`) — abordagem das aulas. Use se preferir o mapa colorido,
  calibrando a cor da bandeira (ver abaixo).

Do maior *blob* encontrado calculamos o **centróide** → `offset` horizontal
normalizado e a **área** (proxy de proximidade).

### Calibração (se a detecção não funcionar de primeira)

```bash
# Veja as labels presentes na imagem (modo labels):
ros2 topic echo /robot_cam/labels_map --no-arr
# Veja a detecção desenhada:
ros2 run rqt_image_view rqt_image_view   # selecione /flag/debug_image
```

Ajuste `target_label` (ou, no modo `color`, `target_color_b/g/r`) em
[`config/mission_params.yaml`](config/mission_params.yaml).

---

## 7. Navegação e desvio de obstáculos

- O LIDAR (`/scan`, 360 amostras, alcance 0,12–3,5 m) é **setorizado**
  (frente / frente-esq / frente-dir / laterais) em
  [`navigation.py`](evolutionary_explorer_ros/navigation.py).
- Se a distância frontal cai abaixo de `obstacle_block_distance`, o robô
  **desvia** virando para o lado mais livre; abaixo de
  `emergency_stop_distance`, ele **gira parado** (segurança).
- O desvio é aplicado **tanto explorando quanto navegando** até a bandeira.
  Durante a navegação, quando a bandeira está centralizada, o obstáculo à frente
  é a própria bandeira — então o robô prossegue (em vez de desviar dela).
- O **posicionamento final** se aproxima até a bandeira ficar grande na imagem
  (`complete_area_ratio`), centralizando-a (`centering_tolerance`); o LIDAR
  (`emergency_stop_distance`) é usado apenas como segurança anticolisão. Isso é
  mais robusto que medir a distância do mastro fino com o LIDAR.

---

## 8. Preparação para computação evolutiva (fase 2)

A arquitetura separa **comportamento** (máquina de estados, em
`mission_control.py`) de **parâmetros** (ganhos/limiares, em `MissionParams`).
Isso é o ponto de injeção da evolução:

- [`mission_params.py`](evolutionary_explorer_ros/mission_params.py) define a
  dataclass **`MissionParams`** com todos os genes e:
  - `to_genome()` / `from_genome(vetor)` — (de)serialização do **cromossomo**;
  - `genome_keys()` / `bounds()` — ordem e **limites** de cada gene;
  - `declare_and_read(node)` — declara os parâmetros como **parâmetros ROS 2**,
    permitindo sobrescrevê-los por arquivo/CLI.
- O launch aceita `params_file:=...`, então um **driver evolutivo** poderá, a
  cada indivíduo, gerar um YAML de genoma e rodar um episódio:

  ```bash
  ros2 launch evolutionary_explorer_ros missao.launch.py params_file:=/tmp/individuo_42.yaml
  ```

- `navigation.py` é **livre de ROS**, podendo ser reutilizado num avaliador de
  *fitness* headless.
- `/mission/state` e `/odom_gt` servem como sinais para a **função de fitness**
  (ex.: tempo até a bandeira, distância percorrida, colisões, cobertura).

Assim, **nenhuma mudança na máquina de estados** será necessária para evoluir o
comportamento — apenas os valores dos parâmetros mudam.

---

## 9. Estrutura do pacote

```
evolutionary_explorer_ros/
├── package.xml / setup.py / setup.cfg / resource/   # pacote ament_python
├── config/
│   ├── mission_params.yaml        # parâmetros (genoma) da missão
│   └── controller_config.yaml     # diff_drive + gripper (ros2_control)
├── description/robot.urdf.xacro    # robô (câmera segm., LIDAR, IMU, +farol)
├── launch/
│   ├── inicia_simulacao.launch.py  # mundo no Gazebo
│   ├── carrega_robo.launch.py      # robô + sensores + bridge + rviz
│   ├── missao.launch.py            # robô + controle autônomo (completo)
│   └── teste_urdf.launch.py        # só URDF no RViz
├── world/ , models/                # arenas e obstáculos
├── rviz/                           # configs do RViz
└── evolutionary_explorer_ros/
    ├── mission_control.py          # MÁQUINA DE ESTADOS (cérebro)
    ├── flag_detector.py            # percepção da bandeira
    ├── navigation.py               # utilidades de LIDAR/navegação (sem ROS)
    ├── mission_params.py           # parâmetros + suporte a genoma
    ├── ground_truth_odometry.py    # odometria ground-truth + TF
    └── robo_mapper.py              # mapa de ocupação (opcional)
```

## 10. Documentação da feira (pôster/slides)

> _Adicionar aqui o link para o pôster/slides da apresentação._

---

## Créditos

Baseado no pacote da disciplina SSC0712 (Prof. Dr. Matheus Machado dos Santos),
`https://github.com/matheusbg8/prm_2026`. Licença Apache-2.0.
