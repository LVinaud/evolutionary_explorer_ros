# evolutionary_explorer_ros

## Visão geral

Este pacote ROS 2 implementa o sistema de controle de um robô móvel diferencial que explora um ambiente simulado, localiza a bandeira do time adversário por visão computacional, navega até ela desviando de obstáculos com o sensor LIDAR, captura a bandeira com o manipulador, transporta a bandeira de volta até a própria base e a deposita dentro do círculo amarelo demarcado. Todo o comportamento é coordenado por uma máquina de estados, que é o módulo central do trabalho. O projeto foi desenvolvido para a disciplina SSC0712 Programação de Robôs Móveis do ICMC USP e utiliza ROS 2 Humble com o simulador Gazebo Fortress.

Este repositório atende ao Trabalho 1, que ia até localizar e se posicionar diante da bandeira, e ao Trabalho 2, que acrescenta a captura com a garra, o retorno à base e o depósito. A entrega do Trabalho 2 está na branch trabalho-2.

O pacote deriva do pacote base da disciplina, disponível em github.com/matheusbg8/prm_2026. O robô, os sensores, os mundos e a infraestrutura de lançamento foram reaproveitados e adaptados. O pacote recebeu um nome próprio, o robô foi renomeado para explorer_robot e foram acrescentados os nós de percepção e de controle da missão, além de modificações de modelagem e estabilidade.

O nome do pacote contém a palavra evolutionary porque o projeto tem uma segunda fase planejada, na qual os parâmetros de comportamento do robô serão ajustados por computação evolutiva. A arquitetura já foi preparada para isso, mantendo toda a lógica de decisão separada dos valores numéricos de comportamento. Esses valores ficam reunidos em um único arquivo de parâmetros e em uma classe de dados dedicada, de modo que evoluir o comportamento significa apenas trocar valores, sem alterar a máquina de estados.

## Branches do repositório

Este repositório tem as duas entregas em branches diferentes. A branch main contém o Trabalho 1, no qual o robô explora a arena, localiza a bandeira inimiga e se posiciona diante dela. A branch trabalho-2 contém o Trabalho 2, que acrescenta a captura da bandeira com a garra, o transporte de volta e o depósito na base. Este README é o da branch trabalho-2, ou seja, descreve a missão completa do Trabalho 2. Os passos de instalação e de compilação são iguais nas duas branches, mudando apenas o nome da branch no momento de clonar e o que a missão faz ao final.

## Requisitos

O sistema foi feito para Ubuntu 22.04 com ROS 2 Humble e Gazebo Fortress na variante Ignition. As dependências principais são a biblioteca cliente rclpy, os pacotes de mensagens padrão do ROS, a ponte ros_gz_bridge, o ros_gz_sim, o ign_ros2_control, o ros2_control e os ros2_controllers, o robot_state_publisher, o xacro, o rviz2, o topic_tools, a biblioteca OpenCV com o cv_bridge, além de numpy e scipy. Todas as dependências estão declaradas no arquivo package.xml.

Caso o ROS 2 e o Gazebo ainda não estejam instalados na máquina, o repositório inclui um script auxiliar em scripts/install_ros_humble.sh que configura o repositório de pacotes do ROS e instala tudo o que é necessário.

## Instalação do zero em Ubuntu 22.04

Esta seção descreve a instalação completa em uma máquina Ubuntu 22.04 sem nada instalado, do sistema até o projeto rodando. Importante, não execute estes passos com um ambiente Conda ativo, porque o Conda coloca as bibliotecas dele de Qt e OpenGL na frente das do sistema e isso impede a interface do Gazebo de subir. Se o início do terminal mostrar a marca de ambiente base, rode conda deactivate antes, ou não use Conda nesta máquina.

Passo 1, ferramentas básicas e locale.

```
sudo apt update
sudo apt install -y git curl gnupg lsb-release software-properties-common locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Passo 2, habilitar o repositório universe.

```
sudo add-apt-repository -y universe
```

Passo 3, adicionar a chave e o repositório de pacotes do ROS 2.

```
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Passo 4, instalar o ROS 2 Humble, o Gazebo Fortress e as dependências. O pacote ros-humble-ros-gz já traz o Gazebo Fortress junto.

```
sudo apt install -y \
  ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-cv-bridge ros-humble-image-transport ros-humble-topic-tools ros-humble-teleop-twist-keyboard \
  python3-opencv python3-scipy python3-numpy
```

Passo 5, inicializar o rosdep.

```
sudo rosdep init
rosdep update
```

Passo 6, criar o workspace e clonar este projeto na branch do Trabalho 2.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b trabalho-2 https://github.com/LVinaud/evolutionary_explorer_ros.git
```

Passo 7, resolver as dependências do pacote e compilar.

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Como atalho dos passos 1 a 5, depois de clonar o repositório no passo 6 você pode rodar o script que acompanha o pacote, que faz a mesma instalação do ROS, do Gazebo e das dependências, e então seguir do passo 7 em diante.

```
cd ~/ros2_ws/src/evolutionary_explorer_ros
chmod +x scripts/install_ros_humble.sh
ROS_PROFILE=desktop ./scripts/install_ros_humble.sh
```

Depois de compilar, siga para a seção de execução. Em máquina nativa com GPU não é preciso o argumento software_render, porque o padrão já usa a placa de vídeo. O software_render só é necessário em máquina virtual sem aceleração 3D.

## Instalação das dependências

Com o ROS 2 já instalado, as dependências do pacote podem ser resolvidas pelo rosdep a partir da raiz do workspace. Os comandos a seguir assumem um workspace chamado ros2_ws na pasta do usuário.

```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Compilação

Coloque a pasta deste pacote dentro de ~/ros2_ws/src e compile o workspace com o colcon a partir da raiz do workspace. Em seguida atualize o ambiente do terminal.

```
cd ~/ros2_ws
colcon build --symlink-install --packages-select evolutionary_explorer_ros
source install/setup.bash
```

## Execução

A execução completa da missão usa dois terminais. Em ambos é necessário carregar o ambiente do ROS e o ambiente do workspace antes de qualquer comando, com as duas linhas de source que aparecem no início de cada bloco abaixo. O primeiro terminal abre o mundo no Gazebo e o segundo coloca o robô no mundo e inicia o controle autônomo.

### Passo 1, abrir o mundo no Gazebo

No primeiro terminal inicie o mundo. Esse comando carrega a arena padrão com as duas bandeiras, as duas bases, as paredes e os obstáculos cilíndricos. Espere a janela do Gazebo terminar de abrir e a cena aparecer antes de seguir para o passo 2.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py
```

Observação importante para quem usa máquina virtual. Em uma máquina virtual sem aceleração de vídeo 3D, por exemplo com o adaptador gráfico VMware SVGA, a janela do Gazebo pode abrir com a área de visualização em branco e o quadriculado piscando, mesmo com o mundo carregado corretamente. Isso acontece porque a engine de renderização ogre2 não consegue desenhar a cena 3D com o driver de vídeo virtual. O servidor da simulação continua funcionando normalmente, e a prova disso é que a árvore de entidades do Gazebo lista todos os modelos e o RViz mostra o robô. Para que a janela do Gazebo também mostre a cena, inicie o mundo com a renderização por software ligada, usando o argumento software_render com valor 1. A simulação fica mais lenta porque a CPU passa a desenhar a cena, mas o mundo aparece.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py software_render:=1
```

Apenas o lançamento do mundo precisa do argumento software_render, porque é ele que abre a janela do Gazebo. O lançamento da missão, descrito no passo 2, não precisa desse argumento. Como alternativa mais rápida, quando for possível alterar as configurações da máquina virtual, ative a aceleração de gráficos 3D nas opções de vídeo da máquina virtual e instale as ferramentas de integração do sistema convidado. Isso usa a GPU do hospedeiro e dispensa a renderização por software. Existe ainda o argumento headless com valor true, que executa o Gazebo sem janela gráfica, útil para testes automatizados e para a fase de computação evolutiva.

O cenário é escolhido pelo argumento world. Sem ele, carrega o cenário padrão com obstáculos do Trabalho 1. Os três cenários de teste do Trabalho 2 são o mapa aberto e os mapas com obstáculos, e se iniciam assim.

```
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py world:=empty_arena.sdf software_render:=1
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py world:=arena_cilindros.sdf software_render:=1
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py world:=arena_paredes.sdf software_render:=1
```

### Passo 2, iniciar o robô e a missão

No segundo terminal, depois que a arena terminar de abrir, carregue o robô e o controle autônomo da missão. Esse comando coloca o robô no mundo, sobe os sensores, estabelece a ponte entre o Gazebo e o ROS, abre o RViz, publica a odometria de referência e inicia os nós de percepção da bandeira e de controle da missão. O robô passa a explorar automaticamente.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros missao.launch.py
```

### O que esperar durante a execução

Logo após o passo 2 o robô começa a explorar a arena partindo da base vermelha. Ele constrói um mapa de ocupação a partir do LIDAR enquanto se move e planeja uma rota livre na direção do lado adversário, contornando os cilindros. Ao avistar a bandeira azul, o robô passa a navegar até ela, sempre desviando dos obstáculos pelo caminho. Quando a bandeira fica grande na imagem, sinal de que está perto, o robô entra na fase de posicionamento, faz o ajuste fino de frente para a bandeira e conclui a missão. Em máquinas sem aceleração de vídeo a simulação roda abaixo do tempo real, então a travessia da arena pode levar alguns minutos. Isso é esperado e não indica falha.

### Acompanhar a máquina de estados

Para observar a troca de estados durante a execução, abra um terceiro terminal e acompanhe o tópico de estado da missão. Os estados vão aparecer na sequência EXPLORANDO, BANDEIRA_DETECTADA, NAVEGANDO_PARA_BANDEIRA, POSICIONANDO_PARA_COLETA e, por fim, MISSAO_CONCLUIDA.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /mission/state
```

### Argumentos úteis do lançamento da missão

O lançamento da missão aceita argumentos que facilitam os testes. O argumento use_rviz com valor false desliga o RViz, o que deixa a execução muito mais leve, porque o RViz é um segundo desenho 3D competindo com a renderização da câmera. Em máquina virtual sem aceleração de vídeo, desligar o RViz é o ajuste que mais acelera a simulação, e dá para acompanhar a missão pelos tópicos. Os argumentos spawn_x, spawn_y, spawn_z e spawn_yaw definem a posição e a orientação iniciais do robô. O argumento params_file permite carregar um arquivo de parâmetros diferente, recurso pensado para a fase de computação evolutiva. O exemplo a seguir inicia o robô em uma posição mais próxima da bandeira adversária, o que permite observar a sequência completa em pouco tempo.

```
ros2 launch evolutionary_explorer_ros missao.launch.py spawn_x:=3.0
```

### Controle pelo teclado

Como alternativa ao controle autônomo, o robô também pode ser conduzido pelo teclado. Para isso, inicie o mundo no primeiro terminal, carregue apenas o robô e os sensores com o lançamento carrega_robo no segundo terminal e use o teleop no terceiro terminal.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Também existe um lançamento chamado teste_urdf que abre apenas a descrição do robô no RViz, sem física, útil para inspecionar a modelagem.

## Arquitetura geral do sistema

O sistema é organizado em nós independentes que se comunicam por tópicos. O simulador Gazebo publica os dados dos sensores em seu próprio sistema de tópicos e a ponte ros_gz_bridge converte esses dados para tópicos ROS. A partir daí, o nó de percepção processa a imagem da câmera e o nó de controle decide o que o robô deve fazer.

O nó mission_control é o cérebro do sistema. Ele implementa a máquina de estados, lê os dados do LIDAR, da câmera processada pela percepção, da odometria e da unidade inercial, e publica comandos de velocidade no tópico cmd_vel. Esses comandos são redirecionados para o controlador de tração diferencial do robô.

O nó flag_detector é responsável pela percepção. Ele recebe a imagem da câmera de segmentação semântica e identifica a bandeira adversária, publicando se a bandeira está visível, qual o deslocamento horizontal dela em relação ao centro da imagem e qual a fração da imagem que ela ocupa, que serve como medida de proximidade.

O nó ground_truth_odometry publica a odometria de referência do robô e a transformação entre os sistemas de coordenadas, a partir da pose fornecida pelo simulador. O nó robo_mapper mantém um mapa de ocupação simples para visualização e é opcional.

Os principais tópicos do sistema são os seguintes. O tópico cmd_vel transporta o comando de velocidade do controle para o robô. O tópico scan transporta as leituras do LIDAR. O tópico imu transporta os dados da unidade inercial. Os tópicos sob robot_cam transportam os mapas da câmera de segmentação. Os tópicos de bandeira detectada, de deslocamento e de área transportam o resultado da percepção da bandeira. O tópico mission state publica o estado atual da máquina de estados. O tópico mission start permite iniciar a missão caso o robô esteja configurado para aguardar comando.

## Máquina de estados

A máquina de estados é o módulo central e está implementada no arquivo mission_control.py, com cada estado documentado no próprio código. Os estados são os seguintes.

No estado AGUARDANDO_COMANDO o robô permanece parado até receber a ordem de início. Por padrão a missão começa imediatamente, mas é possível configurar o robô para esperar um comando externo.

No estado EXPLORANDO o robô varre o ambiente em busca da bandeira. Enquanto se move, ele constrói um mapa de ocupação da arena a partir das leituras do LIDAR e planeja por busca A estrela uma rota livre até um ponto à frente, na direção do lado adversário, seguindo essa rota por uma sequência de pontos de passagem. Esse planejamento sobre o mapa construído evita os mínimos locais em que uma navegação puramente reativa prendia o robô no aglomerado de obstáculos. Caso o planejamento não encontre rota em um instante, o robô recorre a um comportamento reativo de campos potenciais como reserva. Quando a bandeira é confirmada pela percepção, o robô passa ao estado seguinte.

No estado BANDEIRA_DETECTADA o robô registra que a bandeira foi identificada visualmente e estima a direção dela em relação à câmera, usando o deslocamento horizontal e o campo de visão. Em seguida segue para a navegação.

No estado NAVEGANDO_PARA_BANDEIRA o robô estima a direção da bandeira a partir do deslocamento horizontal dela na imagem e do campo de visão da câmera e planeja por A estrela uma rota até um ponto naquela direção, sobre o mesmo mapa de ocupação construído pelo LIDAR. Dessa forma ele se aproxima da bandeira contornando os cilindros que aparecem no caminho em vez de avançar reto contra eles. Quando a bandeira ocupa uma fração suficiente da imagem, sinal de que está realmente perto, e está razoavelmente centralizada, o robô passa ao posicionamento. A transição usa a área da bandeira na imagem como critério, e não a distância frontal do LIDAR, porque um cilindro entre o robô e a bandeira seria confundido com a bandeira e faria o robô parar cedo demais.

No estado POSICIONANDO_PARA_COLETA o robô faz o ajuste fino para a captura. Ele alinha com o mastro e avança ao mesmo tempo, numa perseguição suave que gira proporcional ao deslocamento da bandeira na imagem e reduz a velocidade quanto mais descentralizado, evitando o movimento pendular. Como o centro do agrupamento azul é puxado para o lado do painel, a mira recebe um pequeno viés lateral para apontar no mastro, e não no painel, de modo que a garra encaixe no mastro. Se durante essa aproximação houver um obstáculo entre o robô e a bandeira, identificado por algo perto à frente enquanto a bandeira ainda está pequena na imagem, o robô contorna esse obstáculo por A estrela ali mesmo, sem avançar contra ele. Quando a bandeira fica grande na imagem, sinal de que está em posição de pega, e o robô está alinhado, ele passa para a captura. O LIDAR atua como segurança para evitar colisão.

No estado CAPTURANDO_BANDEIRA o robô executa uma sequência temporizada do manipulador para agarrar a bandeira por atrito. Primeiro abaixa a haste na altura do mastro e abre os dois braços, depois avança devagar o pouco que falta para enfiar o mastro entre os braços, em seguida fecha os braços prendendo o mastro e por fim eleva a haste com a bandeira presa, pronto para transportar. Durante essa sequência o robô só se move na etapa de avanço.

No estado RETORNANDO_PARA_BASE o robô mantém a bandeira presa e elevada e planeja por A estrela uma rota até a sua base, contornando os obstáculos pela grade de ocupação construída com o LIDAR. A referência da base é a posição inicial do robô, gravada no começo da missão, já que o robô sempre nasce sobre o centro da base. Enquanto carrega a bandeira, o robô ignora as leituras do LIDAR mais próximas que uma certa distância, para não confundir o mastro preso na garra com um obstáculo, e gira no lugar quando precisa dar meia-volta antes de avançar. Uma detecção de travamento com manobra de escape continua ativa durante o transporte.

No estado POSICIONANDO_PARA_DEPOSITO o robô faz o ajuste fino sobre o ponto de depósito, que é a posição inicial dentro do círculo demarcado da base. Ele gira para o ponto e avança em direção ao centro. Como ao entrar neste estado o robô já está dentro do círculo de depósito, ele deposita ao chegar perto do centro ou após um tempo limite tentando, o que evita ficar preso tentando centralizar quando a borda do disco ou obstáculos próximos da base impedem o ajuste fino.

No estado DEPOSITANDO_BANDEIRA o robô abaixa a haste e abre os braços para soltar a bandeira no círculo de depósito e em seguida recua um pouco, para não reencostar nela nem derrubá-la. Depois disso a missão é concluída.

No estado REDETECTANDO_BANDEIRA o robô trata a perda da bandeira do campo de visão. Ele gira no lugar, no sentido em que a bandeira foi vista por último, para reencontrá-la. Caso não a reencontre dentro de um tempo limite, volta a explorar.

No estado MISSAO_CONCLUIDA o robô para, tendo capturado a bandeira adversária, retornado à base e depositado a bandeira no círculo amarelo. Um breve movimento de comemoração é executado nos primeiros segundos.

A robustez do sistema aparece principalmente em dois pontos. O primeiro é o tratamento da perda da bandeira, que leva o robô a girar e reencontrá-la em vez de prosseguir às cegas. O segundo é a prioridade do desvio de obstáculos sobre a perseguição da bandeira, além de uma detecção de travamento que dispara uma manobra de escape quando o robô deixa de progredir.

## Manipulador e captura da bandeira

A garra usada é a do robô base do professor, composta por uma haste de sustentação que sobe e desce por uma junta revolucional e por dois braços laterais acionados por juntas prismáticas, que abrem e fecham deslizando para os lados. O controle é feito publicando no tópico do controlador da garra um vetor de três valores na ordem definida pelo professor, elevação da haste, abertura do braço direito e abertura do braço esquerdo. A máquina de estados publica essas poses diretamente, nas etapas da captura e do depósito.

A captura é feita por atrito. Quando o robô está em posição de pega, a haste desce até a altura do mastro, os braços abrem, o robô avança o pouco que falta para o mastro entrar entre os braços, os braços fecham prendendo o mastro e a haste sobe levando a bandeira. Para que isso seja possível, as colisões dos braços e das pontas da garra, que tinham sido removidas no Trabalho 1 para estabilizar o robô, foram restauradas, já que são essas superfícies que prendem o mastro. O lastro baixo introduzido no Trabalho 1 mantém o robô estável mesmo com as colisões da garra de volta.

Os ângulos das poses, os tempos de cada etapa e a velocidade de avanço da captura ficam em parâmetros, no arquivo de configuração da missão, e podem ser ajustados sem recompilar para afinar a pega conforme o cenário.

## Detecção visual da bandeira

A câmera do robô é do tipo segmentação semântica, definida na descrição do robô. O simulador atribui a cada objeto da cena uma etiqueta numérica e publica um mapa de etiquetas no qual o valor de cada pixel corresponde à etiqueta do objeto naquele ponto. No mundo padrão, a bandeira adversária, que é a azul, recebe a etiqueta de número vinte e cinco, enquanto a bandeira do próprio time, a vermelha, recebe a etiqueta vinte. O robô nasce no lado vermelho, portanto a bandeira a ser capturada é a azul.

O nó de percepção lê o mapa de etiquetas, isola os pixels cujo valor corresponde à etiqueta da bandeira adversária, encontra o maior agrupamento desses pixels e calcula o centro e a área desse agrupamento. A partir do centro obtém o deslocamento horizontal normalizado da bandeira na imagem e a partir da área obtém uma medida de proximidade. A detecção por etiqueta é o método padrão porque distingue a bandeira azul da base azul, que tem a mesma cor. Uma detecção apenas por cor confundiria a bandeira com a base e com o piso azul do lado adversário, fazendo o robô declarar chegada longe da bandeira. Mesmo assim, o nó oferece um modo alternativo de detecção por cor, que usa uma câmera RGB comum e uma faixa de cor em torno do azul da bandeira, útil em cenários onde a bandeira tenha uma cor distinta da base. O enunciado do Trabalho 2 fornece a máscara de cor da bandeira azul para esse modo.

## Mapa de ocupação e planejamento por A estrela

A navegação do robô se apoia em um mapa de ocupação bidimensional construído em tempo real a partir do LIDAR. É importante destacar que esse mapa não é conhecido de antemão. O robô começa com uma grade vazia e a preenche enquanto explora, marcando como ocupadas as células onde o LIDAR detecta obstáculos. Cada obstáculo detectado é inflado pelo raio do robô, de modo que as rotas planejadas já mantêm uma folga de segurança em relação aos cilindros e paredes. As células ainda não observadas são tratadas como livres, de forma otimista, para que o robô possa planejar rumo a regiões que ainda não viu.

Sobre essa grade construída pelo robô, a busca A estrela encontra a rota livre mais curta até um ponto objetivo. Na exploração, o objetivo é um ponto à frente na direção do lado adversário. Na navegação até a bandeira, o objetivo é um ponto na direção em que a bandeira foi vista pela câmera. A rota resultante é simplificada em uma sequência de pontos de passagem que o robô segue por controle de rumo. Esse planejamento sobre o mapa construído resolve o problema dos mínimos locais, nos quais o robô, guiado apenas por reação imediata aos obstáculos, ficava preso ao tentar atravessar o aglomerado denso de cilindros que fica logo à frente da base.

O código do mapa e da busca está no arquivo planner.py, em uma classe dedicada que não depende do ROS. Isso permite testar o planejamento de forma isolada e reutilizá-lo em um avaliador de desempenho sem simulação gráfica na fase de computação evolutiva.

## Desvio de obstáculos com o LIDAR

As leituras do LIDAR são agrupadas em setores que descrevem a distância mínima à frente, nas diagonais frontais e nas laterais. Esse agrupamento está em um módulo auxiliar livre de dependências do ROS, o que facilita testar a lógica de forma isolada e reutilizá-la em um avaliador de desempenho sem simulação gráfica.

Os setores do LIDAR servem como salvaguarda imediata sobre o planejamento. A velocidade de avanço diminui à medida que um obstáculo se aproxima e chega a zero em distância crítica, situação na qual o robô apenas gira para se reorientar. Caso a busca A estrela não retorne uma rota em algum instante, o robô recorre a um comportamento reativo de campos potenciais, com uma componente que o atrai para a direção desejada e uma componente que o repele dos obstáculos próximos. Além disso, uma detecção de travamento observa se o robô deixou de progredir e, nesse caso, dispara uma manobra de escape que recua um pouco e gira antes de voltar a planejar.

## Modelagem do robô e sensores

A descrição do robô está em description/robot.urdf.xacro, no formato Xacro. O robô é diferencial, com duas rodas motorizadas, um apoio frontal, uma câmera de segmentação, um LIDAR, uma unidade inercial e um manipulador (garra) com uma haste de elevação e dois braços laterais. A descrição também inclui uma câmera RGB comum, mantida desativada por padrão, disponível para a detecção por cor. As transformações entre os sistemas de coordenadas são publicadas pelo robot_state_publisher e pela odometria de referência.

As modificações feitas sobre o robô base incluem a renomeação do robô, a correção das referências internas ao novo pacote e a adição de um elemento de sinalização no topo, que serve como identificação visual e demonstra a inserção de um novo link e de uma nova junta. Foram feitas também modificações voltadas à estabilidade, descritas na próxima seção.

## Estabilidade do robô

Durante os testes em simulação, o robô base mostrou tendência a tombar para frente ao encostar nos obstáculos, devido ao centro de massa elevado e a um apoio frontal que prendia em curvas. Para corrigir isso, foram adotadas as seguintes medidas. Foi acrescentado um lastro de massa baixa próximo ao solo, o que abaixa bastante o centro de massa e deixa o robô estável ao encostar em obstáculos. O apoio frontal passou a ter atrito praticamente nulo, para deslizar livremente nas curvas em vez de prender. As acelerações do controlador foram suavizadas. Por fim, a máquina de estados conta com uma salvaguarda que detecta inclinação excessiva pela unidade inercial e interrompe os comandos nessa situação. As colisões da garra haviam sido removidas no Trabalho 1, mas foram restauradas no Trabalho 2 porque são elas que prendem o mastro na captura, e o lastro baixo mantém o robô estável mesmo com essas colisões de volta.

## Parâmetros e preparação para a computação evolutiva

Todos os ganhos e limiares que governam o comportamento ficam reunidos no arquivo config/mission_params.yaml e espelhados na classe MissionParams, no arquivo mission_params.py. A máquina de estados lê exclusivamente desses parâmetros, de modo que a lógica de decisão fica separada dos valores numéricos.

Essa separação é o ponto de injeção da computação evolutiva planejada para a segunda fase. A classe de parâmetros oferece métodos para serializar o conjunto de valores como um vetor, que faz o papel de cromossomo, e para reconstruir os parâmetros a partir de um vetor, além de fornecer os limites de cada valor evoluível. O lançamento aceita um arquivo de parâmetros externo, o que permitirá a um otimizador gerar um arquivo por indivíduo e executar um episódio de avaliação para cada conjunto de parâmetros. Os tópicos de estado da missão e de odometria servem como sinais para calcular o desempenho de cada indivíduo, por exemplo tempo até alcançar a bandeira, distância percorrida e ocorrência de colisões. Dessa forma, nenhuma alteração na máquina de estados será necessária para evoluir o comportamento, apenas os valores mudam.

## Estrutura do pacote

O pacote segue a estrutura de um pacote Python do ROS 2. O arquivo package.xml descreve o pacote e suas dependências. O arquivo setup.py define os pontos de entrada dos nós e a instalação dos recursos. A pasta config contém os parâmetros da missão e a configuração dos controladores. A pasta description contém a descrição do robô em Xacro. A pasta launch contém os arquivos de lançamento. A pasta world contém as arenas e a pasta models contém os obstáculos e demais modelos. A pasta rviz contém as configurações de visualização. A pasta com o mesmo nome do pacote contém os nós em Python e os módulos auxiliares, entre eles mission_control.py com a máquina de estados, flag_detector.py com a percepção, planner.py com o mapa de ocupação construído pelo LIDAR e a busca A estrela, navigation.py com as utilidades de LIDAR e navegação, mission_params.py com os parâmetros, ground_truth_odometry.py com a odometria de referência e robo_mapper.py com o mapeamento opcional. A pasta test contém os testes, incluindo testes do módulo de navegação e dos parâmetros.

## Estado atual e limitações conhecidas

O robô executa a missão completa do Trabalho 2 de ponta a ponta, exploração, detecção, navegação, posicionamento, captura por atrito, retorno à base e depósito no círculo, terminando no estado de missão concluída. Isso foi verificado no mapa aberto, empty_arena, e no mapa com cilindros, arena_cilindros. Não tomba ao encostar em obstáculos, e o mapa de ocupação construído em tempo real com o planejamento por A estrela permite atravessar o campo de cilindros de forma estável, tanto na ida quanto na volta carregando a bandeira. A sequência de comandos da garra segue a ordem do controlador do professor.

A captura por atrito depende do alinhamento entre a garra e o mastro, então os ângulos da haste, a área de início da captura e os tempos de cada etapa ficam expostos como parâmetros em config/mission_params.yaml e podem precisar de pequeno ajuste conforme o cenário, sem recompilar. A principal limitação prática é o tempo de execução em máquinas sem aceleração de vídeo, onde a simulação roda bem abaixo do tempo real, e por isso desligar o RViz com use_rviz false ajuda bastante. Os ganhos de exploração, de desvio, de planejamento e de captura continuam sendo bons candidatos a ajuste automático na fase de computação evolutiva, com o objetivo de obter um comportamento ainda mais rápido e robusto.

## Documentação da feira

O material de apresentação em formato de pôster ou de slides será disponibilizado por meio de um link nesta seção.

## Créditos

Projeto baseado no pacote da disciplina SSC0712, sob responsabilidade do Prof. Dr. Matheus Machado dos Santos, disponível em github.com/matheusbg8/prm_2026, sob licença Apache 2.0.
