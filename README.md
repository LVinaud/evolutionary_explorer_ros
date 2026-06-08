# evolutionary_explorer_ros

## Visao geral

Este pacote ROS 2 implementa o sistema de controle de um robo movel diferencial que explora um ambiente simulado, localiza a bandeira do time adversario por visao computacional, navega ate ela desviando de obstaculos com o sensor LIDAR e se posiciona de frente para ela para a coleta. Todo o comportamento e coordenado por uma maquina de estados, que e o modulo central do trabalho. O projeto foi desenvolvido para a disciplina SSC0712 Programacao de Robos Moveis do ICMC USP e utiliza ROS 2 Humble com o simulador Gazebo Fortress.

O pacote deriva do pacote base da disciplina, disponivel em github.com/matheusbg8/prm_2026. O robo, os sensores, os mundos e a infraestrutura de lancamento foram reaproveitados e adaptados. O pacote recebeu um nome proprio, o robo foi renomeado para explorer_robot e foram acrescentados os nos de percepcao e de controle da missao, alem de modificacoes de modelagem e estabilidade.

O nome do pacote contem a palavra evolutionary porque o projeto tem uma segunda fase planejada, na qual os parametros de comportamento do robo serao ajustados por computacao evolutiva. A arquitetura ja foi preparada para isso, mantendo toda a logica de decisao separada dos valores numericos de comportamento. Esses valores ficam reunidos em um unico arquivo de parametros e em uma classe de dados dedicada, de modo que evoluir o comportamento significa apenas trocar valores, sem alterar a maquina de estados.

## Requisitos

O sistema foi feito para Ubuntu 22.04 com ROS 2 Humble e Gazebo Fortress na variante Ignition. As dependencias principais sao a biblioteca cliente rclpy, os pacotes de mensagens padrao do ROS, a ponte ros_gz_bridge, o ros_gz_sim, o ign_ros2_control, o ros2_control e os ros2_controllers, o robot_state_publisher, o xacro, o rviz2, o topic_tools, a biblioteca OpenCV com o cv_bridge, alem de numpy e scipy. Todas as dependencias estao declaradas no arquivo package.xml.

Caso o ROS 2 e o Gazebo ainda nao estejam instalados na maquina, o repositorio inclui um script auxiliar em scripts/install_ros_humble.sh que configura o repositorio de pacotes do ROS e instala tudo o que e necessario.

## Instalacao das dependencias

Com o ROS 2 ja instalado, as dependencias do pacote podem ser resolvidas pelo rosdep a partir da raiz do workspace. Os comandos a seguir assumem um workspace chamado ros2_ws na pasta do usuario.

```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Compilacao

Coloque a pasta deste pacote dentro de ~/ros2_ws/src e compile o workspace com o colcon a partir da raiz do workspace. Em seguida atualize o ambiente do terminal.

```
cd ~/ros2_ws
colcon build --symlink-install --packages-select evolutionary_explorer_ros
source install/setup.bash
```

## Execucao

A execucao usa dois terminais. Em ambos e necessario carregar o ambiente do ROS e o ambiente do workspace antes de qualquer comando. As linhas de source aparecem no inicio de cada bloco abaixo.

No primeiro terminal inicie o mundo no Gazebo. Esse comando carrega a arena padrao com as duas bandeiras, as bases, as paredes e os obstaculos cilindricos.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py
```

Importante para quem usa maquina virtual. Em uma maquina virtual sem aceleracao de video 3D, por exemplo com o adaptador grafico VMware SVGA, a janela do Gazebo pode abrir com a area de visualizacao em branco e o quadriculado piscando, mesmo com o mundo carregado corretamente. Isso acontece porque a engine de renderizacao ogre2 nao consegue desenhar a cena 3D com o driver de video virtual. O servidor da simulacao continua funcionando normalmente, e a prova disso e que a arvore de entidades do Gazebo lista todos os modelos e o RViz mostra o robo. Para que a janela do Gazebo tambem mostre a cena, inicie o mundo com a renderizacao por software ligada, usando o argumento software_render com valor 1. A simulacao fica mais lenta porque a CPU passa a desenhar a cena, mas o mundo aparece.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py software_render:=1
```

Apenas o lancamento do mundo precisa do argumento software_render, porque e ele que abre a janela do Gazebo. O lancamento da missao, descrito a seguir, nao precisa desse argumento. Como alternativa mais rapida, quando for possivel alterar as configuracoes da maquina virtual, ative a aceleracao de graficos 3D nas opcoes de video da maquina virtual e instale as ferramentas de integracao do sistema convidado. Isso usa a GPU do hospedeiro e dispensa a renderizacao por software. Existe ainda o argumento headless com valor true, que executa o Gazebo sem janela grafica, util para testes automatizados e para a fase de computacao evolutiva.

No segundo terminal, depois que a arena terminar de abrir, carregue o robo e o controle autonomo da missao. Esse comando coloca o robo no mundo, sobe os sensores, estabelece a ponte entre o Gazebo e o ROS, abre o RViz, publica a odometria de referencia e inicia os nos de percepcao da bandeira e de controle da missao. O robo passa a explorar automaticamente.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch evolutionary_explorer_ros missao.launch.py
```

O lancamento da missao aceita argumentos uteis. O argumento use_rviz com valor false desliga o RViz, o que e util para testes mais leves. Os argumentos spawn_x, spawn_y, spawn_z e spawn_yaw definem a posicao e a orientacao iniciais do robo. O argumento params_file permite carregar um arquivo de parametros diferente, recurso pensado para a fase de computacao evolutiva. O exemplo a seguir inicia o robo em uma posicao mais proxima da bandeira adversaria, o que permite observar a sequencia completa de deteccao, navegacao e posicionamento em pouco tempo.

```
ros2 launch evolutionary_explorer_ros missao.launch.py spawn_x:=3.0
```

Para observar a troca de estados da maquina de estados durante a execucao, abra um terceiro terminal e acompanhe o topico de estado da missao.

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /mission/state
```

Como alternativa ao controle autonomo, o robo tambem pode ser conduzido pelo teclado. Para isso, inicie o mundo no primeiro terminal, carregue apenas o robo e os sensores com o lancamento carrega_robo no segundo terminal e use o teleop no terceiro terminal.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Tambem existe um lancamento chamado teste_urdf que abre apenas a descricao do robo no RViz, sem fisica, util para inspecionar a modelagem.

## Arquitetura geral do sistema

O sistema e organizado em nos independentes que se comunicam por topicos. O simulador Gazebo publica os dados dos sensores em seu proprio sistema de topicos e a ponte ros_gz_bridge converte esses dados para topicos ROS. A partir dai, o no de percepcao processa a imagem da camera e o no de controle decide o que o robo deve fazer.

O no mission_control e o cerebro do sistema. Ele implementa a maquina de estados, le os dados do LIDAR, da camera processada pela percepcao, da odometria e da unidade inercial, e publica comandos de velocidade no topico cmd_vel. Esses comandos sao redirecionados para o controlador de tracao diferencial do robo.

O no flag_detector e responsavel pela percepcao. Ele recebe a imagem da camera de segmentacao semantica e identifica a bandeira adversaria, publicando se a bandeira esta visivel, qual o deslocamento horizontal dela em relacao ao centro da imagem e qual a fracao da imagem que ela ocupa, que serve como medida de proximidade.

O no ground_truth_odometry publica a odometria de referencia do robo e a transformacao entre os sistemas de coordenadas, a partir da pose fornecida pelo simulador. O no robo_mapper mantem um mapa de ocupacao simples e e opcional.

Os principais topicos do sistema sao os seguintes. O topico cmd_vel transporta o comando de velocidade do controle para o robo. O topico scan transporta as leituras do LIDAR. O topico imu transporta os dados da unidade inercial. Os topicos sob robot_cam transportam os mapas da camera de segmentacao. O topico flag detectado, o topico de deslocamento e o topico de area transportam o resultado da percepcao da bandeira. O topico mission state publica o estado atual da maquina de estados. O topico mission start permite iniciar a missao caso o robo esteja configurado para aguardar comando.

## Maquina de estados

A maquina de estados e o modulo central e esta implementada no arquivo mission_control.py, com cada estado documentado no proprio codigo. Os estados sao os seguintes.

No estado AGUARDANDO_COMANDO o robo permanece parado ate receber a ordem de inicio. Por padrao a missao comeca imediatamente, mas e possivel configurar o robo para esperar um comando externo.

No estado EXPLORANDO o robo varre o ambiente em busca da bandeira. O movimento combina uma atracao na direcao do lado adversario com uma repulsao em relacao aos obstaculos detectados pelo LIDAR, de forma que o robo avanca pela arena contornando os obstaculos. Quando a bandeira e confirmada pela percepcao, o robo passa ao estado seguinte.

No estado BANDEIRA_DETECTADA o robo registra que a bandeira foi identificada visualmente e estima a direcao dela em relacao a camera, usando o deslocamento horizontal e o campo de visao. Em seguida segue para a navegacao.

No estado NAVEGANDO_PARA_BANDEIRA o robo se dirige a bandeira por um controle proporcional sobre o deslocamento horizontal, mantendo a bandeira centralizada enquanto avanca. O desvio de obstaculos tem prioridade sobre a perseguicao, exceto quando o objeto a frente e a propria bandeira ja centralizada. Quando a bandeira fica suficientemente proxima e centralizada, o robo passa ao posicionamento.

No estado POSICIONANDO_PARA_COLETA o robo faz o ajuste fino. Ele se aproxima centralizando a bandeira ate que ela ocupe uma fracao suficiente da imagem, o que indica proximidade adequada. O LIDAR atua como seguranca para evitar colisao. Esse criterio baseado na area da imagem foi escolhido porque o mastro da bandeira e fino e a leitura direta de distancia pelo LIDAR sobre ele e pouco confiavel.

No estado REDETECTANDO_BANDEIRA o robo trata a perda da bandeira do campo de visao. Ele gira no lugar, no sentido em que a bandeira foi vista por ultimo, para reencontra-la. Caso nao a reencontre dentro de um tempo limite, volta a explorar.

No estado MISSAO_CONCLUIDA o robo para, tendo alcancado e se posicionado diante da bandeira adversaria. Um breve movimento de comemoracao e executado nos primeiros segundos.

A robustez do sistema aparece principalmente em dois pontos. O primeiro e o tratamento da perda da bandeira, que leva o robo a girar e reencontra-la em vez de prosseguir as cegas. O segundo e a prioridade do desvio de obstaculos sobre a perseguicao da bandeira, alem de uma deteccao de travamento que dispara uma manobra de escape quando o robo deixa de progredir.

## Deteccao visual da bandeira

A camera do robo e do tipo segmentacao semantica, definida na descricao do robo. O simulador atribui a cada objeto da cena uma etiqueta numerica e publica um mapa de etiquetas no qual o valor de cada pixel corresponde a etiqueta do objeto naquele ponto. No mundo padrao, a bandeira adversaria, que e a azul, recebe a etiqueta de numero vinte e cinco, enquanto a bandeira do proprio time, a vermelha, recebe a etiqueta vinte. O robo nasce no lado vermelho, portanto a bandeira a ser capturada e a azul.

O no de percepcao le o mapa de etiquetas, isola os pixels cujo valor corresponde a etiqueta da bandeira adversaria, encontra o maior agrupamento desses pixels e calcula o centro e a area desse agrupamento. A partir do centro obtem o deslocamento horizontal normalizado da bandeira na imagem e a partir da area obtem uma medida de proximidade. Essa abordagem por etiqueta e mais robusta do que casar uma cor exata, porque nao depende de calibrar valores de cor. Ainda assim, o no oferece um modo alternativo de deteccao por cor, para quem preferir trabalhar com o mapa colorido da segmentacao.

## Navegacao e desvio de obstaculos

As leituras do LIDAR sao agrupadas em setores que descrevem a distancia minima a frente, nas diagonais frontais e nas laterais. Esse agrupamento esta em um modulo auxiliar livre de dependencias do ROS, o que facilita testar a logica de forma isolada e reutiliza-la em um avaliador de desempenho sem simulacao grafica.

Durante a exploracao, o robo se move por um esquema inspirado em campos potenciais. Existe uma componente que o atrai para a direcao do lado adversario e uma componente que o repele dos obstaculos proximos, somadas em um unico comando de giro. A velocidade de avanco diminui a medida que um obstaculo se aproxima e chega a zero em distancia critica, situacao na qual o robo apenas gira para se reorientar. Com isso, o robo tende a contornar os obstaculos em vez de avancar contra eles.

Durante a navegacao ate a bandeira, o desvio continua ativo, com a ressalva de que um objeto a frente, quando coincide com a bandeira ja centralizada, e tratado como o proprio alvo e nao como um obstaculo a evitar. O posicionamento final controla a aproximacao pela area da bandeira na imagem, parando a uma distancia adequada e com a bandeira centralizada.

## Modelagem do robo e sensores

A descricao do robo esta em description/robot.urdf.xacro, no formato Xacro. O robo e diferencial, com duas rodas motorizadas, um apoio frontal, uma camera de segmentacao, um LIDAR e uma unidade inercial. As transformacoes entre os sistemas de coordenadas sao publicadas pelo robot_state_publisher e pela odometria de referencia.

As modificacoes feitas sobre o robo base incluem a renomeacao do robo, a correcao das referencias internas ao novo pacote e a adicao de um elemento de sinalizacao no topo, que serve como identificacao visual e demonstra a insercao de um novo link e de uma nova junta. Foram feitas tambem modificacoes voltadas a estabilidade, descritas na proxima secao.

## Estabilidade do robo

Durante os testes em simulacao, o robo base mostrou tendencia a tombar para frente ao encostar nos obstaculos, devido ao centro de massa elevado e a um apoio frontal que prendia em curvas. Para corrigir isso, foram adotadas as seguintes medidas. Foi acrescentado um lastro de massa baixa proximo ao solo, o que abaixa bastante o centro de massa e deixa o robo estavel ao encostar em obstaculos. O apoio frontal passou a ter atrito praticamente nulo, para deslizar livremente nas curvas em vez de prender. As colisoes da garra, que e usada apenas na fase futura de coleta, foram removidas para que ela nao prenda no chao. As aceleracoes do controlador foram suavizadas. Por fim, a maquina de estados conta com uma salvaguarda que detecta inclinacao excessiva pela unidade inercial e interrompe os comandos nessa situacao.

## Parametros e preparacao para a computacao evolutiva

Todos os ganhos e limiares que governam o comportamento ficam reunidos no arquivo config/mission_params.yaml e espelhados na classe MissionParams, no arquivo mission_params.py. A maquina de estados le exclusivamente desses parametros, de modo que a logica de decisao fica separada dos valores numericos.

Essa separacao e o ponto de injecao da computacao evolutiva planejada para a segunda fase. A classe de parametros oferece metodos para serializar o conjunto de valores como um vetor, que faz o papel de cromossomo, e para reconstruir os parametros a partir de um vetor, alem de fornecer os limites de cada valor evoluivel. O lancamento aceita um arquivo de parametros externo, o que permitira a um otimizador gerar um arquivo por individuo e executar um episodio de avaliacao para cada conjunto de parametros. Os topicos de estado da missao e de odometria servem como sinais para calcular o desempenho de cada individuo, por exemplo tempo ate alcancar a bandeira, distancia percorrida e ocorrencia de colisoes. Dessa forma, nenhuma alteracao na maquina de estados sera necessaria para evoluir o comportamento, apenas os valores mudam.

## Estrutura do pacote

O pacote segue a estrutura de um pacote Python do ROS 2. O arquivo package.xml descreve o pacote e suas dependencias. O arquivo setup.py define os pontos de entrada dos nos e a instalacao dos recursos. A pasta config contem os parametros da missao e a configuracao dos controladores. A pasta description contem a descricao do robo em Xacro. A pasta launch contem os arquivos de lancamento. A pasta world contem as arenas e a pasta models contem os obstaculos e demais modelos. A pasta rviz contem as configuracoes de visualizacao. A pasta com o mesmo nome do pacote contem os nos em Python e os modulos auxiliares, entre eles mission_control.py com a maquina de estados, flag_detector.py com a percepcao, navigation.py com as utilidades de LIDAR e navegacao, mission_params.py com os parametros, ground_truth_odometry.py com a odometria de referencia e robo_mapper.py com o mapeamento opcional. A pasta test contem os testes, incluindo testes do modulo de navegacao e dos parametros.

## Estado atual e limitacoes conhecidas

O robo nao tomba mais ao encostar em obstaculos e a missao completa, da deteccao ao posicionamento, funciona de forma confiavel quando o robo parte de uma regiao com espaco livre. A travessia desde a base, atravessando o aglomerado denso de cilindros que fica logo a frente, ainda nao e confiavel, pois a navegacao reativa por vezes prende o robo nesse aglomerado. Melhorar essa travessia em ambientes densos e justamente um dos objetivos da fase de computacao evolutiva, na qual os parametros de exploracao e de desvio serao ajustados de forma automatica para obter um comportamento mais eficiente e robusto.

## Documentacao da feira

O material de apresentacao em formato de poster ou de slides sera disponibilizado por meio de um link nesta secao.

## Creditos

Projeto baseado no pacote da disciplina SSC0712, sob responsabilidade do Prof. Dr. Matheus Machado dos Santos, disponivel em github.com/matheusbg8/prm_2026, sob licenca Apache 2.0.
