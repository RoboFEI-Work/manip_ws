# manip_ws

Workspace ROS 2 do manipulador da RoboFEI@Work.

Este repositorio contem a descricao do robo, configuracao do MoveIt 2, bringup, interfaces para comando de braco e garra, e a interface de hardware para os motores Dynamixel XM540.

## Visao geral

Este workspace possui 8 pacotes principais:

- `manip_description`: URDF/Xacro, RViz e launch para visualizacao do robo.
- `manip_moveit_config`: configuracao MoveIt 2 (SRDF, kinematics, limites, controladores e launch files).
- `manip_bringup`: bringup integrado com `robot_state_publisher`, `ros2_control`, RViz e `move_group`.
- `manip_commander`: nos C++ para enviar comandos ao MoveIt (ex.: `commmander`, `test_moveit`).
- `manip_bt`: executor de Behavior Tree e tradutor YAML para sequencia de acoes (`pick`/`place`).
- `manip_hardware`: interface de hardware `ros2_control` para os 7 motores Dynamixel XM540 (joints 1-5 do braco, joints 6-7 da garra).
- `mtc_tutorial`: nos de manipulacao com MoveIt Task Constructor e action servers de pick/place.
- `my_robot_msgs`: interfaces ROS 2 customizadas (actions `PickTag` e `PlaceTag`).

## Estrutura do repositorio

- `src/manip_description/urdf/`: modelo do robo.
- `src/manip_description/launch/display.launch.xml`: visualizacao rapida da descricao.
- `src/manip_moveit_config/config/`: SRDF, joint limits, controladores e parametros MoveIt.
- `src/manip_moveit_config/launch/`: launch files do MoveIt.
- `src/manip_bringup/launch/manip_bringup.launch.xml`: bringup principal do sistema.
- `src/manip_bringup/config/ros2_controllers.yaml`: controladores do `ros2_control`.
- `src/manip_bt/behavior_tree_manip/`: arvores XML e YAMLs de exemplo para BT.
- `src/manip_bt/src/bt_executor.cpp`: executa a arvore BT registrada no pacote.
- `src/manip_bt/src/competition_yaml_translator.cpp`: traduz YAML de tarefa para lista de acoes.
- `src/manip_commander/src/`: implementacao dos nos de comando.
- `src/manip_hardware/include/manip_hardware/`: headers do driver XM540 e da interface de hardware.
- `src/manip_hardware/src/arm_hardware_interface.cpp`: implementacao da interface de hardware.
- `src/mtc_tutorial/src/mtc_pick_action_node.cpp`: action server `/pick_tag`.
- `src/mtc_tutorial/src/mtc_place_action_node.cpp`: action server `/place_tag`.
- `src/my_robot_msgs/action/PickTag.action`: definicao da action de pick por tag.
- `src/my_robot_msgs/action/PlaceTag.action`: definicao da action de place por tag.

## Requisitos

- Ubuntu Linux com ROS 2 instalado.
- MoveIt 2 instalado no ambiente.
- `rosdep` configurado.
- `dynamixel_sdk` instalado.

## Setup e build

No root do workspace:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Build apenas dos pacotes do manip:

```bash
colcon build --packages-select manip_description manip_moveit_config manip_bringup manip_commander manip_bt manip_hardware my_robot_msgs mtc_tutorial
source install/setup.bash
```

## Fluxos de execucao

### 1) Visualizar apenas o robo (URDF)

```bash
ros2 launch manip_description display.launch.xml
```

### 2) Subir demo padrao do MoveIt 2

```bash
ros2 launch manip_moveit_config demo.launch.py
```

### 3) Subir bringup integrado (recomendado para testes)

```bash
ros2 launch manip_bringup manip_bringup.launch.xml
```

Esse launch sobe os componentes necessarios para comando do braco e garra via MoveIt.

Por padrao, esse bringup tambem sobe os action servers de pick/place (`mtc_pick_action_node` e `mtc_place_action_node`).

Por padrao, ao subir o bringup com `launch_pick_action:=true`, o arquivo `~/manip_ws/container_states.yaml` e resetado para deixar todos os containers vazios. Para preservar o estado anterior dos containers:

```bash
ros2 launch manip_bringup manip_bringup.launch.xml reset_container_states:=false
```

Argumentos disponiveis no bringup:

- `use_mock_components` (default: `true`)
- `launch_realsense` (default: `false`)
- `launch_apriltag` (default: `false`)
- `launch_pick_action` (default: `true`)
- `launch_place_action` (default: `true`)
- `reset_container_states` (default: `true`)
- `apriltag_params_file` (default: `$(env HOME)/manip_ws/src/apriltag_ros/cfg/tags_36h11.yaml`)

Para usar o hardware real (motores XM540), passe o argumento:

```bash
ros2 launch manip_bringup manip_bringup.launch.xml use_mock_components:=false
```

Para subir camera + detector AprilTag no mesmo bringup:

```bash
ros2 launch manip_bringup manip_bringup.launch.xml use_mock_components:=false\
	launch_realsense:=true \
	launch_apriltag:=true
```

Se o arquivo de parametros da tag estiver em outro caminho:

```bash
ros2 launch manip_bringup manip_bringup.launch.xml \
	launch_realsense:=true \
	launch_apriltag:=true \
	apriltag_params_file:=/caminho/para/tags_36h11.yaml
```

Observacao: o repositorio local pode nao conter os fontes de `apriltag_ros` e `realsense2_camera`; nesse caso, use os pacotes instalados no ROS 2.

## Actions de pick/place por tag

Action definida em `my_robot_msgs/action/PickTag.action`:

- Goal: `tag_frame`
- Result: `success`, `message`
- Feedback: `current_stage`

Enviar goal manualmente:

```bash
ros2 action send_goal /pick_tag my_robot_msgs/action/PickTag "{tag_frame: tag_M30_nut}" --feedback
```

Action definida em `my_robot_msgs/action/PlaceTag.action`:

- Goal: `tag_frame`, `table_pose`
- Result: `success`, `message`
- Feedback: `current_stage`

Enviar goal manualmente:

```bash
ros2 action send_goal /place_tag my_robot_msgs/action/PlaceTag "{tag_frame: tag_M30_nut, table_pose: Mesa15}" --feedback
```

## Behavior Tree (`manip_bt`)

Executaveis disponiveis no pacote:

- `ros2 run manip_bt manip_bt_executor`
- `ros2 run manip_bt bt_yaml_executor`
- `ros2 run manip_bt competition_yaml_translator`
- `ros2 run manip_bt task_planner`

Nos BT customizados registrados no executor:

- `GoToNamedPose` (porta: `pose_name`)
- `PickTag` (porta: `tag_frame`)
- `PlaceTag` (portas: `tag_frame`, `table_pose`)

### Rodar o executor de BT

1. Suba o bringup (com actions habilitadas):

```bash
source install/setup.bash
ros2 launch manip_bringup manip_bringup.launch.xml launch_pick_action:=true launch_place_action:=true
```

2. Em outro terminal, rode o executor:

```bash
source install/setup.bash
ros2 run manip_bt manip_bt_executor
```

Observacao: atualmente o executor usa caminho fixo para a arvore `behavior_tree_manip/test2.xml` dentro do share do pacote `manip_bt`.

### Rodar BT a partir de YAML de acoes

O `bt_yaml_executor` le um YAML com a lista de acoes e monta a Behavior Tree em memoria. Ele nao precisa de um arquivo XML como `test1.xml` ou `test2.xml`: o XML equivalente e gerado temporariamente pelo proprio executor.

Durante essa montagem, os valores do YAML sao salvos no blackboard da BT com chaves internas como `action_0_tag_frame` e `action_1_table_pose`. Os nos `PickTag` e `PlaceTag` recebem os dados por portas BT usando referencias no formato `{chave}`.

Uso:

```bash
source install/setup.bash
ros2 run manip_bt bt_yaml_executor <actions_yaml>
```

Exemplo:

```bash
source install/setup.bash
ros2 run manip_bt bt_yaml_executor att1.yaml
```

Se nenhum arquivo for passado, o executor tenta usar `att1.yaml`. O caminho pode ser absoluto, relativo ao workspace, ou um arquivo dentro de `src/manip_bt/behavior_tree_manip/` instalado no share do pacote.

Schema esperado do YAML:

- `actions[].kind`: `home`, `goto`, `pick` ou `place`
- `actions[].mesa`: mesa/named target, obrigatoria em `goto`
- `actions[].tag_frame`: frame da tag usado no pick/place
- `actions[].table_pose`: pose de destino, obrigatoria em `place`; pode ser uma mesa (`Mesa15`) ou uma pose de container (`ct10`)
- `actions[].ws`: workspace de destino, opcional em `place`

### Traduzir YAML de competicao para lista de acoes

Uso:

```bash
ros2 run manip_bt competition_yaml_translator <competition_yaml> <output_yaml> [apriltag_yaml] [ws_table_map_yaml]
```

Exemplo com arquivo do pacote:

```bash
source install/setup.bash
ros2 run manip_bt competition_yaml_translator competition_sample_btt1.yaml btt1_actions.yaml
```

Exemplo com caminho absoluto:

```bash
source install/setup.bash
ros2 run manip_bt competition_yaml_translator \
	src/manip_bt/behavior_tree_manip/ATT1.yaml \
	att1_actions.yaml \
	src/apriltag_ros/cfg/tags_36h11.yaml \
	src/manip_bt/behavior_tree_manip/ws_table_mapping.yaml
```

Mapeamento WS -> mesa usado pelo tradutor (facil de ajustar):

- Arquivo padrao: `src/manip_bt/behavior_tree_manip/ws_table_mapping.yaml`
- Chave esperada no YAML: `ws_to_table_pose`
- Exemplo de entrada: `WS_3: Mesa10`

Saida gerada (schema minimo):

- `actions[].kind`: `pick` ou `place`
- `actions[].tag_frame`: frame da tag resolvido via arquivo AprilTag
- `actions[].ws`: workspace de destino (somente `place`)
- `actions[].table_pose`: pose de destino (somente `place`); usa o mapeamento WS -> mesa quando o objeto vai para uma area, ou `ct10`, `ct16`, etc. quando a tarefa tem constraints de container

Quando o YAML de competicao possui a chave `containers` e constraints no `finish_state`, por exemplo `O2,O4 must be inside 10 (BLUE)`, o tradutor direciona esses objetos para a pose `ct10`. Ou seja, o nome da pose de destino usa o id real do container: container id `16` vira `ct16`.

### Planejar com navegacao

O `task_planner` usa o mesmo formato de entrada do tradutor, mas ordena as acoes considerando navegacao entre mesas e capacidade maxima de 3 tags no robo. A saida inclui acoes:

- `kind: goto` com `mesa`
- `kind: home` antes de voltar para navegacao depois que a tarefa ja comecou
- `kind: pick`
- `kind: place`

Uso:

```bash
ros2 run manip_bt task_planner <competition_yaml> <output_yaml> [apriltag_yaml] [ws_table_map_yaml]
```

## Comando via `commmander`

O executavel de comando se chama `commmander` (com tres letras `m`).

Atualmente, o `commmander` ja e iniciado dentro de `manip_bringup.launch.xml`.

### Sequencia recomendada (2 terminais)

Terminal 1 (bringup):

```bash
source install/setup.bash
ros2 launch manip_bringup manip_bringup.launch.xml
```

Terminal 2 (publicar comandos):

```bash
source install/setup.bash
```

Se voce iniciar o `commmander` manualmente enquanto ele ja estiver no bringup, podera haver dois nos consumindo os mesmos topicos.

## Topicos de comando aceitos pelo `commmander`

O no assina os topicos abaixo:

- `open_gripper` (`example_interfaces/msg/Bool`)
- `go_to_joint_target` (`example_interfaces/msg/Float64MultiArray`)
- `go_to_pose_target` (`example_interfaces/msg/Float64MultiArray`)
- `go_to_named_target` (`example_interfaces/msg/String`)

Voce pode publicar como `open_gripper` ou `/open_gripper` (idem para os demais), pois o no roda sem namespace.

### Formato esperado de cada comando

- `open_gripper`: `true` abre, `false` fecha.
- `go_to_joint_target`: vetor com exatamente 5 valores, na ordem das juntas do grupo `arm` (`manip_joint1` a `manip_joint5`).
- `go_to_pose_target`: vetor com exatamente 6 valores na ordem `x, y, z, roll, yaw, pitch` (frame `base_link`).
- `go_to_named_target`: string com nome de estado existente no SRDF.

### Exemplos prontos (`ros2 topic pub`)

Abrir garra:

```bash
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"
```

Fechar garra:

```bash
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: false}"
```

Mover braco por juntas (5 DOF):

```bash
ros2 topic pub --once /go_to_joint_target example_interfaces/msg/Float64MultiArray "{data: [0.0, -1.57, 1.57, 0.0, 0.0]}"
```

Mover braco por pose (x, y, z, roll, yaw, pitch):

```bash
ros2 topic pub --once /go_to_pose_target example_interfaces/msg/Float64MultiArray "{data: [0.40, 0.00, -0.10, 0.0, 3.14, 0.0]}"
```

Mover braco por alvo nomeado:

```bash
ros2 topic pub --once /go_to_named_target example_interfaces/msg/String "{data: 'home'}"
```

## Alvos nomeados disponiveis no SRDF

Para o grupo `arm`:

- `home`
- `pirocao`
- `pose_1`
- `pose_2`
- `pegar_obj`
- `tag_direita`
- `tag_esquerda`
- `pre_container`
- `container1`
- `container2`
- `container3`
- `Mesa0`
- `Mesa5`
- `Mesa10`
- `Mesa10.1`
- `Mesa10.2`
- `Mesa15`
- `Mesa15.1`
- `Mesa15.2`

Para o grupo `gripper`:

- `gripper_open`
- `gripper_close`
- `gripper_half_open`

## Controladores e grupos

Controladores `ros2_control` configurados:

- `arm_controller` (juntas `manip_joint1` a `manip_joint5`)
- `gripper_controller` (juntas `manip_joint6` e `manip_joint7`)
- `joint_state_broadcaster`

Grupos MoveIt usados pelo commander:

- `arm`
- `gripper`

## Cinematica inversa

O grupo `arm` esta configurado para usar o plugin `pick_ik/PickIkPlugin` (arquivo `src/manip_moveit_config/config/kinematics.yaml`).

Resumo da configuracao atual:

- Solver: `PickIkPlugin`
- Modo: `global`
- Timeout: `0.5`
- Tentativas: `15`

Observacao: com essa configuracao, o comportamento de IK pode diferir do KDL padrao em convergencia e escolha de solucoes para `go_to_pose_target`.

## Hardware — Dynamixel XM540

O pacote `manip_hardware` implementa a interface `ros2_control` para 7 motores Dynamixel XM540 via Protocol 2.0:

| Joint   | Motor ID | Funcao         |
|---------|----------|----------------|
| manip_joint1  | 1        | Braco          |
| manip_joint2  | 2        | Braco          |
| manip_joint3  | 3        | Braco          |
| manip_joint4  | 4        | Braco          |
| manip_joint5  | 5        | Braco          |
| manip_joint6  | 6        | Garra          |
| manip_joint7  | 7        | Garra          |

Os IDs e a porta serial sao configurados no URDF/Xacro (`manip.ros2_control.xacro`).

## Comandos uteis de verificacao

Ver nos ativos:

```bash
ros2 node list
```

Ver topicos disponiveis:

```bash
ros2 topic list
```

Ver tipo e detalhes de um topico:

```bash
ros2 topic info /go_to_joint_target -v
```

Ver controladores:

```bash
ros2 control list_controllers
```

Ver estado das juntas:

```bash
ros2 topic echo /joint_states
```

## Deteccao de AprilTag e TF no RViz

### Subir camera RealSense (D455)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_d455_pointcloud_launch.py
```

### Subir detector AprilTag (com remap correto)

Use imagem RGB no remap de `image_rect`. Nao use o topico de pointcloud nesse remap.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args \
	-r image_rect:=/camera/camera/color/image_raw \
	-r camera_info:=/camera/camera/color/camera_info \
	--params-file ~/manip_ws/src/apriltag_ros/cfg/tags_36h11.yaml
```

Exemplo incorreto (nao usar):

```bash
-r image_rect:=/camera/camera/depth/color/points
```

Motivo: `image_rect` espera `sensor_msgs/Image`, mas `/camera/camera/depth/color/points` e `sensor_msgs/PointCloud2`.

### Verificar deteccoes

```bash
ros2 topic echo /detections
```

### Ver TF da tag no RViz

1. Abra o RViz (`rviz2`).
2. Defina `Fixed Frame` como `camera_link` ou `camera_color_optical_frame`.
3. Adicione o display `TF`.
4. Aponte a tag para a camera.

Se detectar, os frames configurados em `tags_36h11.yaml` (campo `frames`) aparecem na arvore TF.

### Pointcloud estranho/escuro no RViz

Para visualizar melhor `/camera/camera/depth/color/points`:

1. Adicione display `PointCloud2`.
2. Topic: `/camera/camera/depth/color/points`.
3. `Color Transformer`: `RGB8` (ou `Intensity` se precisar).
4. `Size (Pixels)`: 2 ou 3.
5. `Decay Time`: 0.

Comandos uteis de diagnostico:

```bash
ros2 topic type /camera/camera/depth/color/points
ros2 topic type /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/color/points
```

Se precisar, rode a camera com alinhamento de profundidade e pointcloud habilitados:

```bash
ros2 launch realsense2_camera rs_launch.py \
	camera_name:=camera \
	align_depth.enable:=true \
	pointcloud.enable:=true \
	enable_color:=true \
	enable_depth:=true
```

## Troubleshooting

Erro de pacote nao encontrado:

- Garanta que `colcon build` foi executado sem erro.
- Rode `source install/setup.bash` em todo terminal usado.

Comando nao movimenta o robo:

- Verifique se `commmander` esta rodando.
- Verifique se `move_group` esta ativo.
- Verifique controladores com `ros2 control list_controllers`.

Mensagem rejeitada no `go_to_joint_target`:

- Envie exatamente 5 valores.

Mensagem rejeitada no `go_to_pose_target`:

- Envie exatamente 6 valores (`x, y, z, roll, yaw, pitch`).

Sem movimento da garra:

- Teste `open_gripper` com `true` e `false`.
- Confirme se os estados `gripper_open` e `gripper_close` existem no SRDF.

Hardware real nao responde:

- Verifique se a porta serial esta correta em `manip.ros2_control.xacro` (padrao `/dev/ttyUSB0`).
- Verifique se os IDs dos motores batem com o configurado.
- Confirme que o baudrate dos motores esta em 57600.

## Launch files disponiveis

Em `manip_moveit_config/launch/`:

- `demo.launch.py`
- `move_group.launch.py`
- `moveit_rviz.launch.py`
- `rsp.launch.py`
- `setup_assistant.launch.py`
- `spawn_controllers.launch.py`
- `static_virtual_joint_tfs.launch.py`
- `warehouse_db.launch.py`

## Executaveis do pacote `manip_commander`

- `ros2 run manip_commander commmander`
- `ros2 run manip_commander test_moveit`

## Observacoes de manutencao

- Alguns `package.xml` ainda possuem campos `TODO` (descricao/licenca) e podem ser revisados.

## Licenca

- `manip_moveit_config`: BSD-3-Clause.
- Demais pacotes: revisar campo de licenca em cada `package.xml`.

## COMANDO APRILTAG

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
	-r image_rect:=/camera/camera/color/image_raw \
	-r camera_info:=/camera/camera/color/camera_info \
	--params-file ~/manip_ws/src/apriltag_ros/cfg/tags_36h11.yaml
```

## COMANDO CAMERA

```bash
ros2 launch realsense2_camera rs_launch.py \
	camera_name:=camera \
	align_depth.enable:=true \
	pointcloud.enable:=true \
	enable_color:=true \
	enable_depth:=true
```
