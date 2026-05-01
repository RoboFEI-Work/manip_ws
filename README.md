# manip_ws

Workspace ROS 2 do manipulador da RoboFEI@Work.

Este repositorio contem a descricao do robo, configuracao do MoveIt 2, bringup e interfaces para comando de braco e garra.

## Visao geral

Este workspace possui 4 pacotes principais:

- `manip_description`: URDF/Xacro, RViz e launch para visualizacao do robo.
- `manip_moveit_config`: configuracao MoveIt 2 (SRDF, kinematics, limites, controladores e launch files).
- `manip_bringup`: bringup integrado com `robot_state_publisher`, `ros2_control`, RViz e `move_group`.
- `manip_commander`: nos C++ para enviar comandos ao MoveIt (ex.: `commmander`, `test_moveit`).

## Estrutura do repositorio

- `src/manip_description/urdf/`: modelo do robo.
- `src/manip_description/launch/display.launch.xml`: visualizacao rapida da descricao.
- `src/manip_moveit_config/config/`: SRDF, joint limits, controladores e parametros MoveIt.
- `src/manip_moveit_config/launch/`: launch files do MoveIt.
- `src/manip_bringup/launch/manip_bringup.launch.xml`: bringup principal do sistema.
- `src/manip_bringup/config/ros2_controllers.yaml`: controladores do `ros2_control`.
- `src/manip_commander/src/`: implementacao dos nos de comando.

## Requisitos

- Ubuntu Linux com ROS 2 instalado.
- MoveIt 2 instalado no ambiente.
- `rosdep` configurado.

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
colcon build --packages-select manip_description manip_moveit_config manip_bringup manip_commander
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

## Comando via `commmander`

O executavel de comando se chama `commmander` (com tres letras `m`).

### Sequencia recomendada (3 terminais)

Terminal 1 (bringup):

```bash
source install/setup.bash
ros2 launch manip_bringup manip_bringup.launch.xml
```

Terminal 2 (commander):

```bash
source install/setup.bash
ros2 run manip_commander commmander
```

Terminal 3 (publicar comandos):

```bash
source install/setup.bash
```

## Topicos de comando aceitos pelo `commmander`

O no assina os topicos abaixo:

- `open_gripper` (`example_interfaces/msg/Bool`)
- `go_to_joint_target` (`example_interfaces/msg/Float64MultiArray`)
- `go_to_pose_target` (`example_interfaces/msg/Float64MultiArray`)
- `go_to_named_target` (`example_interfaces/msg/String`)

Voce pode publicar como `open_gripper` ou `/open_gripper` (idem para os demais), pois o no roda sem namespace.

### Formato esperado de cada comando

- `open_gripper`: `true` abre, `false` fecha.
- `go_to_joint_target`: vetor com exatamente 5 valores, na ordem das juntas do grupo `arm` (`joint1` a `joint5`).
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
- `pose_1`
- `pose_2`

Para o grupo `gripper`:

- `gripper_open`
- `gripper_close`
- `gripper_half_open`

## Controladores e grupos

Controladores `ros2_control` configurados:

- `arm_controller` (juntas `joint1` a `joint5`)
- `gripper_controller` (junta `joint6`)
- `joint_state_broadcaster`

Grupos MoveIt usados pelo commander:

- `arm`
- `gripper`

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


