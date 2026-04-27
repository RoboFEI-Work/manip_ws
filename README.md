# manip_ws

Pacote de configuracao do MoveIt 2 para o robo `kevin_levin_fantasmatico`.

Este repositorio contem o pacote ROS 2 `manip_moveit`, gerado com o MoveIt Setup Assistant e ajustado para uso no workspace da RoboFEI.

## Conteudo

- `config/`: SRDF, limites de juntas, kinematics, controladores e configuracoes do RViz.
- `launch/`: arquivos de launch para subir o MoveIt, RViz e componentes auxiliares.
- `package.xml`: dependencias ROS 2 do pacote.
- `CMakeLists.txt`: instalacao dos arquivos `launch` e `config` via `ament_cmake`.

## Requisitos

- Ubuntu Linux com ROS 2 instalado.
- MoveIt 2 instalado no ambiente.
- Dependencias do pacote `manip_moveit` resolvidas.
- Pacote `manip` disponivel no mesmo workspace (dependencia declarada em `package.xml`).

## Como construir

No root do workspace:

```bash
colcon build --packages-select manip_moveit
source install/setup.bash
```

Se for a primeira vez no ambiente, instale dependencias antes do build:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Como executar

### Subir demo completa do MoveIt (recomendado)

```bash
ros2 launch manip_moveit demo.launch.py
```

### Subir apenas o `move_group`

```bash
ros2 launch manip_moveit move_group.launch.py
```

### Subir RViz com configuracao do MoveIt

```bash
ros2 launch manip_moveit moveit_rviz.launch.py
```

## Launch files disponiveis

- `demo.launch.py`
- `move_group.launch.py`
- `moveit_rviz.launch.py`
- `rsp.launch.py`
- `spawn_controllers.launch.py`
- `static_virtual_joint_tfs.launch.py`
- `warehouse_db.launch.py`

## Estrutura do robo

Os arquivos principais de modelo e planejamento estao em:

- `config/kevin_levin_fantasmatico.urdf.xacro`
- `config/kevin_levin_fantasmatico.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/moveit_controllers.yaml`

## Solucao de problemas

- Erro de pacote nao encontrado:
  Confirme que o workspace foi compilado e o `install/setup.bash` foi carregado.
- Erro relacionado ao pacote `manip`:
  Garanta que o pacote `manip` esta presente em `src/` e compila no mesmo workspace.
- RViz abre sem modelo:
  Verifique se `robot_state_publisher` e `move_group` estao ativos.

## Licenca

Este pacote usa licenca BSD-3-Clause (conforme `package.xml`).
