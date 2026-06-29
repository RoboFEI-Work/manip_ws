import cv2
import numpy as np
import time
from math import degrees, atan2

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    from dynamixel_sdk import PortHandler
    from dynamixel_motor import DynamixelMotor
except ImportError:
    PortHandler = None
    DynamixelMotor = None

try:
    from varredura import VarreduraAprilTag
except ImportError:
    VarreduraAprilTag = None

try:
    import rclpy
    from apriltag_msgs.msg import AprilTagDetectionArray
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from sensor_msgs.msg import Image
    from trajectory_msgs.msg import JointTrajectoryPoint
except ImportError:
    rclpy = None
    AprilTagDetectionArray = None
    Duration = None
    FollowJointTrajectory = None
    ActionClient = None
    JointState = None
    Image = None
    JointTrajectoryPoint = None

# === CONFIGURAÇÃO SERIAL ===
DEVICENAME = 'COM12'
BAUDRATE = 1000000

# === PARÂMETROS DE CONTROLE (ajuste aqui) ===
KP_TICKS = 5
SENTIDO = 1
LIMITE_MIN, LIMITE_MAX = 0, 4095
AJUSTE_MIN_TICKS = 1
STEP_MOTORES = 5   # incremento em ticks para motores 2, 3 e 4

# Pesos manuais (não usados no novo score, mantidos para compatibilidade)
PESO_M2 = 1
PESO_M3 = 2

# === Parâmetros da AprilTag ===
TAG_SIZE_CM = 5.0   # tamanho real da tag em centímetros
FOCAL_LENGTH_PX = 600  # ajuste fino (px) - depende da câmera

# Distâncias (cm) usadas para decisões de aproximação/ajuste
ANGLE_ADJUST_DISTANCE_CM = 28  # distância abaixo da qual fazemos ajuste de ângulo com M5
GRIP_CLOSE_DISTANCE_CM = 15   # distância para iniciar fechamento da garra

# Visual / timing
FRAME_W, FRAME_H = 640, 480
SLEEP_LOOP = 0.0005

# === PARÂMETROS DE DETECÇÃO APRILTAG ===
MIN_DECISION_MARGIN = 2
DETECTOR_PARAMS = dict(
    families="tag36h11",
    nthreads=4,
    quad_decimate=0.5,
    quad_sigma=1.5,
    refine_edges=True,
    decode_sharpening=1
)


RAD_TO_DXL_POSITION = 651.088636364
MOTOR_TO_JOINT = {
    1: 'manip_joint1',
    2: 'manip_joint2',
    3: 'manip_joint3',
    4: 'manip_joint4',
    5: 'manip_joint5',
    6: 'manip_joint6',
    7: 'manip_joint7',
}
ARM_JOINTS = [MOTOR_TO_JOINT[i] for i in range(1, 6)]
GRIPPER_JOINTS = [MOTOR_TO_JOINT[i] for i in range(6, 8)]


class RosImageSource:
    def __init__(self, topic):
        if rclpy is None or Image is None:
            raise RuntimeError("rclpy/sensor_msgs are required for ROS image input")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('pick_galera_image_source')
        self.latest_image = None
        self.subscription = self.node.create_subscription(
            Image,
            topic,
            self._on_image,
            1)

    def _on_image(self, message):
        self.latest_image = self._image_to_bgr(message)

    def read(self, timeout_sec=1.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest_image is not None:
                return self.latest_image.copy()
        return None

    def close(self):
        self.node.destroy_node()

    @staticmethod
    def _image_to_bgr(message):
        image = np.frombuffer(message.data, dtype=np.uint8)
        if message.encoding == 'bgr8':
            return image.reshape((message.height, message.width, 3))
        if message.encoding == 'rgb8':
            rgb = image.reshape((message.height, message.width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if message.encoding == 'bgra8':
            bgra = image.reshape((message.height, message.width, 4))
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        if message.encoding == 'rgba8':
            rgba = image.reshape((message.height, message.width, 4))
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if message.encoding == 'mono8':
            gray = image.reshape((message.height, message.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise RuntimeError(f"Unsupported image encoding: {message.encoding}")


class ApriltagRosSource:
    def __init__(self, topic, max_detection_age_sec=0.5):
        if rclpy is None or AprilTagDetectionArray is None:
            raise RuntimeError("rclpy/apriltag_msgs are required for apriltag_ros input")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('pick_galera_apriltag_source')
        self.max_detection_age_sec = float(max_detection_age_sec)
        self.latest_by_id = {}
        self.subscription = self.node.create_subscription(
            AprilTagDetectionArray,
            topic,
            self._on_detections,
            10)

    def _on_detections(self, message):
        now = time.time()
        for detection in message.detections:
            self.latest_by_id[detection.id] = (now, detection)

    def read(self, target_tag, timeout_sec=0.1):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            detection = self._fresh_detection(target_tag)
            if detection is not None:
                return detection
        return self._fresh_detection(target_tag)

    def close(self):
        self.node.destroy_node()

    def _fresh_detection(self, target_tag):
        item = self.latest_by_id.get(target_tag)
        if item is None:
            return None
        stamp, detection = item
        if time.time() - stamp > self.max_detection_age_sec:
            return None
        if getattr(detection, "decision_margin", 0.0) < MIN_DECISION_MARGIN:
            return None
        return detection


class Ros2ControlMotorBank:
    def __init__(
        self,
        arm_action='/arm_controller/follow_joint_trajectory',
        gripper_action='/gripper_controller/follow_joint_trajectory',
        state_topic='/joint_states',
        command_duration_sec=0.25):
        required = (rclpy, FollowJointTrajectory, ActionClient, JointState,
                    JointTrajectoryPoint, Duration)
        if any(item is None for item in required):
            raise RuntimeError("rclpy/control_msgs/trajectory_msgs are required for ros2_control")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('pick_galera_ros2_control')
        self.command_duration_sec = float(command_duration_sec)
        self.positions = {}
        self._state_sub = self.node.create_subscription(
            JointState,
            state_topic,
            self._on_joint_state,
            10)
        self.arm_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            arm_action)
        self.gripper_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            gripper_action)

        self._wait_for_action(self.arm_client, arm_action)
        self._wait_for_action(self.gripper_client, gripper_action)
        self._wait_for_joint_states()

    def close(self):
        self.node.destroy_node()

    def motor(self, dxl_id):
        return Ros2ControlMotor(dxl_id, self)

    def _on_joint_state(self, message):
        for name, position in zip(message.name, message.position):
            self.positions[name] = position

    def _wait_for_action(self, client, name):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server not available: {name}")

    def _wait_for_joint_states(self):
        deadline = time.time() + 5.0
        required = set(MOTOR_TO_JOINT.values())
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if required.issubset(self.positions.keys()):
                return
        missing = sorted(required.difference(self.positions.keys()))
        raise RuntimeError(f"Missing joint states: {missing}")

    def get_ticks(self, dxl_id):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        joint = MOTOR_TO_JOINT[dxl_id]
        position = self.positions.get(joint)
        if position is None:
            return None
        direction = -1.0 if dxl_id >= 6 else 1.0
        return int(round(direction * position * RAD_TO_DXL_POSITION + 2048))

    def set_ticks(self, dxl_id, ticks):
        joint = MOTOR_TO_JOINT[dxl_id]
        direction = -1.0 if dxl_id >= 6 else 1.0
        position = direction * ((int(ticks) - 2048) / RAD_TO_DXL_POSITION)
        self.positions[joint] = position

        if dxl_id <= 5:
            joints = ARM_JOINTS
            client = self.arm_client
        else:
            joints = GRIPPER_JOINTS
            client = self.gripper_client

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [self.positions[name] for name in joints]
        point.time_from_start = Duration(
            sec=int(self.command_duration_sec),
            nanosec=int((self.command_duration_sec % 1.0) * 1_000_000_000))
        goal.trajectory.points.append(point)

        future = client.send_goal_async(goal)
        if not self._wait_future(future, timeout_sec=2.0):
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        return self._wait_future(
            result_future,
            timeout_sec=max(2.0, self.command_duration_sec + 1.5))

    def _wait_future(self, future, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if future.done():
                return True
        return False


class Ros2ControlMotor:
    def __init__(self, dxl_id, bank):
        self.id = dxl_id
        self.bank = bank

    def enable(self):
        return True

    def set_profile(self, acceleration=10, velocity=30):
        return True

    def get_pos(self):
        return self.bank.get_ticks(self.id)

    def get_present_position(self):
        return self.get_pos()

    def set_goal(self, pos):
        return self.bank.set_ticks(self.id, pos)

    def set_goal_position(self, pos):
        return self.set_goal(pos)

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

class SafeMotor:
    """Wrapper simples para evitar repetição de try/except nas chamadas de motor."""
    def __init__(self, dxl_id, port_handler):
        if DynamixelMotor is None:
            raise RuntimeError("dynamixel_motor is required when use_ros2_control=False")
        self.id = dxl_id
        self._obj = DynamixelMotor(dxl_id, port_handler)
        self._available = True
    def enable(self):
        try:
            self._obj.enable_torque()
        except Exception as e:
            self._available = False
            print(f"⚠️ SafeMotor: falha ao habilitar torque M{self.id}: {e}")
    def set_profile(self, acceleration=10, velocity=30):
        try:
            self._obj.set_profile(acceleration=acceleration, velocity=velocity)
        except Exception as e:
            print(f"⚠️ SafeMotor: falha ao set_profile M{self.id}: {e}")
    def get_pos(self):
        if not self._available:
            return None
        try:
            return self._obj.get_present_position()
        except Exception as e:
            print(f"⚠️ SafeMotor: get_pos erro M{self.id}: {e}")
            self._available = False
            return None
    def set_goal(self, pos):
        if not self._available:
            return False
        try:
            self._obj.set_goal_position(int(pos))
            return True
        except Exception as e:
            print(f"⚠️ SafeMotor: set_goal erro M{self.id}: {e}")
            self._available = False
            return False

def escolher_cantos_proximos(cantos, ref_x, ref_y):
    pts = [tuple(c) for c in cantos]
    distancias = [(p, np.linalg.norm(np.array(p) - np.array([ref_x, ref_y]))) for p in pts]
    distancias.sort(key=lambda x: x[1])
    return np.array(distancias[0][0]), np.array(distancias[1][0])

def calcular_angulo_vertical(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    ang = degrees(atan2(dy, dx))
    ang_vertical = ang - 90
    if ang_vertical > 90:
        ang_vertical -= 180
    elif ang_vertical < -90:
        ang_vertical += 180
    return ang_vertical

def move_motor_proporcional(motor: SafeMotor, pos_atual, ajuste_ticks, limite_min, limite_max):
    if pos_atual is None:
        return False
    nova = clamp(pos_atual - ajuste_ticks, limite_min, limite_max)
    return motor.set_goal(nova)


def aguardar_movimento_local(motor, posicao_alvo, tolerancia=20, timeout=10.0):
    """Versão local de espera compatível com SafeMotor/DynamixelMotor.
    Timeout padrão 10s.
    """
    inicio = time.time()
    last_pos = None
    while time.time() - inicio < timeout:
        try:
            if hasattr(motor, 'get_present_position') and callable(motor.get_present_position):
                pos_atual = motor.get_present_position()
            elif hasattr(motor, 'get_pos') and callable(motor.get_pos):
                pos_atual = motor.get_pos()
            else:
                return False
            last_pos = pos_atual
            if last_pos is None:
                time.sleep(0.005)
                continue
            if abs(last_pos - posicao_alvo) <= tolerancia:
                return True
        except Exception:
            pass
        time.sleep(0.005)
    print(f"⚠️ aguardar_movimento_local: timeout motor {getattr(motor,'id',None)}. Última posição: {last_pos}")
    return False

def main(
    target_tag=None,
    port_handler=None,
    use_ros2_control=True,
    arm_controller_action='/arm_controller/follow_joint_trajectory',
    gripper_controller_action='/gripper_controller/follow_joint_trajectory',
    joint_state_topic='/joint_states',
    use_ros_image=True,
    image_topic='/camera/camera/color/image_raw',
    apriltag_detections_topic='/detections',
    max_detection_age_sec=0.5,
    show_debug_window=False):
    pick_succeeded = False
    if target_tag is None:
        print("❌ target_tag é obrigatório para executar o fallback visual")
        return False

    # === Inicializa porta e motores ===
    precisa_fechar_porta = False
    control_bank = None
    if use_ros2_control:
        try:
            control_bank = Ros2ControlMotorBank(
                arm_action=arm_controller_action,
                gripper_action=gripper_controller_action,
                state_topic=joint_state_topic)
        except Exception as e:
            print(f"❌ Erro ao conectar no ros2_control: {e}")
            return False
    elif port_handler is None:
        if PortHandler is None:
            print("❌ dynamixel_sdk é necessário quando use_ros2_control=False")
            return False
        port_handler = PortHandler(DEVICENAME)
        if not port_handler.openPort():
            print("❌ Erro ao abrir porta serial")
            return False
        if not port_handler.setBaudRate(BAUDRATE):
            print("❌ Erro ao configurar baudrate")
            try:
                port_handler.closePort()
            except Exception:
                pass
            return False
        precisa_fechar_porta = True
    motors = {}
    for i in range(1, 8):
        if control_bank is not None:
            motors[i] = control_bank.motor(i)
        else:
            motors[i] = SafeMotor(i, port_handler)

    for m in motors.values():
        m.enable()
        m.set_profile(acceleration=10, velocity=30)

    motor1, motor2, motor3, motor4, motor5, motor6, motor7 = motors[1], motors[2], motors[3], motors[4], motors[5], motors[6], motors[7]

    pipeline = None
    config = None
    image_source = None
    detection_source = None
    try:
        detection_source = ApriltagRosSource(
            apriltag_detections_topic,
            max_detection_age_sec=max_detection_age_sec)
    except Exception as e:
        print(f"❌ Erro ao usar detecções do apriltag_ros em {apriltag_detections_topic}: {e}")
        if control_bank is not None:
            try:
                control_bank.close()
            except Exception:
                pass
        if precisa_fechar_porta:
            try:
                port_handler.closePort()
            except Exception:
                pass
        return False

    if use_ros_image:
        try:
            image_source = RosImageSource(image_topic)
        except Exception as e:
            print(f"❌ Erro ao usar topico de imagem ROS {image_topic}: {e}")
            if precisa_fechar_porta:
                try:
                    port_handler.closePort()
                except Exception:
                    pass
            return False
    else:
        if rs is None:
            print("❌ pyrealsense2 é necessário quando use_ros_image=False")
            if precisa_fechar_porta:
                try:
                    port_handler.closePort()
                except Exception:
                    pass
            return False
        # Modo manual. Nao use junto com o no realsense2_camera rodando.
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, FRAME_W, FRAME_H, rs.format.bgr8, 60)
        try:
            pipeline.start(config)
        except Exception as e:
            print(f"❌ Erro ao iniciar Realsense: {e}")
            if precisa_fechar_porta:
                try:
                    port_handler.closePort()
                except Exception:
                    pass
            return False

    w, h = FRAME_W, FRAME_H
    cell_w, cell_h = w // 3, h // 3
    ref_x = cell_w * 1 + cell_w // 2
    ref_y = cell_h * 0 + cell_h // 2

    dist_cm = None
    garra_fechando = False

    # === NOVAS VARIÁVEIS PARA CONTROLE DE VARREDURA ===
    varredura_sentido = 1
    ultimo_tempo_tag = time.time()
    varredura_ativa = False

    try:
        while True:
            if image_source is not None:
                color_image = image_source.read(timeout_sec=1.0)
                if color_image is None:
                    print(f"⚠️ Aguardando imagem no tópico {image_topic}")
                    continue
            else:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())

            for i in range(1, 3):
                cv2.line(color_image, (i * cell_w, 0), (i * cell_w, h), (100, 100, 100), 1)
                cv2.line(color_image, (0, i * cell_h), (w, i * cell_h), (100, 100, 100), 1)
            cv2.circle(color_image, (ref_x, ref_y), 5, (255, 0, 0), -1)

            # === DETECÇÃO APRILTAG ===
            tag = detection_source.read(target_tag, timeout_sec=0.05)

            status_msg = "🔍 Procurando AprilTag..."
            info_sup = info_inf = info_motor = info_m4 = ""

            if tag is not None:
                ultimo_tempo_tag = time.time()
                varredura_ativa = False

                cantos = np.array([[corner.x, corner.y] for corner in tag.corners])
                p_sup1, p_sup2 = cantos[0], cantos[1]
                p_inf1, p_inf2 = cantos[3], cantos[2]
                len_sup = np.linalg.norm(p_sup1 - p_sup2)
                len_inf = np.linalg.norm(p_inf1 - p_inf2)
                taxa_sup = (len_sup / w) * 100
                taxa_inf = (len_inf / w) * 100
                info_sup = f"Reta Sup: {len_sup:.1f}px ({taxa_sup:.1f}%)"
                info_inf = f"Reta Inf: {len_inf:.1f}px ({taxa_inf:.1f}%)"

                cv2.line(color_image, tuple(p_sup1.astype(int)), tuple(p_sup2.astype(int)), (255, 255, 0), 2)
                cv2.line(color_image, tuple(p_inf1.astype(int)), tuple(p_inf2.astype(int)), (0, 255, 255), 2)

                tamanho_px = (len_sup + len_inf) / 2.0
                if tamanho_px > 0:
                    dist_cm = (TAG_SIZE_CM * FOCAL_LENGTH_PX) / tamanho_px
                    if show_debug_window:
                        cv2.putText(color_image, f"Distancia Estimada: {dist_cm:.1f} cm",
                                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                centro_tag = np.array([tag.centre.x, tag.centre.y])
                x_centro, y_centro = float(centro_tag[0]), float(centro_tag[1])

                pos_cache = {}
                for mid, mobj in motors.items():
                    try:
                        if mobj is None:
                            pos_cache[mid] = None
                        elif hasattr(mobj, 'get_present_position') and callable(mobj.get_present_position):
                            pos_cache[mid] = mobj.get_present_position()
                        elif hasattr(mobj, 'get_pos') and callable(mobj.get_pos):
                            pos_cache[mid] = mobj.get_pos()
                        else:
                            pos_cache[mid] = None
                    except Exception:
                        pos_cache[mid] = None

                try:
                    pos_m1 = pos_cache[1]
                    if pos_m1 is not None:
                        step_m1 = 50 if x_centro > ref_x else -50
                        step_m1 = step_m1 if abs(x_centro - ref_x) > 10 else 0
                        nova_m1 = clamp(pos_m1 + step_m1, LIMITE_MIN, LIMITE_MAX)
                        motor1.set_goal(nova_m1)
                except Exception as e:
                    info_motor = f"erro M1: {e}"

                try:
                    pos_m4 = pos_cache[4]
                    if pos_m4 is not None:
                        step_m4 = 15 if y_centro > ref_y else -15
                        step_m4 = step_m4 if y_centro - ref_y > 10 else 0
                        nova_m4 = clamp(pos_m4 + step_m4, LIMITE_MIN, LIMITE_MAX)
                        motor4.set_goal(nova_m4)
                        info_m4 = f"M4 centro_x={x_centro:.1f} | ref_x={ref_x} | Mov: {'+10' if step_m4 > 0 else '-10'} ticks"
                except Exception as e:
                    info_m4 = f"erro M4:{e}"

                try:
                    p1, p2 = escolher_cantos_proximos(cantos, ref_x, ref_y)
                    ang_vertical = calcular_angulo_vertical(p1, p2)
                    status_msg = f"Ângulo vertical={ang_vertical:.2f}° (alvo 0°)"

                    if dist_cm is not None and dist_cm < ANGLE_ADJUST_DISTANCE_CM and abs(ang_vertical) < 85:
                        ajuste_ticks = int(SENTIDO * 2 * ang_vertical)
                        if abs(ajuste_ticks) < AJUSTE_MIN_TICKS:
                            ajuste_ticks = 0
                        if ajuste_ticks != 0:
                            pos_m5 = pos_cache[5]
                            if pos_m5 is not None:
                                nova_pos = clamp(pos_m5 - ajuste_ticks, LIMITE_MIN, LIMITE_MAX)
                                motor5.set_goal(nova_pos)
                                status_msg += f" | M5: {pos_m5}→{nova_pos} (Δ={-ajuste_ticks})"
                except Exception as e:
                    status_msg += f" | erro M5:{e}"

                try:
                    pos2 = pos_cache[2]
                    pos3 = pos_cache[3]
                    peso2 = pos2 * PESO_M2 if pos2 is not None else 0
                    peso3 = pos3 * PESO_M3 if pos3 is not None else 0
                    if pos2 is not None and pos3 is not None:
                        if len_inf >= len_sup:
                            if peso3 <= peso2:
                                motor2.set_goal(clamp(pos2 - STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                            elif peso2 < peso3:
                                motor3.set_goal(clamp(pos3 - STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                        elif len_sup > len_inf:
                            if peso3 < peso2:
                                motor2.set_goal(clamp(pos2 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                            elif peso2 <= peso3:
                                motor3.set_goal(clamp(pos3 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                    else:
                        if pos2 is not None:
                            motor2.set_goal(clamp(pos2 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                        if pos3 is not None:
                            motor3.set_goal(clamp(pos3 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                except Exception as e:
                    info_motor = f"erro seleção M2/M3: {e}"

                try:
                    p1i, p2i = (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1]))
                    cv2.line(color_image, p1i, p2i, (0, 255, 0), 2)
                    cv2.circle(color_image, (int(centro_tag[0]), int(centro_tag[1])), 6, (0, 0, 255), -1)
                except Exception:
                    pass

                if dist_cm is not None and dist_cm <= GRIP_CLOSE_DISTANCE_CM and not garra_fechando:
                    garra_fechando = True
                    try:
                        pos6 = pos_cache[6]
                        pos7 = pos_cache[7]
                        if pos6 is not None and pos7 is not None:
                            nova_pos6 = clamp(pos6 - 600, LIMITE_MIN, LIMITE_MAX)
                            nova_pos7 = clamp(pos7 + 600, LIMITE_MIN, LIMITE_MAX)
                            motor6.set_goal(nova_pos6)
                            motor7.set_goal(nova_pos7)
                            aguardar_movimento_local(motor7, nova_pos7, tolerancia=20, timeout=10.0)
                            if show_debug_window:
                                cv2.putText(color_image, "GARRA FECHANDO!", (10, 175),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            pick_succeeded = True
                            break
                        print("⚠️ Não foi possível ler posição de M6/M7 para fechar garra.")
                    except Exception as e:
                        print(f"⚠️ Erro ao mover garra: {e}")
            else:
                tempo_sem_tag = time.time() - ultimo_tempo_tag
                if tempo_sem_tag >= 10.0 and not varredura_ativa:
                    status_msg = (
                        "🔄 Sem detecção do apriltag_ros. "
                        "Aguardando /detections..."
                    )
                    print(status_msg)
                    varredura_ativa = True

            if show_debug_window:
                cv2.putText(color_image, status_msg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                if info_sup: cv2.putText(color_image, info_sup, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                if info_inf: cv2.putText(color_image, info_inf, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                if info_motor: cv2.putText(color_image, info_motor, (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                if info_m4: cv2.putText(color_image, info_m4, (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

                cv2.imshow("Controle AprilTag + Motores", color_image)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    print("⏹️ Usuário requisitou saída (ESC).")
                    break

            time.sleep(SLEEP_LOOP)

    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if image_source is not None:
            try:
                image_source.close()
            except Exception:
                pass
        if detection_source is not None:
            try:
                detection_source.close()
            except Exception:
                pass
        if control_bank is not None:
            try:
                control_bank.close()
            except Exception:
                pass
        if precisa_fechar_porta:
            try:
                port_handler.closePort()
            except Exception:
                pass
        if show_debug_window:
            cv2.destroyAllWindows()
        print("✔️ Finalizado")
    return pick_succeeded

if __name__ == "__main__":
    main(22, use_ros2_control=False, use_ros_image=False, show_debug_window=True)
    
