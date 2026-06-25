import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector
from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor
import time
from math import degrees, atan2

# === CONFIGURAÇÃO SERIAL ===
DEVICENAME = 'COM12'
BAUDRATE = 1000000

# === PARÂMETROS DE CONTROLE (ajuste aqui) ===
KP_TICKS = 1
SENTIDO = 1
LIMITE_MIN, LIMITE_MAX = 0, 4095
AJUSTE_MIN_TICKS = 1
STEP_MOTORES = 1    # incremento em ticks para motores 2, 3 e 4

# Pesos manuais (não usados no novo score, mantidos para compatibilidade)
PESO_M2 = 1
PESO_M3 = 2

# === Parâmetros da AprilTag ===
TAG_SIZE_CM = 4.0   # tamanho real da tag em centímetros
FOCAL_LENGTH_PX = 600  # ajuste fino (px) - depende da câmera

# Visual / timing
FRAME_W, FRAME_H = 640, 480
SLEEP_LOOP = 0.005

# === PARÂMETROS DE DETECÇÃO APRILTAG ===
# Reduzindo o nível de confiança (threshold) da detecção:
# - MIN_DECISION_MARGIN define o mínimo de confiança aceito (quanto menor, mais permissivo)
# - quad_decimate < 1 melhora sensibilidade mas consome mais CPU
MIN_DECISION_MARGIN = 2 # ajuste principal — reduza para detectar tags mais difíceis
DETECTOR_PARAMS = dict(
    families="tag36h11",
    nthreads=4,
    quad_decimate=0.5,
    quad_sigma=1.5,
    refine_edges=True,
    decode_sharpening=1
)

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

class SafeMotor:
    """Wrapper simples para evitar repetição de try/except nas chamadas de motor."""
    def __init__(self, dxl_id, port_handler):
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

def main(target_tag=None, port_handler=None):
    # === Inicializa porta e motores ===
    # If no port_handler was provided, open one and remember we must close it.
    precisa_fechar_porta = False
    if port_handler is None:
        port_handler = PortHandler(DEVICENAME)
        if not port_handler.openPort():
            print("❌ Erro ao abrir porta serial")
            return
        if not port_handler.setBaudRate(BAUDRATE):
            print("❌ Erro ao configurar baudrate")
            try:
                port_handler.closePort()
            except Exception:
                pass
            return
        precisa_fechar_porta = True
    motors = {}
    for i in range(1, 8):
        motors[i] = SafeMotor(i, port_handler)

    for m in motors.values():
        m.enable()
        m.set_profile(acceleration=10, velocity=30)

    motor1, motor2, motor3, motor4, motor5, motor6, motor7 = motors[1], motors[2], motors[3], motors[4], motors[5], motors[6], motors[7]

    # === Inicializa câmera Realsense ===
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_W, FRAME_H, rs.format.bgr8, 60)
    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"❌ Erro ao iniciar Realsense: {e}")
        if precisa_fechar_porta:
            try:
                port_handler.closePort()
            except Exception:
                pass
        return

    w, h = FRAME_W, FRAME_H
    cell_w, cell_h = w // 3, h // 3
    ref_x = cell_w * 1 + cell_w // 2
    ref_y = cell_h * 2 + cell_h // 2

    # === Detector com parâmetros ajustáveis ===
    detector = Detector(**DETECTOR_PARAMS)

    dist_cm = None
    garra_fechando = False

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            for i in range(1, 3):
                cv2.line(color_image, (i * cell_w, 0), (i * cell_w, h), (100, 100, 100), 1)
                cv2.line(color_image, (0, i * cell_h), (w, i * cell_h), (100, 100, 100), 1)
            cv2.circle(color_image, (ref_x, ref_y), 5, (255, 0, 0), -1)

            # === DETECÇÃO COM FILTRO DE CONFIANÇA REDUZIDO ===
            raw_tags = detector.detect(gray)
            tags = [t for t in raw_tags if getattr(t, "decision_margin", 0.0) >= MIN_DECISION_MARGIN]

            status_msg = "🔍 Procurando AprilTag..."
            info_sup = info_inf = info_motor = info_m4 = ""

            if len(tags) > 0:
                tag = None
                for t in tags:
                    if t.tag_id == target_tag:
                        tag = t
                        break

                if tag is None:
                    cv2.putText(color_image, f"Tag ID {target_tag} não encontrada",
                                (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cantos = np.array(tag.corners)

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
                    cv2.putText(color_image, f"Distancia Estimada: {dist_cm:.1f} cm",
                                (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                centro_tag = np.mean(cantos, axis=0)
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

                # === Controle Motor 1 ===
                try:
                    pos_m1 = pos_cache[1]
                    if pos_m1 is not None:
                        step_m1 = 10 if x_centro > ref_x else -10
                        step_m1 = step_m1 if abs(x_centro - ref_x) > 10 else 0
                        nova_m1 = clamp(pos_m1 + step_m1, LIMITE_MIN, LIMITE_MAX)
                        motor1.set_goal(nova_m1)
                except Exception as e:
                    info_motor = f"erro M1: {e}"

                # === Controle Motor 4 ===
                try:
                    pos_m4 = pos_cache[4]
                    if pos_m4 is not None:
                        step_m4 = 15 if y_centro > ref_y else -15
                        step_m4 = step_m4 if y_centro - ref_y > 10 else 0
                        nova_m4 = clamp(pos_m4 + step_m4, LIMITE_MIN, LIMITE_MAX)
                        motor4.set_goal(nova_m4)
                        info_m4 = f"M4 centro_x={x_centro:.1f} | ref_x={ref_x} | Mov: {'+10' if step_m4>0 else '-10'} ticks"
                except Exception as e:
                    info_m4 = f"erro M4:{e}"

                # === Controle ângulo motor 5 ===
                try:
                    p1, p2 = escolher_cantos_proximos(cantos, ref_x, ref_y)
                    ang_vertical = calcular_angulo_vertical(p1, p2)
                    status_msg = f"Ângulo vertical={ang_vertical:.2f}° (alvo 0°)"

                    if dist_cm is not None and dist_cm < 35:
                        if abs(ang_vertical) < 85:
                            ajuste_ticks = int(SENTIDO * KP_TICKS * ang_vertical)
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

                # === Controle motor 2 e 3 ===
                try:
                    pos2 = pos_cache[2]
                    pos3 = pos_cache[3]
                    peso2 = pos2 * PESO_M2 if pos2 is not None else 0
                    peso3 = pos3 * PESO_M3 if pos3 is not None else 0
                    if pos2 is not None and pos3 is not None:
                        if len_inf > len_sup:
                            if peso3 < peso2:
                                motor2.set_goal(clamp(pos2 - STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                            elif peso2 < peso3:
                                motor3.set_goal(clamp(pos3 - STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                        elif len_sup > len_inf:
                            if peso3 < peso2:
                                motor2.set_goal(clamp(pos2 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                            elif peso2 < peso3:
                                motor3.set_goal(clamp(pos3 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                    else:
                        if pos2 is not None:
                            motor2.set_goal(clamp(pos2 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                        if pos3 is not None:
                            motor3.set_goal(clamp(pos3 + STEP_MOTORES, LIMITE_MIN, LIMITE_MAX))
                except Exception as e:
                    info_motor = f"erro seleção M2/M3: {e}"

                # Desenho visual debug
                p1i, p2i = (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1]))
                cv2.line(color_image, p1i, p2i, (0, 255, 0), 2)
                cv2.circle(color_image, (int(centro_tag[0]), int(centro_tag[1])), 6, (0, 0, 255), -1)

                if dist_cm is not None and dist_cm <= 40 and not garra_fechando:
                    garra_fechando = True
                    try:
                        pos6 = pos_cache[6]
                        pos7 = pos_cache[7]
                        pos2 = pos_cache[2]
                        pos3 = pos_cache[3]
                        if pos6 is not None and pos7 is not None:
                            nova_pos6 = clamp(pos6 + 600, LIMITE_MIN, LIMITE_MAX)
                            nova_pos7 = clamp(pos7 - 600, LIMITE_MIN, LIMITE_MAX)
                            motor3.set_goal(pos3 + 300)
                            motor2.set_goal(pos2 - 100)
                            time.sleep(1.0)
                            motor6.set_goal_position(nova_pos6) if hasattr(motor6, 'set_goal_position') else motor6.set_goal(nova_pos6)
                            motor7.set_goal_position(nova_pos7) if hasattr(motor7, 'set_goal_position') else motor7.set_goal(nova_pos7)
                            cv2.putText(color_image, "⚡ GARRA FECHANDO!", (10, 175),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            break
                        else:
                            print("⚠️ Não foi possível ler posição de M6/M7 para fechar garra.")
                    except Exception as e:
                        print(f"⚠️ Erro ao mover garra: {e}")

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
        try:
            pipeline.stop()
        except Exception:
            pass
        if precisa_fechar_porta:
            try:
                port_handler.closePort()
            except Exception:
                pass
        cv2.destroyAllWindows()
        time.sleep(5)
        print("✔️ Finalizado")

if __name__ == "__main__":
    main(12)
