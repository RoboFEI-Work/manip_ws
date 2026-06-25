import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector
from dynamixel_sdk import PortHandler, PacketHandler
import time


class VarreduraAprilTag:
    def __init__(
        self,
        tag_id,
        devicename="COM12",
        baudrate=1000000,
        centro=2600,
        amplitude=600,
        port_handler=None,
        packet_handler=None,
        pipeline=None,
    ):
        self.tag_id = tag_id
        self.devicename = devicename
        self.baudrate = baudrate
        self.centro = centro
        self.amplitude = amplitude

        # === Comunicação Dynamixel ===
        self.portHandler = port_handler or PortHandler(self.devicename)
        # marque se os handlers vieram de fora (não nos pertencem)
        self._external_port = port_handler is not None
        self._external_packet = packet_handler is not None
        self.packetHandler = packet_handler or PacketHandler(2)

        if not self._external_port:  # apenas se for novo
            if not self.portHandler.openPort():
                raise IOError("❌ Falha ao abrir a porta serial.")
            if not self.portHandler.setBaudRate(self.baudrate):
                # se falhar ao configurar, feche porta e informe
                try:
                    self.portHandler.closePort()
                except Exception:
                    pass
                raise IOError("❌ Falha ao configurar baudrate.")

        self.motor_id = 1  # ID do motor de varredura (mude se necessário)
        self._enable_torque()
        self._configurar_velocidade()

        # === Câmera RealSense ===
        self.pipeline = pipeline
        self._pipeline_externo = pipeline is not None
        if self.pipeline is None:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)

        # === Detector de AprilTags ===
        self.detector = Detector(families="tag36h11")

    # --------------------------
    # Funções básicas Dynamixel
    # --------------------------
    def _enable_torque(self):
        ADDR_TORQUE_ENABLE = 64
        TORQUE_ENABLE = 1
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )

    def _disable_torque(self):
        ADDR_TORQUE_ENABLE = 64
        TORQUE_DISABLE = 0
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )

    def _configurar_velocidade(self):
        ADDR_PROFILE_VELOCITY = 112
        ADDR_PROFILE_ACCELERATION = 108
        VELOCIDADE = 15
        ACELERACAO = 5
        self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_PROFILE_VELOCITY, VELOCIDADE
        )
        self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_PROFILE_ACCELERATION, ACELERACAO
        )

    def mover(self, pos):
        ADDR_GOAL_POSITION = 116
        self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_GOAL_POSITION, int(pos)
        )

    # --------------------------
    # Funções de visão
    # --------------------------
    def detectar_tag(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        for r in results:
            if r.tag_id == self.tag_id:
                return True, r
        return False, None

    # --------------------------
    # Alinhamento fino no centro
    # --------------------------
    def alinhar_centro(self, tag, frame_width=640):
        """
        Após detectar a AprilTag, move o motor até que ela esteja centralizada no eixo X da câmera,
        aplicando uma lógica de passo fixo (10 ticks por movimento).
        """
        print("🎯 Alinhando AprilTag no centro da imagem...")
        centro_img = frame_width // 2
        LIMITE_MIN = 0
        LIMITE_MAX = 4095
        STEP = 10  # tamanho do passo fixo em ticks
        TOLERANCIA = 10  # margem de erro em pixels

        def clamp(valor, minimo, maximo):
            return max(minimo, min(maximo, valor))

        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            encontrado, tag = self.detectar_tag(frame)


            centro_tag = np.mean(tag.corners, axis=0)
            x_centro = centro_tag[0]
            erro = x_centro - centro_img

            cv2.line(frame, (centro_img, 0), (centro_img, 480), (255, 0, 0), 2)
            cv2.circle(frame, (int(x_centro), int(centro_tag[1])), 6, (0, 0, 255), -1)
            cv2.imshow("Alinhamento", frame)
            cv2.waitKey(1)

            # Se o erro estiver dentro da tolerância, finaliza
            if abs(erro) <= TOLERANCIA:
                print("✅ Tag centralizada no eixo X!")
                break

            # Movimento discreto: define o sentido e aplica um passo fixo
            try:
                step = STEP if x_centro > centro_img else -STEP
                step = step if abs(erro) > TOLERANCIA else 0

                nova_pos = clamp(self.centro + step, LIMITE_MIN, LIMITE_MAX)
                self.mover(nova_pos)
                self.centro = nova_pos  # atualiza posição central
            except Exception as e:
                print(f"⚠️ Erro ao mover motor: {e}")
                break

            time.sleep(0.05)

    # --------------------------
    # Loop principal de varredura
    # --------------------------
    def executar_varredura(self):
        pos_atual = self.centro
        direcao = 1

        print("🌀 Iniciando varredura... Pressione Ctrl+C para interromper.")
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                encontrado, tag = self.detectar_tag(frame)

                if encontrado:
                    print(f"✅ AprilTag ID {self.tag_id} detectada!")
                    # alinhar e sair; não feche recursos aqui — limpeza no finally
                    self.alinhar_centro(tag)
                    break

                pos_atual += direcao * 20
                if pos_atual > self.centro + self.amplitude:
                    direcao = -1
                elif pos_atual < self.centro - self.amplitude:
                    direcao = 1

                self.mover(pos_atual)
                time.sleep(0.05)

                cv2.imshow("Varredura AprilTag", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        except KeyboardInterrupt:
            print("🛑 Varredura interrompida manualmente.")
        finally:
            # Pare a pipeline apenas se for de propriedade deste objeto
            if not self._pipeline_externo:
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
            # Destroy only varredura-specific windows so we don't close the
            # main application's GUI windows (which would hide the camera
            # view in ajuste_vetorial). Use destroyWindow for named windows.
            try:
                cv2.destroyWindow("Varredura AprilTag")
            except Exception:
                pass
            try:
                cv2.destroyWindow("Alinhamento")
            except Exception:
                pass

            # Se criamos a porta (não foi passada de fora), desligue torque e feche-a.
            if not self._external_port:
                try:
                    self._disable_torque()
                except Exception:
                    pass
                try:
                    self.portHandler.closePort()
                except Exception:
                    pass
            else:
                # Se a porta veio de fora, talvez a varredura tenha deixado o
                # motor 1 em um estado onde não responde imediatamente. Tenta
                # reativar torque e reaplicar perfil para garantir que o
                # sistema externo (ajuste_vetorial) consiga enviar comandos.
                try:
                    self._enable_torque()
                except Exception:
                    pass
                try:
                    self._configurar_velocidade()
                except Exception:
                    pass

            print("🔚 Varredura finalizada.")
