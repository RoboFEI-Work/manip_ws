#!/usr/bin/env python3

import queue
import re
import shutil
import subprocess
import threading
from pathlib import Path
from tempfile import NamedTemporaryFile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechNode(Node):

    def __init__(self):
        super().__init__('manip_speech')

        self.declare_parameter('backend', 'espeak')
        self.declare_parameter('executable', 'espeak-ng')
        self.declare_parameter('voice', 'pt-br')
        self.declare_parameter('rate', 165)
        self.declare_parameter('pitch', 45)
        self.declare_parameter('word_gap', 0)
        self.declare_parameter('volume', 100)
        self.declare_parameter('piper_executable', 'piper-tts')
        self.declare_parameter('piper_model', '')
        self.declare_parameter('player_executable', 'ffplay')
        self.declare_parameter('voice_effect', 'none')
        self.declare_parameter('effect_sample_rate', 22050)
        self.declare_parameter('queue_size', 10)

        self._backend = str(self.get_parameter('backend').value)
        self._executable = str(self.get_parameter('executable').value)
        self._voice = str(self.get_parameter('voice').value)
        self._rate = int(self.get_parameter('rate').value)
        self._pitch = int(self.get_parameter('pitch').value)
        self._word_gap = int(self.get_parameter('word_gap').value)
        self._volume = int(self.get_parameter('volume').value)
        self._piper_executable = str(
            self.get_parameter('piper_executable').value
        )
        self._piper_model = str(self.get_parameter('piper_model').value)
        self._player_executable = str(
            self.get_parameter('player_executable').value
        )
        self._voice_effect = str(self.get_parameter('voice_effect').value)
        self._effect_sample_rate = int(
            self.get_parameter('effect_sample_rate').value
        )
        configured_queue_size = (
            self.get_parameter('queue_size').value
        )
        queue_size = max(1, int(configured_queue_size))

        self._messages = queue.Queue(maxsize=queue_size)
        self._running = True
        self._worker = threading.Thread(
            target=self._speech_worker,
            daemon=True,
        )
        self._worker.start()

        self.create_subscription(String, '/manip/speech', self._on_speech, 10)

        self._log_backend_status()

    def _log_backend_status(self):
        if self._backend == 'piper':
            if shutil.which(self._piper_executable) is None:
                self.get_logger().error(
                    f"Executável '{self._piper_executable}' não encontrado. "
                    'Instale o Piper para usar voz local por IA.'
                )
                return
            if not self._piper_model:
                self.get_logger().error(
                    "Parâmetro 'piper_model' vazio. Informe o caminho do "
                    'modelo .onnx para usar o backend piper.'
                )
                return
            if not Path(self._piper_model).exists():
                self.get_logger().error(
                    f"Modelo Piper não encontrado: '{self._piper_model}'"
                )
                return
            if shutil.which(self._player_executable) is None:
                self.get_logger().error(
                    f"Player '{self._player_executable}' não encontrado."
                )
                return
            self.get_logger().info(
                'Síntese local Piper pronta em /manip/speech, '
                f'modelo={self._piper_model}, efeito={self._voice_effect}'
            )
            return

        if shutil.which(self._executable) is None:
            self.get_logger().error(
                f"Executável '{self._executable}' não encontrado. "
                'Instale com: sudo apt install espeak-ng'
            )
        else:
            self.get_logger().info(
                f'Síntese de voz pronta em /manip/speech, voz={self._voice}'
            )

    def _on_speech(self, message):
        text = self._normalize_text(message.data)
        if not text:
            return

        try:
            self._messages.put_nowait(text)
        except queue.Full:
            self.get_logger().warning(
                'Fila de voz cheia; descartando a mensagem mais antiga.'
            )
            try:
                self._messages.get_nowait()
                self._messages.task_done()
            except queue.Empty:
                pass
            self._messages.put_nowait(text)

    @staticmethod
    def _normalize_text(text):
        """Remove formatting artifacts that produce unnatural pauses."""
        text = text.replace('_', ' ')
        return re.sub(r'\s+', ' ', text).strip()

    def _speech_command(self, text):
        return [
            self._executable,
            '-v', self._voice,
            '-s', str(self._rate),
            '-p', str(self._pitch),
            '-g', str(self._word_gap),
            '-a', str(self._volume),
            '-z',
            text,
        ]

    def _piper_command(self, output_file):
        return [
            self._piper_executable,
            '--model', self._piper_model,
            '--output_file', str(output_file),
        ]

    def _voice_effect_filter(self):
        sample_rate = max(8000, self._effect_sample_rate)
        if self._voice_effect == 'dog':
            return (
                f'asetrate={sample_rate}*0.94,aresample={sample_rate},'
                'atempo=1.04,'
                'vibrato=f=4:d=0.04,'
                'volume=1.05'
            )
        if self._voice_effect == 'dog_deep':
            return (
                f'asetrate={sample_rate}*0.82,aresample={sample_rate},'
                'atempo=1.10,'
                'vibrato=f=5:d=0.08,'
                'volume=1.10'
            )
        if self._voice_effect == 'cartoon_dog':
            return (
                f'asetrate={sample_rate}*0.82,aresample={sample_rate},'
                'atempo=1.12,'
                'tremolo=f=7:d=0.18,vibrato=f=5:d=0.15'
            )
        return ''

    def _play_audio_command(self, audio_file):
        command = [
            self._player_executable,
            '-nodisp',
            '-autoexit',
            '-loglevel',
            'error',
        ]
        audio_filter = self._voice_effect_filter()
        if audio_filter:
            command.extend(['-af', audio_filter])
        command.append(str(audio_file))
        return command

    def _speak_with_piper(self, text):
        with NamedTemporaryFile(suffix='.wav', delete=False) as audio_file:
            audio_path = Path(audio_file.name)

        try:
            subprocess.run(
                self._piper_command(audio_path),
                input=text,
                text=True,
                check=False,
                timeout=30,
            )
            subprocess.run(
                self._play_audio_command(audio_path),
                check=False,
                timeout=30,
            )
        finally:
            audio_path.unlink(missing_ok=True)

    def _speak(self, text):
        if self._backend == 'piper':
            self._speak_with_piper(text)
            return

        subprocess.run(
            self._speech_command(text),
            check=False,
            timeout=30,
        )

    def _speech_worker(self):
        while self._running:
            try:
                text = self._messages.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._speak(text)
            except FileNotFoundError:
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    'A síntese de voz excedeu 30 segundos.'
                )
            finally:
                self._messages.task_done()

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
