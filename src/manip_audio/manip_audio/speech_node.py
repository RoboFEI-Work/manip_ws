#!/usr/bin/env python3

import queue
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechNode(Node):

    def __init__(self):
        super().__init__('manip_speech')

        self.declare_parameter('executable', 'espeak-ng')
        self.declare_parameter('voice', 'pt-br')
        self.declare_parameter('rate', 155)
        self.declare_parameter('volume', 100)
        self.declare_parameter('queue_size', 10)

        self._executable = str(self.get_parameter('executable').value)
        self._voice = str(self.get_parameter('voice').value)
        self._rate = int(self.get_parameter('rate').value)
        self._volume = int(self.get_parameter('volume').value)
        queue_size = max(1, int(self.get_parameter('queue_size').value))

        self._messages = queue.Queue(maxsize=queue_size)
        self._running = True
        self._worker = threading.Thread(target=self._speech_worker, daemon=True)
        self._worker.start()

        self.create_subscription(String, '/manip/speech', self._on_speech, 10)

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
        text = message.data.strip()
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

    def _speech_worker(self):
        while self._running:
            try:
                text = self._messages.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                subprocess.run(
                    [
                        self._executable,
                        '-v', self._voice,
                        '-s', str(self._rate),
                        '-a', str(self._volume),
                        text,
                    ],
                    check=False,
                    timeout=30,
                )
            except FileNotFoundError:
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().warning('A síntese de voz excedeu 30 segundos.')
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
