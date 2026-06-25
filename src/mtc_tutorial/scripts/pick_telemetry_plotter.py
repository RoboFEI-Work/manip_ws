#!/usr/bin/env python3

import csv
from datetime import datetime
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


class PickTelemetryPlotter(Node):
    """Record Dynamixel telemetry while a pick action is active."""

    JOINTS = [f'manip_joint{i}' for i in range(1, 8)]
    INTERFACES = ('current', 'voltage', 'effort')

    def __init__(self):
        super().__init__('pick_telemetry_plotter')

        self.declare_parameter(
            'output_directory',
            '~/manip_ws/pick_telemetry',
        )
        self.declare_parameter(
            'dynamic_joint_states_topic',
            '/dynamic_joint_states',
        )
        self.declare_parameter(
            'pick_active_topic',
            '/manip/pick_active',
        )

        self._output_directory = os.path.abspath(
            os.path.expanduser(
                self.get_parameter('output_directory').value
            )
        )
        os.makedirs(self._output_directory, exist_ok=True)

        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('pick_active_topic').value,
            self._on_pick_active,
            active_qos,
        )
        self.create_subscription(
            DynamicJointState,
            self.get_parameter('dynamic_joint_states_topic').value,
            self._on_joint_state,
            20,
        )

        self._recording = False
        self._start_time = None
        self._samples = []

        self.get_logger().info(
            f'Gráficos de telemetria serão salvos em '
            f'{self._output_directory}'
        )

    def _on_pick_active(self, message):
        if message.data and not self._recording:
            self._recording = True
            self._start_time = self.get_clock().now()
            self._samples = []
            self.get_logger().info('Iniciando telemetria do pick')
        elif not message.data and self._recording:
            self._recording = False
            self._save_recording()

    def _on_joint_state(self, message):
        if not self._recording or self._start_time is None:
            return

        elapsed = (
            self.get_clock().now() - self._start_time
        ).nanoseconds / 1e9

        sample = {'time': elapsed}
        for joint_name, interface_value in zip(
                message.joint_names,
                message.interface_values):
            if joint_name not in self.JOINTS:
                continue
            values = dict(zip(
                interface_value.interface_names,
                interface_value.values,
            ))
            for interface in self.INTERFACES:
                sample[f'{joint_name}/{interface}'] = values.get(
                    interface,
                    math.nan,
                )

        self._samples.append(sample)

    def _save_recording(self):
        if not self._samples:
            self.get_logger().warning(
                'Pick finalizado sem amostras de /dynamic_joint_states'
            )
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_path = os.path.join(
            self._output_directory,
            f'pick_telemetry_{timestamp}',
        )
        self._save_csv(base_path + '.csv')
        self._save_plot(base_path + '.png')

        self.get_logger().info(
            f'Telemetria salva em {base_path}.png e {base_path}.csv'
        )

    def _save_csv(self, path):
        fieldnames = ['time']
        for joint in self.JOINTS:
            for interface in self.INTERFACES:
                fieldnames.append(f'{joint}/{interface}')

        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._samples)

    def _save_plot(self, path):
        figure, axes = plt.subplots(
            3,
            1,
            figsize=(14, 11),
            sharex=True,
        )
        times = [sample['time'] for sample in self._samples]
        panels = (
            ('current', 'Corrente por Dynamixel', 'Corrente [A]'),
            ('voltage', 'Tensão por Dynamixel', 'Tensão [V]'),
            (
                'effort',
                'Esforço estimado por Dynamixel',
                'Esforço [N·m]',
            ),
        )

        for axis, (interface, title, ylabel) in zip(axes, panels):
            for joint in self.JOINTS:
                values = [
                    sample.get(
                        f'{joint}/{interface}',
                        math.nan,
                    )
                    for sample in self._samples
                ]
                axis.plot(times, values, label=joint, linewidth=1.2)
            axis.set_title(title)
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.3)
            axis.legend(ncol=4, fontsize=8)

        axes[-1].set_xlabel('Tempo desde o início do pick [s]')
        figure.suptitle('Telemetria do ciclo de pick')
        figure.tight_layout()
        figure.savefig(path, dpi=160)
        plt.close(figure)


def main(args=None):
    rclpy.init(args=args)
    node = PickTelemetryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._recording:
            node._save_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
