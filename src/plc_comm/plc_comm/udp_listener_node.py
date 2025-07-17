#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from pymodbus.client import ModbusTcpClient
import socket
import json

class UDPJoystickListener(Node):
    def __init__(self):
        super().__init__('udp_listener_node')
        self.client = ModbusTcpClient('192.168.1.5', port=502, timeout=0.1)

        self.udp_ip = "0.0.0.0"
        self.udp_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"UDP listener aktif: {self.udp_ip}:{self.udp_port}")

        self.last_command_time = datetime.now()
        self.last_forward = 0
        self.last_turn = 0
        self.timer = self.create_timer(0.05, self.check_and_receive)

    def check_and_receive(self):
        self.sock.settimeout(0.01)
        try:
            data, _ = self.sock.recvfrom(1024)
            payload = json.loads(data.decode())
            
            # Joystick komutları
            joy_f = int(payload.get('joystick_forward', 0))
            joy_t = int(payload.get('joystick_turn', 0))
            self.last_command_time = datetime.now()
            self.last_forward = joy_f
            self.last_turn = joy_t
            self.process_joystick(joy_f, joy_t)

            # =========== YENİ: Fırça kontrolü ===========
            brush1 = payload.get("brush1", None)
            brush2 = payload.get("brush2", None)
            if brush1 is not None:
                self.write_brush(2068, int(brush1))
                self.get_logger().info(f"Fırça 1 (M20/2068) → {bool(int(brush1))}")
            if brush2 is not None:
                self.write_brush(2069, int(brush2))
                self.get_logger().info(f"Fırça 2 (M21/2069) → {bool(int(brush2))}")
            # =============================================

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"UDP hatası: {str(e)}")

        elapsed = (datetime.now() - self.last_command_time).total_seconds()
        if elapsed > 0.5 and (self.last_forward != 0 or self.last_turn != 0):
            self.get_logger().warn("❌ Bağlantı koptu, robot durduruluyor.")
            self.last_forward = 0
            self.last_turn = 0
            self.process_joystick(0, 0)

    def process_joystick(self, forward, turn):
        left = right = 0
        base_speed = min(abs(forward), 100)

        if forward > 0:
            self.client.write_coils(2048 + 11, [True, False])
            self.client.write_coils(2048 + 3, [False, False])
            if turn > 0:
                left = base_speed
                right = int(base_speed * (1 - abs(turn) / base_speed))
            elif turn < 0:
                right = base_speed
                left = int(base_speed * (1 - abs(turn) / base_speed))
            else:
                left = right = base_speed

        elif forward < 0:
            self.client.write_coils(2048 + 11, [False, True])
            self.client.write_coils(2048 + 3, [False, False])
            if turn > 0:
                left = base_speed
                right = int(base_speed * (1 - abs(turn) / base_speed))
            elif turn < 0:
                right = base_speed
                left = int(base_speed * (1 - abs(turn) / base_speed))
            else:
                left = right = base_speed

        elif forward == 0:
            self.client.write_coils(2048 + 11, [False, False])
            if turn > 0:
                self.client.write_coils(2048 + 3, [True, False])
                left = right = min(abs(turn), 100)
            elif turn < 0:
                self.client.write_coils(2048 + 3, [False, True])
                left = right = min(abs(turn), 100)
            else:
                self.client.write_coils(2048 + 3, [False, False])

        self.client.write_registers(10, [left, right])
        self.get_logger().info(f"Joystick → F:{forward}, T:{turn} | D10={left}, D11={right}")

    # ========== GÜNCELLENEN FONKSİYON ==========
    def write_brush(self, coil_addr, value):
        # Modbus TCP ile COIL (bit) yazma işlemi
        try:
            self.client.write_coil(coil_addr, bool(value))
        except Exception as e:
            self.get_logger().error(f"Fırça coil {coil_addr} yazılamadı: {e}")
    # ============================================

def main(args=None):
    rclpy.init(args=args)
    node = UDPJoystickListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
