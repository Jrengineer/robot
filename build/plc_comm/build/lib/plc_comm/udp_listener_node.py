#!/usr/bin/env python3
"""
UDP üzerinden joystick ve fırça verilerini alıp, Delta PLC'ye Modbus TCP ile gönderen ROS2 Node.
Joystick değerleri aralığını 0-100 ile sınırlandırır. Register değerleri asla negatif olmaz.
Bağlantı koptuğunda veya gecikme yüksekse robotu güvenli moda çeker.
"""

import rclpy
from rclpy.node import Node
import time
import socket
import json
from pymodbus.client import ModbusTcpClient

class UDPJoystickListener(Node):
    def __init__(self):
        super().__init__('udp_listener_node')
        self.client = ModbusTcpClient('192.168.1.5', port=502, timeout=0.1)

        self.udp_ip = "0.0.0.0"
        self.udp_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.01)
        self.get_logger().info(f"UDP listener aktif: {self.udp_ip}:{self.udp_port}")

        self.is_connected = True
        self.timeout_counter = 0

        # --- Son yazılan komutlar ---
        self.last_forward = 0
        self.last_turn = 0
        self.last_brush1 = None
        self.last_brush2 = None

        self.timer = self.create_timer(0.05, self.check_and_receive)  # 20Hz

    def check_and_receive(self):
        last_payload = None

        # Tüm buffer’ı boşalt, sadece en son paketi işle
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                payload = json.loads(data.decode())
                last_payload = payload
            except socket.timeout:
                break
            except Exception as e:
                self.get_logger().error(f"UDP decode hatası: {str(e)}")
                break

        if last_payload is not None:
            pkt_ts = int(last_payload.get('ts', 0))
            now_ts = int(time.time() * 1000)
            gecikme_ms = now_ts - pkt_ts
            self.get_logger().info(f"UDP paket gecikmesi: {gecikme_ms} ms")

            # Gecikme limiti burada! (ör: 3000 ms = 3 sn)
            if gecikme_ms > 6000:
                if self.is_connected:
                    self.get_logger().warn("AĞ GECİKMESİ YÜKSEK! Robot ve fırçalar güvenli moda geçti.")
                self.is_connected = False
                self.process_joystick(0, 0, force=True)
                self.write_brush(2068, 0, force=True)
                self.write_brush(2069, 0, force=True)
                self.timeout_counter = 0
                return

            # Joystick ve brush değerlerini işle
            joy_f = int(last_payload.get('joystick_forward', 0))
            joy_t = int(last_payload.get('joystick_turn', 0))
            brush1 = int(last_payload.get("brush1", 0))
            brush2 = int(last_payload.get("brush2", 0))
            self.process_joystick(joy_f, joy_t)
            self.write_brush(2068, brush1)
            self.write_brush(2069, brush2)

            # Bağlantı sağlıklı, sayaç sıfırla
            if not self.is_connected:
                self.get_logger().info("Mobil uygulama yeniden bağlandı.")
            self.is_connected = True
            self.timeout_counter = 0

        else:
            self.timeout_counter += 1

        # 3 tick boyunca UDP verisi alınmazsa bağlantı kopmuş kabul edilir.
        if self.timeout_counter >= 3:
            if self.is_connected:
                self.get_logger().warn("❌ Mobil uygulama bağlantısı koptu, robot ve fırçalar durduruluyor.")
            self.is_connected = False
            self.process_joystick(0, 0, force=True)
            self.write_brush(2068, 0, force=True)
            self.write_brush(2069, 0, force=True)

    def process_joystick(self, forward, turn, force=False):
        # Sadece değer değiştiyse yaz, veya force=True ise yaz
        if force or forward != self.last_forward or turn != self.last_turn:
            left = right = 0
            base_speed = min(max(abs(forward), 0), 100)  # 0-100 aralığında

            # Ana sürüş hesapları
            if forward > 0:
                self.client.write_coils(2048 + 11, [True, False])  # İleri
                self.client.write_coils(2048 + 3, [False, False])
                if turn > 0:  # Sağa dönüş (sol daha hızlı)
                    left = base_speed
                    right = int(base_speed * (1 - abs(turn) / 100))
                elif turn < 0:  # Sola dönüş (sağ daha hızlı)
                    right = base_speed
                    left = int(base_speed * (1 - abs(turn) / 100))
                else:
                    left = right = base_speed
            elif forward < 0:
                self.client.write_coils(2048 + 11, [False, True])  # Geri
                self.client.write_coils(2048 + 3, [False, False])
                if turn > 0:
                    left = base_speed
                    right = int(base_speed * (1 - abs(turn) / 100))
                elif turn < 0:
                    right = base_speed
                    left = int(base_speed * (1 - abs(turn) / 100))
                else:
                    left = right = base_speed
            elif forward == 0:
                self.client.write_coils(2048 + 11, [False, False])  # Dur
                if turn > 0:
                    self.client.write_coils(2048 + 3, [True, False])  # Sağa
                    left = right = min(abs(turn), 100)
                elif turn < 0:
                    self.client.write_coils(2048 + 3, [False, True])  # Sola
                    left = right = min(abs(turn), 100)
                else:
                    self.client.write_coils(2048 + 3, [False, False])

            # DEĞERLERİ KISITLA
            left = max(0, min(100, left))
            right = max(0, min(100, right))

            try:
                self.client.write_registers(10, [left, right])
            except Exception as e:
                self.get_logger().error(f"write_registers(10, [{left}, {right}]) HATA: {e}")

            self.get_logger().info(f"Joystick → F:{forward}, T:{turn} | D10={left}, D11={right}")
            self.last_forward = forward
            self.last_turn = turn

    def write_brush(self, coil_addr, value, force=False):
        last_val = self.last_brush1 if coil_addr == 2068 else self.last_brush2
        print(f"[JETSON DEBUG] Fırça {coil_addr} gelen değer: {value}, last_val: {last_val}, force: {force}")
        if force or value != last_val:
            self.client.write_coil(coil_addr, bool(value))
            self.get_logger().info(f"Fırça {coil_addr} → {bool(value)}")
            if coil_addr == 2068:
                self.last_brush1 = value
            elif coil_addr == 2069:
                self.last_brush2 = value

def main(args=None):
    rclpy.init(args=args)
    node = UDPJoystickListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
