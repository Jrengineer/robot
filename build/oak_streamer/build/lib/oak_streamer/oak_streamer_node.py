import socket
import depthai as dai
import cv2

# TCP ayarları
TCP_IP = '0.0.0.0'
TCP_PORT = 5000

# DepthAI Pipeline
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setIspScale(1, 1)  # Zoomsuz
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Çıkış bağlantısı
xout = pipeline.createXLinkOut()
xout.setStreamName("video")
cam.video.link(xout.input)

# Cihaz başlat
device = dai.Device(pipeline)
video_queue = device.getOutputQueue(name="video", maxSize=8, blocking=False)  # 🔧 buffer büyütüldü

# TCP server oluştur
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 🔁 port reuse
server_socket.bind((TCP_IP, TCP_PORT))
server_socket.listen(1)

print(f"🟢 TCP bekleniyor: {TCP_IP}:{TCP_PORT}")
conn, addr = server_socket.accept()
print(f"📡 Flutter bağlantısı geldi: {addr}")

try:
    while True:
        in_frame = video_queue.tryGet()
        if in_frame is None:
            continue

        frame = in_frame.getCvFrame()
        frame = cv2.resize(frame, (320, 240))  # 🔽 düşük çözünürlük

        success, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 25])  # 🔧 kalite iyileştirildi
        if not success:
            continue

        data = jpeg.tobytes()
        conn.sendall(len(data).to_bytes(4, 'big') + data)

        # time.sleep(1 / 30)  # ❌ FPS sınırlaması kaldırıldı
except Exception as e:
    print(f"❌ Bağlantı koptu: {e}")
finally:
    conn.close()
    server_socket.close()
