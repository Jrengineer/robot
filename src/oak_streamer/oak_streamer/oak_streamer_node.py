import socket
import struct
import cv2
import depthai as dai

def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_rgb = pipeline.create(dai.node.XLinkOut)

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setPreviewSize(640, 360)  # <<< Küçük çözünürlük, düşük gecikme için

    xout_rgb.setStreamName("video")
    cam_rgb.preview.link(xout_rgb.input)

    return pipeline

def start_server(host='0.0.0.0', port=5000):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"🚀 TCP server başlatıldı: {host}:{port}")

    while True:
        print("📡 Bağlantı bekleniyor...")
        client_socket, addr = server_socket.accept()
        print(f"✅ Flutter bağlantısı geldi: {addr}")

        try:
            with dai.Device(create_pipeline()) as device:
                video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

                while True:
                    frame = video.get().getCvFrame()
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]  # <<< JPEG kalite %60
                    result, img_encoded = cv2.imencode('.jpg', frame, encode_param)

                    if not result:
                        continue

                    data = img_encoded.tobytes()
                    length = struct.pack('>I', len(data))

                    try:
                        client_socket.sendall(length + data)
                    except (socket.error, BrokenPipeError):
                        print("⚡ Bağlantı koptu, yeni bağlantı bekleniyor...")
                        break

        except Exception as e:
            print(f"🚨 Hata oluştu: {e}")

        finally:
            client_socket.close()

def main():
    start_server()

if __name__ == "__main__":
    main()
