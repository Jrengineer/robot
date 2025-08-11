#!/usr/bin/env bash
set -euo pipefail

# --- İSTENİRSE VENV ---
# Eğer bir virtualenv kullanıyorsan burayı aç:
# source /home/kaanjetson/.venvs/ros/bin/activate || true

# --- ORTAM ---
export PYTHONUNBUFFERED=1
# Paket import’ları için ws/src’ı PYTHONPATH’e ekleyelim
export PYTHONPATH="/home/kaanjetson/ros2_plc_ws/src:${PYTHONPATH:-}"

# Seri port erişimi için kontrol (opsiyonel – hata verirse sadece uyarır)
if [[ ! -r /dev/ttyUSB0 ]]; then
  echo "⚠️  /dev/ttyUSB0 şu an erişilemiyor (izin/yok). Devam ediliyor..."
fi

# Çalışma klasörü
cd /home/kaanjetson/ros2_plc_ws

# --- UYGULAMA ---
exec /usr/bin/python3 /home/kaanjetson/ros2_plc_ws/src/battery_streamer/battery_udp_node.py
