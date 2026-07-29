#!/usr/bin/env python3
# OLED IP Address Display
# SSD1306 128x32 OLEDにIPアドレスとシステム情報を表示する
# smbus + PIL で直接I2C通信（adafruit_blinka/board不要）

import time
import subprocess
import os.path
import traceback
import smbus

from PIL import Image, ImageDraw, ImageFont

# SSD1306 定数
SSD1306_ADDR = 0x3C
SSD1306_WIDTH = 128
SSD1306_HEIGHT = 32
SSD1306_PAGES = SSD1306_HEIGHT // 8


class SSD1306:
    """smbus経由でSSD1306を制御する最小実装"""

    def __init__(self, bus_num, addr=SSD1306_ADDR):
        self.bus = smbus.SMBus(bus_num)
        self.addr = addr
        self._init_display()

    def _cmd(self, cmd):
        self.bus.write_byte_data(self.addr, 0x00, cmd)

    def _init_display(self):
        init_cmds = [
            0xAE,        # Display OFF
            0xD5, 0x80,  # Clock div
            0xA8, SSD1306_HEIGHT - 1,  # Multiplex
            0xD3, 0x00,  # Display offset
            0x40,        # Start line
            0x8D, 0x14,  # Charge pump ON
            0x20, 0x00,  # Horizontal addressing
            0xA1,        # Segment remap
            0xC8,        # COM scan dec
            0xDA, 0x02,  # COM pins (128x32)
            0x81, 0x8F,  # Contrast
            0xD9, 0xF1,  # Pre-charge
            0xDB, 0x40,  # VCOMH deselect
            0xA4,        # Display from RAM
            0xA6,        # Normal (not inverted)
            0xAF,        # Display ON
        ]
        for c in init_cmds:
            self._cmd(c)

    def display(self, image):
        """PIL Image (mode='1', 128x32) をOLEDに転送"""
        # ページ・カラム範囲を設定
        self._cmd(0x21)  # Column addr
        self._cmd(0)
        self._cmd(SSD1306_WIDTH - 1)
        self._cmd(0x22)  # Page addr
        self._cmd(0)
        self._cmd(SSD1306_PAGES - 1)

        # PIL Image → SSD1306フレームバッファ形式に変換
        pixels = image.load()
        buf = []
        for page in range(SSD1306_PAGES):
            for x in range(SSD1306_WIDTH):
                byte = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if y < SSD1306_HEIGHT and pixels[x, y]:
                        byte |= (1 << bit)
                buf.append(byte)

        # 32バイトずつ送信（smbus制限）
        for i in range(0, len(buf), 32):
            chunk = buf[i:i+32]
            self.bus.write_i2c_block_data(self.addr, 0x40, chunk)

    def clear(self):
        image = Image.new('1', (SSD1306_WIDTH, SSD1306_HEIGHT), 0)
        self.display(image)


def detect_i2c_bus():
    """プラットフォームに応じたI2Cバス番号を検出"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().lower()
        if 'orin' in model:
            return 7
        elif 'jetson' in model:
            return 1
    except Exception:
        pass
    return 1


def get_network_interfaces():
    """利用可能なネットワークインターフェースを取得"""
    try:
        result = subprocess.check_output("ls /sys/class/net/", shell=True).decode('utf-8').strip().split()

        ethernet_patterns = ['eth', 'enp', 'ens', 'enx']
        wifi_patterns = ['wlan', 'wlp', 'wls', 'wlx']

        ethernet = None
        wifi = None

        for iface in result:
            if iface == 'lo':
                continue
            iface_lower = iface.lower()
            if not ethernet and any(p in iface_lower for p in ethernet_patterns):
                ethernet = iface
            elif not wifi and any(p in iface_lower for p in wifi_patterns):
                wifi = iface

        return ethernet, wifi
    except Exception:
        return None, None


def get_ip_address(interface):
    """ネットワークインターフェースのIPアドレスを取得"""
    if interface is None:
        return "N/A"

    state_path = f'/sys/class/net/{interface}/operstate'
    if os.path.isfile(state_path):
        try:
            state = subprocess.check_output(f'cat {state_path}', shell=True).decode('ascii').strip()
            if state == 'down':
                return "DOWN"
        except Exception:
            pass

    try:
        cmd = f"ip addr show {interface} | grep -Eo 'inet ([0-9]*\\.)+[0-9]*' | grep -Eo '([0-9]*\\.)+[0-9]*' | head -1"
        result = subprocess.check_output(cmd, shell=True).decode('ascii').strip()
        return result if result else "NO IP"
    except subprocess.CalledProcessError:
        return "ERROR"


def main():
    i2c_bus = detect_i2c_bus()
    print(f"Using I2C bus {i2c_bus}")

    oled = SSD1306(i2c_bus)
    oled.clear()

    # フォント読み込み
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 8)
    except Exception:
        try:
            font = ImageFont.truetype('/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf', 8)
        except Exception:
            font = ImageFont.load_default()

    eth_iface, wifi_iface = get_network_interfaces()
    print(f"Detected - Ethernet: {eth_iface}, WiFi: {wifi_iface}")

    count = 0
    while True:
        image = Image.new('1', (SSD1306_WIDTH, SSD1306_HEIGHT))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, SSD1306_WIDTH, SSD1306_HEIGHT), outline=0, fill=0)

        eth_ip = get_ip_address(eth_iface)
        wifi_ip = get_ip_address(wifi_iface)

        mem_usage = subprocess.check_output(
            "free -m | awk 'NR==2{printf \"%d/%dMB\", $3,$2}'", shell=True
        ).decode('utf-8')

        disk_usage = subprocess.check_output(
            "df -h / | awk 'NR==2{printf \"%s/%s\", $3,$2}'", shell=True
        ).decode('utf-8')

        top = -1
        draw.text((0, top),      f"Eth: {eth_ip}", font=font, fill=255)
        draw.text((0, top + 8),  f"WiFi:{wifi_ip}", font=font, fill=255)
        draw.text((0, top + 16), f"Mem: {mem_usage}", font=font, fill=255)
        draw.text((0, top + 24), f"Disk:{disk_usage}", font=font, fill=255)

        oled.display(image)

        if (eth_ip in ['N/A', 'DOWN', 'NO IP'] or wifi_ip in ['N/A', 'DOWN', 'NO IP']) and count < 10:
            time.sleep(1)
            count += 1
        else:
            break


if __name__ == "__main__":
    try:
        i2c_bus = detect_i2c_bus()
        bus = smbus.SMBus(i2c_bus)
        bus.read_byte(SSD1306_ADDR)
        bus.close()
        main()
    except Exception:
        err_info = traceback.format_exc()
        print(err_info)
