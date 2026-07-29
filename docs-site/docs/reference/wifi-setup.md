# Raspberry Pi NetworkManager WiFi設定ガイド

## 接続設定ファイルの作成

`/etc/NetworkManager/system-connections/` に `.nmconnection` ファイルを作成する。

```ini
[connection]
id=my-wifi
type=wifi
autoconnect=true
autoconnect-priority=100    # 数値が大きいほど優先

[wifi]
mode=infrastructure
ssid=SSID名

[wifi-security]
key-mgmt=wpa-psk
psk=パスワード

[ipv4]
method=auto

[ipv6]
method=auto
```

固定IPの場合は `[ipv4]` を以下に変更：

```ini
[ipv4]
method=manual
addresses=192.168.1.100/24
gateway=192.168.1.1
dns=8.8.8.8;8.8.4.4;
```

パーミッション設定：

```bash
sudo chmod 600 /etc/NetworkManager/system-connections/my-wifi.nmconnection
```

## 起動と反映

```bash
sudo systemctl enable NetworkManager   # 自動起動の有効化
sudo systemctl start NetworkManager    # 起動
sudo nmcli connection reload           # 設定ファイルの再読み込み
sudo nmcli connection up my-wifi       # 接続
```

## 複数接続の優先順位

`autoconnect-priority` の値で制御する（大きい方が優先）。

```bash
sudo nmcli connection modify "home-wifi"       connection.autoconnect-priority 100
sudo nmcli connection modify "office-wifi"      connection.autoconnect-priority 50
sudo nmcli connection modify "mobile-hotspot"   connection.autoconnect-priority 10
```

確認：

```bash
nmcli -f name,autoconnect,autoconnect-priority connection show
```

## フォールバック動作

接続が切れると、`autoconnect=true` のSSIDの中から優先度順に自動再接続される。

### 再試行回数の調整

```bash
sudo nmcli connection modify "my-wifi" connection.autoconnect-retries 3  # デフォルト4、-1で無制限
```

### 切断検知の高速化（WiFi省電力の無効化）

```bash
cat <<EOF | sudo tee /etc/NetworkManager/conf.d/wifi-powersave.conf
[connection]
wifi.powersave = 2
EOF
```

### 自動接続の無効化（手動接続のみにする場合）

```bash
sudo nmcli connection modify "mobile-hotspot" connection.autoconnect no
```

## トラブルシュート

```bash
nmcli connection show --active   # 現在の接続確認
ip addr show wlan0               # IPアドレス確認
nmcli monitor                    # 接続状態のリアルタイム監視
journalctl -u NetworkManager -f  # ログ確認
```
