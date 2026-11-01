# 問題排除紀錄

HW2 進行過程中遇到的問題、原因與解決方式。之後遇到新問題會持續補上。

---

## 1. Mac 無法解析 `rover-yang`

**症狀**

```
ssh: Could not resolve hostname rover-yang: nodename nor servname provided, or not known
```

**原因**：Mac 上的 Tailscale 沒有在執行，MagicDNS 無法把 `rover-yang` 轉成 IP。

**解決**：打開 Tailscale App 並確認已登入（Connected）。

---

## 2. 可以解析 `rover-yang`，但 SSH 逾時

**症狀**

```
ssh: connect to host rover-yang port 22: Operation timed out
```

`tailscale status` 顯示：

```
rover-yang  linux  offline, last seen 16d ago
```

**原因**：rover 本身沒有連上網路。rover 只記得「4442 BroHo」和「Yang-iphone」兩個 Wi-Fi，換了地點（例如到學校）之後兩個都不在範圍內，所以開機後沒有網路，Tailscale 也就離線。

**診斷方式：用 USB 序列埠登入 rover**

rover 用 USB 線接 Mac 時，出現的是**序列埠主控台**，不是網路介面：

```bash
ls /dev/cu.usbmodem*          # 例如 /dev/cu.usbmodem58C60623441
screen /dev/cu.usbmodem58C60623441 115200
# 按 Enter 會出現 rover-yang login:，輸入帳號 ubuntu 和密碼
# 離開 screen：Ctrl-A 再按 K，然後按 y
```

登入後檢查網路：

```bash
ip -br a                       # wlan0 是 DOWN 代表沒連上 Wi-Fi
nmcli -t dev                   # 看各介面狀態
nmcli -t -f NAME,TYPE con show # 看已儲存的 Wi-Fi
nmcli dev wifi list            # 掃描附近的 Wi-Fi
```

**解決**：開啟 iPhone 個人熱點，讓 rover 連上已儲存的「Yang-iphone」。

```bash
sudo nmcli con up Yang-iphone
```

> 注意：`nmcli` 在序列埠主控台下需要 `sudo`，否則會出現 `Not authorized to control networking`。

---

## 3. rover 的系統時間錯誤

**症狀**：rover 顯示的日期是 10/13，實際是 10/29；`timedatectl` 顯示 `System clock synchronized: no`、`RTC time: 1970-01-01`。

**原因**：Rubik Pi 的硬體時鐘（RTC）沒有電池，斷電後時間會回到上次關機時記錄的時間，要等連上網路後由 NTP 校正。時間錯誤可能造成 TLS 憑證驗證失敗，讓 Tailscale 連不上控制伺服器。

**解決**：手動設定時間，再重新啟動 Tailscale：

```bash
sudo date -u -s '2026-10-30 00:33:46'   # 填入目前的 UTC 時間
sudo systemctl restart tailscaled
tailscale status                         # rover-yang 應顯示為上線
```

之後 NTP 會自動同步（`timedatectl` 顯示 `System clock synchronized: yes`）。

---

## 4. Tailscale 重新上線後，Mac 仍然 SSH 逾時

**症狀**：rover 端 `tailscale status` 正常，但 Mac 端 `tailscale ping rover-yang` 逾時（`rx 0`）。

**原因**：rover 剛重新上線，雙方的連線路徑（DERP 中繼）還沒建立。

**解決**：從 rover 端反向 ping 一次 Mac，連線就建立起來了：

```bash
tailscale ping fongs-macbook-pro
```

---

## 5. 每次 SSH 都要求在瀏覽器登入

**症狀**

```
# Tailscale SSH requires an additional check.
# To authenticate, visit: https://login.tailscale.com/a/xxxxxxxx
```

**原因**：Tailscale SSH 預設的 ACL 規則是 `"action": "check"`，會定期（預設 12 小時）要求在瀏覽器重新驗證身分。這不是 rover 的問題。

**解決（擇一）**

- 點連結登入一次即可，有效期間內不會再問。
- 到 Tailscale 管理後台（Access controls）把自己裝置的 SSH 規則從 `"check"` 改成 `"accept"`，之後就不需要再驗證：

```json
"ssh": [
  {
    "action": "accept",
    "src":    ["autogroup:member"],
    "dst":    ["autogroup:self"],
    "users":  ["autogroup:nonroot", "root"]
  }
]
```

---

## 6. rover 上沒有相機節點與校正工具

**症狀**：`ros2 pkg list` 找不到 `robot_vision_camera` 和 `camera_calibration`。

**原因**：rover 的 `~/ros2_ws/src` 只同步過 HW1 的 `robot_control`，校正工具也沒有預先安裝。

**解決**：

```bash
# 在 Mac 上，從 rubikpi_ros2 資料夾同步相機套件到 rover
rsync -az --exclude build --exclude install --exclude log robot_vision ubuntu@rover-yang:~/ros2_ws/src/

# 在 rover 上安裝 ROS2 校正工具
sudo apt-get install -y ros-jazzy-camera-calibration
```

---

## 7. 編譯 `robot_vision_camera` 失敗：找不到 gstreamer-1.0

**症狀**

```
Package 'gstreamer-1.0', required by 'virtual:world', not found
```

**原因**：rover 缺少 GStreamer 的開發函式庫（只有執行檔，沒有標頭檔）。

**解決**：

```bash
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
cd ~/ros2_ws && colcon build --packages-select robot_vision_camera
```

---

## 8. 相機節點無法啟動：`Unable to set the pipeline to the playing state`

**症狀**：相機節點的 topic 有出現，但沒有影像。直接用 GStreamer 測試：

```bash
gst-launch-1.0 -e qtiqmmfsrc camera=0 ! "video/x-raw,format=NV12,width=1280,height=720,framerate=10/1" ! fakesink
# ERROR: ... Failed to Open Camera!
```

`journalctl -u cam-server` 顯示 `Invalid camera idx: 0`，`sudo dmesg` 顯示 I2C 讀取感測器失敗：

```
CAM_ERR: CAM-CCI: cam_cci_read: ERROR with Slave 0x34
CAM_ERR: CAM-SENSOR: cam_sensor_subdev_ioctl: Failed in Driver cmd: -19
```

**原因**：rover 使用的相機感測器是 **OV5647**，Rubik Pi 系統預設只內建 IMX219／IMX477／IMX708 等感測器的驅動。系統只會去 IMX 系列的 I2C 位址（`0x34`）找相機，自然找不到。相機燈有亮只代表有供電，和排線無關。

**解決**（課程助教公告的修正，講師已背書）：安裝 OV5647 驅動檔，然後重新開機。

```bash
# 在 rover 上
cd ~
git clone https://github.com/notu97/rubikpi_flash.git
cd rubikpi_flash
sudo cp com.qti.sensor.ov5647.so* com.qti.sensormodule.cam*_ov5647.bin /usr/lib/qcm6490/camera/
ls /usr/lib/qcm6490/camera/ | grep ov5647   # 應列出 5 個檔案
sudo reboot                                  # 相機只在開機時偵測，一定要重開
```

> 注意：只重新啟動 `cam-server` 服務無法生效，必須重新開機。

---

## 9. Python 訂閱原始影像只有約 2.7 Hz

**症狀**：相機設定是 10 fps，但 Python 節點訂閱 `/camera/image_raw` 只收到約 2.7 Hz。

**原因**：1280×720 的 BGR 原始影像每張約 2.7 MB，在 Python 裡反序列化太慢。

**解決**：改訂閱 `/camera/image_raw/compressed`，再用 `cv2.imdecode` 解碼，就能跑到 10 Hz。在 Foxglove 看畫面時也要選 compressed 的 topic，否則透過熱點和 Tailscale 傳原始影像會非常卡。

---

## 10. rover 反覆斷線，Foxglove 一直顯示 Waiting for messages

**症狀**：Foxglove 連得上、看得到 topic 列表，但 Image 面板一直是 `Waiting for messages...`；SSH 時好時壞，`tailscale status` 顯示 `offline, last seen 1m ago`。

**原因**（兩個同時存在）：

1. **rover 曾經非預期重開機**：`last -x reboot shutdown` 顯示有一次開機前沒有對應的關機紀錄，代表是突然斷電（電池電量低或電壓不穩）。重開後相機節點不會自動啟動。
2. **Wi-Fi 省電模式**：NetworkManager 預設會開啟無線網卡省電，容易造成熱點連線不穩。

**解決**：

```bash
# 關閉熱點連線的 Wi-Fi 省電模式（2 = disable），設定會永久保存
sudo nmcli con modify Yang-iphone 802-11-wireless.powersave 2
nmcli -g 802-11-wireless.powersave con show Yang-iphone   # 應顯示 disable

# 檢查是否有非預期重開機
last -x reboot shutdown | head -5
uptime
```

- 電池要保持充足電量。
- rover 重開機後，要重新執行相機的 launch 指令，Foxglove 也要重新連線。

> 注意：不要透過 SSH 執行 `nmcli con up Yang-iphone`，重新啟動 Wi-Fi 會把自己的 SSH 連線切斷。

---

## 11. `ssh rover-yang` 被拒絕：not permit you to SSH as user "yanglin"

**症狀**

```
tailscale: tailnet policy does not permit you to SSH as user "yanglin"
```

**原因**：沒有指定帳號時，SSH 會用 Mac 的使用者名稱（`yanglin`）登入，但 rover 上只有 `ubuntu` 帳號。

**解決**：在 Mac 的 `~/.ssh/config` 加入預設帳號，之後打 `ssh rover-yang` 就會自動用 `ubuntu` 登入：

```
Host rover-yang
    User ubuntu
```

---

## 12. 物體偵測：推論速度時快時慢（24 ms～150 ms）

**症狀**：`ros2 run landmark_detector benchmark` 有時 YOLO11n（320）只要 24 ms，有時要 85 ms 以上。

**原因**：Rubik Pi（QCS6490）的 CPU 是大小核架構：CPU 0～3 是較慢的 A55 小核，CPU 4～7 是 A78 大核。Linux 排程器常把推論的執行緒排到小核上，速度會慢約 3.5 倍。

**解決**：把程式固定在大核上執行。`landmark_detector/yolo.py` 已經在建立 ONNX Runtime 前自動設定，不需要手動處理。手動測試的方法：

```bash
taskset -c 4-7 <指令>     # 只在 CPU 4～7 上執行
```

執行緒數用 4 最快，開到 6 或 8 會用到小核，反而變慢。

---

## 13. rover 過熱降頻（CPU 溫度 80～88°C）

**症狀**：偵測器連續跑一段時間後越來越慢，CPU 溫度超過 80°C。

**原因**：Rubik Pi 沒有主動散熱時，連續跑 YOLO 會過熱，系統會自動降低 CPU 頻率。

**解決**：

- 偵測器加了 `max_fps` 參數（預設 5），限制每秒最多處理幾張畫面。導航用 5 Hz 的偵測就夠了。
- 建議加裝散熱片或風扇。
- 之後可以把模型改跑 NPU，速度更快也更省電。

```bash
# 查看目前最高溫度（單位：千分之一 °C）
cat /sys/class/thermal/thermal_zone*/temp | sort -n | tail -1
```

---

## 14. 校正結果不穩定：焦距從 638 跳到 1200 以上

**症狀**：第二批 40 張只有 15 張能用，算出 fx ≈ 1212、水平視角只有 55°；換成全部照片做交叉驗證，fx 也在 638 到 846 之間跳動。RMS 看起來還可以（< 1 px），但參數不可信。

**原因**：

1. 背光：亮燈在鏡頭正前方，相機壓低曝光，白格變灰，SB 偵測器抓不到板子。
2. 板子太遠（約 0.5～1.5 公尺）、方格太小，傾斜也不夠，焦距和距離分不開。

**解決**：

- 改善光線，板子拿到 20～50 公分，並多拍傾斜的姿勢。
- `calib_capture.py` 現在會當場擋掉太遠、低對比或模糊的照片，並顯示傾斜張數。
- `calibrate.py` 在 SB 失敗時改用傳統偵測器。

> RMS 小不代表校正正確，它只反映已拍到的資料。判斷時要看：參數是否合理、換一批照片後是否穩定。

---

## 15. 畫面幾乎是黑白，路標顏色分不出來

**症狀**：黃色馬克杯看起來是米色，藍色麥片盒變灰。

**原因**：相機節點的預設值是 `saturation 0.2`、`hue 0.1`，把顏色壓掉又偏移了色相。

**解決**：`robot_vision_camera.launch.py` 改成預設 `saturation 0.6`、`hue 0.0`。設成 1.0 時暗部和白色物體會明顯偏綠，所以不用。訓練資料和實際使用必須是同一組設定。

---

## 16. YOLO-World 自動標註不可靠

**症狀**：30 張照片中，麥片盒、風扇完全沒有標到（信心值門檻 0.05）；小狗的框只包到玩偶上半部，沒有包含杯子。換提示詞和彩色畫面也沒有明顯改善，甚至會把小狗和杯子標成同一類。

**解決**：先人工標註第一批照片、訓練模型，再用這個模型幫忙預標其餘照片。

---

## 17. rover 開機後連線與 tmux 的小問題

**問題 1：`ssh rover-yang` 逾時，區域網路 IP 也 ping 不到。**

- 症狀：`tailscale status` 顯示 `offline, last seen 20h ago`。
- 原因：rover 沒有開機。這和第 1 節的「Wi-Fi 連不上」不同，開機後就自動恢復。
- 解法：先確認電源，再查網路。

**問題 2：用 IP 連線出現 `Host key verification failed`。**

- 原因：`known_hosts` 裡只有 `rover-yang`，沒有這個 IP 的紀錄。
- 解法：改用 `ssh rover-yang`。

**問題 3：`tmux new-window -t cam` 出現 `create window failed: index 0 in use`。**

- 原因：`-t cam` 會被當成「視窗」目標，指到已經存在的第 0 個視窗。
- 解法：在名稱後面加冒號，寫成 `tmux new-window -t cam: -n cap`。

**問題 4：拍到 `--max` 上限後想多拍。**

- 程式會自動結束。用同一個 `--session` 和較大的 `--max` 重新執行即可，編號會接著已有的張數，舊照片不會被蓋掉。

## 18. 導航程式偶爾報 `No IMU data`，但 IMU 明明有資料

- 症狀：`ekf_navigator` 啟動後出現 `No IMU data on /imu_data. Is motor_control running?`，同時 `ros2 topic hz /imu_data` 卻有約 48 Hz。
- 原因：新節點要花幾秒才連上 topic，2 秒的陀螺儀校零在收到第一筆資料前就結束了。第一次能跑只是剛好連得夠快。
- 解法：`measure_gyro_bias` 先等第一筆 IMU 資料，最多 10 秒，再開始校零。

## 19. 行駛中車子慢慢偏掉，EKF 的朝向一直增加

- 症狀：直線行駛時 EKF 朝向持續往左增加，車子卻往右偏；路標方位和預期越差越多，最後被離群值檢查擋掉。
- 原因：行駛中陀螺儀有時會連續輸出**完全相同的數值**，一次可達數秒。例如 2.6 秒內一直是 10.69°/s。積分後 EKF 以為車子在左轉，控制器就往右修正，真實車子因此被帶偏。
- 怎麼查到的：在位姿紀錄裡同時記錄陀螺儀和左右輪指令，就看得到一長串相同的數值。
- 解法：連續 5 筆相同時，改用左右輪指令估算轉速，並放大朝向的不確定度，讓路標可以修正。
- 仍未確定：為什麼 IMU 會卡住。目前只是在 `ekf_navigator` 偵測並繞過，`motor_control` 沒有改。

## 快速檢查清單（連不上 rover 時）

1. Mac 的 Tailscale App 是否已連線？
2. iPhone 熱點是否開啟？
3. `tailscale status` 中 `rover-yang` 是否顯示 offline？
   - 是：用 USB 序列埠登入，檢查 `ip -br a` 和 `date`。
4. 時間錯誤：`sudo date -u -s '...'`，然後 `sudo systemctl restart tailscaled`。
5. Mac 仍 ping 不到：從 rover 端 `tailscale ping fongs-macbook-pro`。
