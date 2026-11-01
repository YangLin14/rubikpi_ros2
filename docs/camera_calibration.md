# 相機校正教學

用棋盤格校正 rover 的 OV5647 廣角相機，求出相機內參（焦距、光學中心）與畸變係數。

## 事前準備

| 項目 | 值 |
|---|---|
| 棋盤格方格數 | 11 × 8（長邊 6 黑 5 白，短邊 4 黑 4 白） |
| 內角點（校正要填的） | **10 × 7** |
| 每格邊長 | **0.0207 m**（尺量 10 格總長 20.7 cm） |
| 白邊 | 不影響校正：程式只找四個方格交會的內角點，不會用到外框 |
| 影像解析度 | 1280 × 720（校正和實際使用必須相同） |
| 校正程式 | `calibration/calib_capture.py`、`calibration/calibrate.py`（rover 上在 `~/calibration/`） |

如果 rover 上還沒有校正程式，在 Mac 的 `rubikpi_ros2` 資料夾執行：

```bash
rsync -az calibration ubuntu@rover-yang:~/
```

---

## 步驟 0：確認 rover 連得上

1. iPhone 打開「設定 → 個人熱點」：
   - 「允許其他人加入」要打開。
   - 建議打開「最大化相容性」（改用 2.4 GHz，比較穩定）。
   - 校正過程中盡量停在這個畫面，熱點比較不會被系統關掉。
2. rover 開機，等約 30 秒會自動連上熱點。
3. Mac 的 Tailscale App 要是 Connected。
4. 測試：

```bash
ssh ubuntu@rover-yang
```

連不上時參考 `docs/troubleshooting.md` 的「快速檢查清單」。

---

## 步驟 1：SSH 到 rover，開 tmux

```bash
ssh ubuntu@rover-yang
tmux new -s cam
```

**tmux** 讓你在同一個 SSH 連線裡開多個視窗，SSH 斷線時程式也會繼續跑。重新連上後用 `tmux attach -t cam` 接回去。

| 按鍵 | 作用 |
|---|---|
| `Ctrl-b` 再按 `c` | 開新視窗 |
| `Ctrl-b` 再按 `0` / `1` / `2` | 切換到第幾個視窗 |
| `Ctrl-b` 再按 `d` | 離開 tmux（程式繼續跑） |

---

## 步驟 2：啟動相機（視窗 0）

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py
```

- 兩個 `source` 的作用是讓終端機找得到 ROS2 和你編譯好的套件。**每開一個新視窗都要重新執行。**
- 這個 launch 檔會同時啟動相機節點，以及 Foxglove 用的連線伺服器（port 8765）。
- 看到 `Server listening on port 8765` 就表示成功。
- 這個視窗不要關。

---

## 步驟 3：確認有影像（視窗 1）

按 `Ctrl-b c` 開新視窗：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list                                   # 列出所有 topic
ros2 topic hz /camera/image_raw/compressed        # 看幀率，應約 10 Hz，按 Ctrl-C 停止
```

**topic** 是 ROS 節點之間傳資料的管道。相機節點把影像發佈到 `/camera/image_raw`，其他程式訂閱它就能拿到影像。應該看到：

```
/camera/camera_info
/camera/image_raw
/camera/image_raw/compressed
```

---

## 步驟 4：用 Foxglove 在 Mac 上看畫面

1. 打開 https://app.foxglove.dev 並登入。如果網頁版連不上，改用 [桌面版](https://foxglove.dev/download)。
2. 點 **Open connection**，選 **Foxglove WebSocket**，網址填：
   - `ws://rover-yang:8765`
   - 連不上的話改填 `ws://100.124.88.109:8765`
   - 在家（4442 BroHo）也可以直接用 rover 的區域網路 IP，例如 `ws://192.168.1.72:8765`
3. 新增 **Image** 面板，topic 選 **`/camera/image_raw/compressed`**，Calibration 選 None。

> ⚠️ 不要選 `/camera/image_raw`。原始影像每秒約 27 MB，經過熱點會卡住。

---

## 步驟 5：拍校正照片（視窗 2）

先把 **rover 放好、不要動它**，由你拿著棋盤格移動。按 `Ctrl-b c` 開新視窗：

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/calibration/calib_capture.py --cols 10 --rows 7 --out ~/calib_images
```

在 Foxglove 再加一個 Image 面板，topic 選 **`/calib/preview/compressed`**，就能看到即時預覽：

- **彩色線條**：偵測到的棋盤格角點。
- **上方黃字**：目前狀態、已存張數、傾斜張數（`tilted`）、覆蓋率。
  - `board not found`：看不到整塊棋盤格。
  - `too far (..px) - move closer`：板子太遠，方格小於 22 像素，不會存檔。
  - `board found - move to a new pose`：有偵測到，但跟已存的照片太像，換個位置。
  - `low contrast/blurry - add light, hold still`：太暗或太模糊，不會存檔。
  - `SAVED #n`：存了一張。
- **綠色區域**：已經被棋盤格覆蓋過的範圍。**目標是把整個畫面塗綠，尤其是四個角和邊緣。**

### 拍攝要點

0. **光線**：光源要在 rover 後方或側面，照亮板子。不要讓窗戶或亮燈出現在鏡頭正前方，否則相機會壓低曝光，白格變灰。下半部拍不到時，把 rover 墊高約 30 cm。
1. **整塊棋盤格都要在畫面內**，10×7 個內角點都要看得到才算數。板子很小，大概要離鏡頭 **20～50 公分**，超過約 60 公分程式就不會存檔。手指不要蓋到方格，拿白邊就好。
2. **位置**：中間、四個角落、上下左右邊緣都要拍到。廣角鏡頭的畸變在角落最嚴重，最需要資料。
3. **距離**：遠近都要有，讓棋盤格在畫面裡有大有小。
4. **傾斜**：往左右、上下各傾斜約 30～45 度。沒有傾斜的照片，焦距會算不準，`tilted` 至少要到 15。
5. **拿穩**：每個姿勢停 1 秒，動作太快會糊掉。
6. 存到 40 張會自動停止，也可以按 `Ctrl-C` 提早結束。**25 張以上就夠用。**

---

## 步驟 6：計算相機參數

```bash
python3 ~/calibration/calibrate.py --images ~/calib_images --cols 10 --rows 7 --square 0.0207 \
    --out ~/calib_images/camera_parameter.yaml
```

輸出說明：

| 項目 | 意義 |
|---|---|
| **RMS reprojection error** | 用算出的參數把棋盤格角點投影回影像，和實際偵測位置的平均誤差。**< 0.5 px 很好，< 1.0 px 可以接受。** |
| `worst` | 誤差最大的 5 張照片。誤差太大就把該照片刪掉後重算。 |
| **K（相機矩陣）** | 焦距 fx、fy 和光學中心 cx、cy（單位：像素）。 |
| **Distortion** | 畸變係數 `[k1 k2 p1 p2 k3]`。 |
| Field of view | 水平和垂直視角。 |

程式也會輸出 `~/calib_images/undistort_check.jpg`，左邊是原圖、右邊是去畸變後的圖。**右邊的直線應該要是直的。** 可以複製回 Mac 查看：

```bash
# 在 Mac 上執行
scp ubuntu@rover-yang:~/calib_images/undistort_check.jpg ~/Downloads/
```

> `--square` 不影響焦距和畸變係數（填不同數值算出的 K、D 完全相同），只影響校正過程中算出的棋盤位置。之後估算路標距離用的是 K 和路標的實際高度，所以真正要量準的是**路標高度**。

---

## 步驟 7：套用結果

```bash
cp ~/calib_images/camera_parameter.yaml ~/ros2_ws/src/robot_vision/config/camera_parameter.yaml
cd ~/ros2_ws && colcon build --packages-select robot_vision_camera
```

然後回到視窗 0，按 `Ctrl-C` 停止相機，再重新執行步驟 2 的 launch 指令。

確認新參數已生效：

```bash
ros2 topic echo /camera/camera_info --once
```

`k:` 欄位應該是新算出來的相機矩陣。之後 AprilTag 或物體辨識都會用到這組參數。

> 記得把 rover 上的 `camera_parameter.yaml` 也複製回 Mac 的 `rubikpi_ros2/robot_vision/config/`，否則下次從 Mac 同步程式碼時會被舊的檔案蓋掉：
>
> ```bash
> scp ubuntu@rover-yang:~/calib_images/camera_parameter.yaml robot_vision/config/
> ```

---

## 注意事項

- 校正完之後**不要改變解析度**。要換解析度就得重新校正，或按比例縮放內參。
- 相機的位置或鏡頭被碰到（例如鏡頭被轉動），也要重新校正。
- 做物體辨識（加分選項）一樣需要這組參數：偵測器輸出的 bounding box 要靠內參換算成方向和距離。

---

## 目前的校正結果（OV5647，1280 × 720）

| 項目 | 值 |
|---|---|
| fx / fy | 648.25 / 648.44（交叉驗證標準差約 1.6） |
| cx / cy | 639.82 / 306.78（標準差約 1.9 / 4.2） |
| 畸變 `[k1 k2 p1 p2 k3]` | `[-0.29729, 0.08772, -0.00128, 0.00054, -0.01165]` |
| RMS 重投影誤差 | 0.717 px（未參與計算的照片：0.838 px） |
| 視角 | 水平 89.3°，垂直 58.1° |
| 使用照片 | 35 張（5 張高誤差照片移到 `~/calib_images/excluded/`） |

參數已套用到 `robot_vision/config/camera_parameter.yaml`，原本的範本備份在 `calibration/camera_parameter_template.yaml`。

## 距離驗證

板子正對鏡頭放在三個距離，用 `calibration/distance_check.py` 估算距離，捲尺量測誤差約 1～2 cm。估算值已換算成實測邊長 2.07 cm。

| 捲尺量測 | 估算距離 | 誤差 |
|---|---|---|
| 30 cm | 30.1 cm | +0.1 cm（+0.3%） |
| 50 cm | 49.9 cm | -0.1 cm（-0.2%） |
| 80 cm | 76.3 cm | -3.7 cm（-4.6%） |

30、50 cm 的結果很好。80 cm 偏短，超過捲尺的量測誤差範圍；若實際距離是 78 cm，誤差約 -2.1%。三個點不足以判斷遠距離是否有系統誤差，之後路標距離要在實際工作距離再檢查一次。
