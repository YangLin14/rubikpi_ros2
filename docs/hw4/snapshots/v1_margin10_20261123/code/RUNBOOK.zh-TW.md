# HW4 操作手冊

你會在 Mac 上產生兩組航點，在 rover 上各跑一次，再把 CSV 拉回 Mac 重建報告。範例資料只用來確認流程，`run` 會拒絕範例地圖。以下 Mac 指令都從 `rubikpi_ros2` 根目錄執行。實測資料、計畫、跑車紀錄和報告都放在 `docs/hw4/work/`，不要放 `/tmp`，macOS 重開機會清掉 `/tmp`。

## 1. 先跑範例

```bash
cd "$HOME/Documents/UCSD/Graduate/Fall26/CSE276A/rubikpi_ros2"
python3 docs/hw4/hw4.py plan --arena docs/hw4/arena.example.json --out /tmp/hw4-example
open /tmp/hw4-example/planned.svg
```

範例的最短距離航線約 **2.545 m／10.4 cm 淨空**，安全航線約 **3.081 m／46.0 cm 淨空**。

## 2. 佈置並量測場地

1. 用膠帶標出 10 ft × 10 ft（3.048 m × 3.048 m）場地，選一角為原點，沿兩個邊標 +x、+y。在對角方向標起點和終點。起點車身中心和朝向用膠帶固定，兩次都從同一位置、同一角度出發。
2. 在場地中央放 1–2 ft 見方的箱子，讓箱子的邊和場地的座標軸平行。量出 `[xmin, ymin, xmax, ymax]`。有獨立凸出物時，將其占地也記為障礙物。
3. 沿邊擺 HW2 的水瓶、衛生紙、麥片盒、杯中的玩偶和 tag 0–3，量物件底部中心和 tag 黑框中心。盡量讓兩條路線上都有清楚的正前方地標，並將地標放在膠帶外。
   箱子四面各貼一張 tag，朝外，id 0–3 可以和場邊重複，對應題目圖中箱子旁的 LM1–4（範例：右 0、上 1、下 2、左 3）。位置填黑框中心，也就是箱面上的點。重複的 id 在 JSON 用別的名字並註明 class，例如 `"box_tag_0": {"class": "tag_0", "position": [...]}`，參考 `arena.example.json`。EKF 會用 NIS 判斷看到的是哪一張；兩張都說得通時就跳過這次觀測。同 id 的兩張至少相隔 1 m，`plan` 會對太近的組合發出警告。
   EKF 會融合物件和所有 tag。實際沒放的地標請從 JSON 刪除。
4. 量車身轉動中心到所有外伸部分的最大距離，包括鏡頭和輪子，填入 `robot_radius_m`。保留 `tracking_margin_m` 作為控制誤差的空間，它同時是到點容許誤差：EKF 判斷偏離航點超過這個值，車會先轉向補正，仍不行才中止。HW3 實測定位誤差約 10–15 cm，若低速試跑顯示偏移過大，提高到 0.15 並重新產生兩條路徑。
5. 複製範例並填上實測值。所有長度用公尺，起點格式為 `[x, y, theta]`，`theta` 用弧度，+x 為 0，朝 +y 為 π/2。

```bash
mkdir -p docs/hw4/work
cp -n docs/hw4/arena.example.json docs/hw4/work/arena.measured.json
open -e docs/hw4/work/arena.measured.json
```

改好每個量測欄位後將 `kind` 改成 `measured`，然後產生航點。

```bash
python3 docs/hw4/hw4.py plan --arena docs/hw4/work/arena.measured.json --out docs/hw4/work/plan
open docs/hw4/work/plan/planned.svg
```

把圖與現場對照。確認路線繞過箱子，檢查各路線和實體牆面、地標的距離。每條航線的第一列是起始姿態，後續列是行駛目標。若兩條結果重合，先檢查起終點、箱子和邊界是否限制了最大淨空，並在報告說明。

## 3. 部署到 rover

以下沿用 HW2/HW3 的 `ubuntu@rover-yang` 和 `~/ros2_ws`，主機名稱不同就換成你的 SSH 位址。

**Mac 終端機**

```bash
ssh ubuntu@rover-yang 'mkdir -p ~/hw4/plan ~/ros2_ws/src'
rsync -az robot_control/ ubuntu@rover-yang:~/ros2_ws/src/robot_control/
rsync -az docs/hw4/hw4.py ubuntu@rover-yang:~/hw4/
rsync -az docs/hw4/work/plan/ ubuntu@rover-yang:~/hw4/plan/
ssh ubuntu@rover-yang
```

**Rover 終端機**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select robot_control
source install/setup.bash
ros2 pkg prefix robot_control
ros2 pkg prefix landmark_detector
ros2 pkg prefix robot_vision_camera
```

如果 rover 已有兩個視覺套件，繼續下一節。若缺套件，從 Mac 同步後，在已載入 ROS 環境的 rover 重建。視覺套件的系統依賴與相機校正沿用 HW2/HW3。 [故障排除](../troubleshooting.md)

```bash
# Mac
rsync -az landmark_detector/ ubuntu@rover-yang:~/ros2_ws/src/landmark_detector/
rsync -az robot_vision/ ubuntu@rover-yang:~/ros2_ws/src/robot_vision/
# Rover
cd ~/ros2_ws
colcon build --packages-select robot_vision_camera landmark_detector robot_control
source install/setup.bash
```

## 4. 啟動 ROS 2，靜止驗證定位

在三個 rover SSH 終端機中，各自先執行下列兩行。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**A，攝影機與地標**

```bash
ros2 launch landmark_detector camera_detector_launch.py
```

**B，馬達與 IMU**

```bash
ros2 run robot_control motor_control
```

**C，先檢查資料與位置估計**

```bash
ros2 topic hz /imu_data
# 看到穩定資料後按 Ctrl-C，再查地標
ros2 topic echo /landmarks/observations --once
# 車放在標好的起點，保持靜止
python3 ~/hw4/hw4.py run --plan ~/hw4/plan --mode safety --look-only \
  --out ~/hw4/check_01
```

查看是否讀到新地標位置、`sightings used` 是否增加、估計位置與朝向是否接近卷尺值。`ambiguous` 應維持 0 或很少；`on a tag` 是被濾掉的、落在 tag 上的 YOLO 誤判。按 Ctrl-C 結束。再次檢查要換新輸出目錄。若觀測一直被拒絕，先查世界座標軸、起始角度、地標名稱、量測位置與可見性。

## 5. 跑兩次並錄影

讓場內淨空，先錄安全路線。從上方或斜上方拍到整個場地、箱子、起點、終點和車。有人守在停止控制旁。實車若朝障礙物偏離、地標有誤或發生碰撞，立即停止。控制器用 EKF 判斷是否到點，執行時仍須現場看顧。先做一趟安全試車確認，再錄正式結果。

在終端機 C 執行。每次保留一個獨立的新目錄。終端機 A、B 繼續跑。

```bash
python3 ~/hw4/hw4.py run --plan ~/hw4/plan --mode safety --drive \
  --out ~/hw4/safety_01
# 量車身中心離終點的實際距離，記下有無接觸障礙物或跨出邊界。
# 把車放回同一個起始位置和角度後，再錄第二次。
python3 ~/hw4/hw4.py run --plan ~/hw4/plan --mode distance --drive \
  --out ~/hw4/distance_01
```

若終端機出現 `ABORT`，保留紀錄和失敗原因。修正地圖、擺位或地標可見性後，用新目錄重跑。地圖變動後重新產生兩條計畫，同一組比較要用同一份計畫。正式影片各上傳一支，確認助教能開啟連結。

## 6. 分析並重建報告

**Mac**

```bash
mkdir -p docs/hw4/work/runs
rsync -az ubuntu@rover-yang:~/hw4/safety_01/ docs/hw4/work/runs/safety/
rsync -az ubuntu@rover-yang:~/hw4/distance_01/ docs/hw4/work/runs/distance/
python3 docs/hw4/hw4.py analyze --plan docs/hw4/work/plan \
  --safety docs/hw4/work/runs/safety --distance docs/hw4/work/runs/distance \
  --out docs/hw4/work/analysis
open docs/hw4/work/analysis/executed.svg
python3 docs/hw4/hw4.py report --plan docs/hw4/work/plan \
  --results docs/hw4/work/analysis --out docs/hw4/work/report \
  --distance-video 'https://youtu.be/DISTANCE_ID' \
  --safety-video 'https://youtu.be/SAFETY_ID'
```

把影片 ID 換成實際連結。在生成的 `docs/hw4/work/report/HW4_Report.md` 寫入兩趟卷尺停點誤差、有無碰撞及實車差異的解釋。分析圖的虛線是 EKF 推估軌跡。卷尺或影片觀察仍要記在報告中。將 Markdown 匯出為 PDF 時，兩張 SVG 請與 Markdown 放在同一個目錄。

在 Mac 用下列指令匯出 PDF。若目前缺少 `rsvg-convert` 或 ReportLab，先執行區塊內對應的安裝指令。

```bash
# 只在缺少 rsvg-convert 時執行
# brew install librsvg
# 只在缺少 ReportLab 時執行
# python3 -m venv /tmp/hw4-pdf-venv
# source /tmp/hw4-pdf-venv/bin/activate
# python3 -m pip install reportlab
python3 docs/hw4/export_pdf.py docs/hw4/work/report/HW4_Report.md docs/hw4/work/HW4_Report.pdf
open docs/hw4/work/HW4_Report.pdf
```

## 7. 打包提交

提交 PDF 報告及完整程式碼 ZIP。下列從 Mac 的 repo 根目錄執行，會打包 HW4 腳本、相關套件、模型、自主導航程式，以及 `docs/hw4/work` 裡的實測場地、計畫與跑車紀錄。範例報告排除在程式碼 ZIP 外。ZIP 放在 repo 外一層，和 `hw3_code.zip` 同一個資料夾。

```bash
zip -r ../hw4_code.zip robot_control landmark_detector robot_vision docs/hw4 docs/troubleshooting.md \
  -x '*/__pycache__/*' '*/.DS_Store' '*/training/runs/*' 'docs/hw4/draft/*' '*.pdf'
unzip -t ../hw4_code.zip
```

如果同名 ZIP 已經存在，另選一個檔名重新打包，以免保留先前版本的檔案。
