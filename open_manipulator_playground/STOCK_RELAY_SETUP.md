# 재고 보충 릴레이 — 실물 연결 순서

재고함(OMX-F #1) → 비글 → 목적지(OMX-F #2) 릴레이를 실물에서 세우는 순서입니다.

**한 번에 다 연결하지 마세요.** 장치를 하나씩 붙이고 그때마다 확인하면, 문제가 생겼을 때
방금 붙인 것이 원인이라는 걸 알 수 있습니다. 전부 연결한 뒤에 안 되면 원인을 찾는 데
몇 배가 걸립니다.

각 단계 끝의 **기록** 표를 채워 가면서 진행하세요. 그 값들이 그대로 설정 파일에 들어갑니다.

---

## 0단계 · 컨테이너 준비

```bash
cd ~/open_manipulator
./docker/container.sh enter          # 이미 떠 있으면 진입만
```

컨테이너가 안 떠 있으면 `./docker/container.sh start` 후 `enter`.

> ⚠️ **주의**: `container.sh start`는 `docker compose pull`을 실행합니다. 이미지가 갱신되면
> 아래 패키지가 사라지므로 다시 설치해야 합니다.
>
> ```bash
> apt-get update && apt-get install -y python3-pip
> python3 -m pip install --break-system-packages roboid
> ```
>
> 영구적으로 만들려면 `docker/Dockerfile`에 추가해야 합니다 (아직 안 되어 있음).

컨테이너 안에서는 항상 **zenoh 라우터를 먼저** 띄웁니다. 이게 없으면 노드끼리 서로를
못 찾습니다.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
```

---

## 1단계 · 장치 식별

USB 장치가 여러 개라 `/dev/ttyACM0` 같은 이름은 꽂는 순서에 따라 바뀝니다. **반드시
`by-id` 경로를 쓰세요.**

```bash
ls -l /dev/serial/by-id/
ls -l /dev/video*
```

어느 경로가 어느 장치인지 확실히 하려면 **하나씩 뽑았다 꽂으면서** 목록이 어떻게 변하는지
보는 게 가장 확실합니다.

### 기록

| 장치 | 경로 |
|---|---|
| OMX-F #1 (재고함 쪽) | `/dev/serial/by-id/` |
| OMX-F #2 (목적지 쪽) | `/dev/serial/by-id/` |
| 비글 블루투스 동글 | `/dev/serial/by-id/` |
| 웹캠 | `/dev/video` |

---

## 2단계 · OMX-F 1대 기동

**2대를 동시에 켜지 마세요.** 지금 코드는 노드 이름과 토픽이 겹쳐서 충돌합니다
(6단계 참고). 먼저 1번 팔만 확인합니다.

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py \
    port_name:=/dev/serial/by-id/<1단계에서 기록한 1번 팔>
```

### 확인

새 터미널에서 (`./docker/container.sh enter`로 하나 더 열기):

```bash
ros2 control list_controllers
```

이렇게 나와야 합니다:

```
joint_state_broadcaster  ... active
arm_controller           ... active
gripper_controller       ... active
```

관절 값도 실제로 들어오는지 봅니다:

```bash
ros2 topic echo /joint_states --once
```

`joint1`~`joint5`, `gripper_joint_1`의 `position`이 보이면 성공입니다.

### 안 될 때

| 증상 | 원인 / 조치 |
|---|---|
| `No such file or directory` | 포트 경로 오타. `ls /dev/serial/by-id/`로 다시 확인 |
| `Permission denied` | 호스트에서 `sudo chmod 666 /dev/ttyACM*` 후 재시도 |
| 컨트롤러가 `active`가 아님 | 팔 전원이 꺼져 있거나 다이나믹셀 ID 충돌. 전원과 케이블 확인 |
| 팔이 갑자기 움직임 | 정상입니다. 기동 시 초기 자세로 갑니다. **주변을 비워두세요** |

---

## 3단계 · 좌표 실측 ★가장 중요

`config/stock_layout.yaml`에 들어 있는 좌표는 **전부 임시로 넣은 값입니다.** 이걸 그대로
쓰면 팔이 엉뚱한 곳으로 갑니다. 반드시 실측해서 바꿔야 합니다.

### 좌표계

```
        +x (팔이 뻗는 정면)
         ↑
         |
  +y ←---●---→ -y          ● = 원점: 베이스 판이 테이블에 닿는 면의 중심
 (왼쪽)  |   (오른쪽)       +z = 위쪽 (테이블 면이 z = 0)
         |
```

단위는 **미터**입니다. 15 cm는 `0.15`입니다.

### 측정할 것

| 항목 | 개수 | 무엇을 재나 |
|---|---|---|
| 재고함 각 칸 | 3 | 부품을 **놓을** 지점 (칸 중앙, 바닥에서 부품 절반 높이) |
| 창고 픽업 지점 | 4 | 부품을 **집을** 지점 (부품 중심) |

`z`는 그리퍼가 부품을 잡는 높이입니다. 테이블 위에 놓인 높이 3 cm 부품이면 `z ≈ 0.015`
(중심) 정도입니다.

### 방법 A · 자로 재기 (도구 없이 바로 가능)

베이스 중심에서 각 지점까지 앞뒤(x), 좌우(y), 높이(z)를 mm 단위로 재서 적습니다.
정확도는 떨어지지만 시작하기엔 충분합니다.

### 방법 B · 조그(jog) 도구 ★권장

키보드로 그리퍼를 옮겨 원하는 위치에 정확히 놓고 좌표를 기록합니다. 자로 재는 것보다
정확하고, y 부호를 틀릴 일이 없습니다.

```bash
ros2 run open_manipulator_playground stock_jog.py
```

| 키 | 동작 |
|---|---|
| `w` / `s` | 앞 / 뒤 (+x / -x) |
| `a` / `d` | 왼쪽 / 오른쪽 (+y / -y) |
| `r` / `f` | 위 / 아래 (+z / -z) |
| `[` / `]` | 이동 폭 줄이기 / 키우기 (1·2·5·10·20·50 mm) |
| `o` / `c` | 그리퍼 열기 / 닫기 |
| `p` | 현재 좌표를 기록 |
| `h` | 시작 위치로 복귀 |
| `q` | 종료 (기록한 좌표를 한꺼번에 출력) |

닿을 수 없는 곳으로 가려 하면 **움직이지 않고 이유를 알려줍니다.** 종료하면 기록한
좌표가 YAML에 그대로 붙여넣을 수 있는 형태로 나옵니다.

> `stock_teach.py`(손으로 끌어서 티칭)는 **쓸 수 없습니다.** 컨트롤러가 떠 있으면
> 서보가 자세를 유지해서 손으로 안 움직입니다. 조그 도구를 쓰세요.

### 입력

`open_manipulator_playground/config/stock_layout.yaml`을 열어 `warehouse.pick_points`와
`bins[].place`를 실측값으로 바꿉니다.

### 반드시 검사

```bash
cd /root/ros2_ws/src/open_manipulator/open_manipulator_playground/scripts
python3 stock_reach_check.py ../config/stock_layout.yaml
```

`all points reachable with margin`이 나와야 합니다. `FAIL`이 나오면 그 지점은 팔이 못
닿습니다 — 재고함을 로봇 쪽으로 당기거나 높이를 낮추세요.

> 팔을 수직 아래로 향하면 손목이 목표점보다 12 cm 위에 있어야 해서 도달 범위가 크게
> 줄어듭니다. z=0.03 m에서는 반경 0.27 m까지 되지만, z=0.15 m에서는 0.22 m,
> z=0.21 m에서는 0.16 m밖에 안 됩니다. 검사 도구가 이 표를 출력해 줍니다.

### 동작 확인

좌표를 넣었으면 실제로 한 번 시켜 봅니다. **비상 정지할 준비를 하고** (Ctrl+C 또는 전원)
진행하세요.

```bash
ros2 run open_manipulator_playground stock_task_manager_node.py --ros-args \
    -p layout_file:=<stock_layout.yaml 경로> -p auto_refill:=false

# 다른 터미널에서
ros2 topic pub --once /stock/refill_request std_msgs/String "{data: bin1}"
```

창고 → bin1으로 부품을 옮기는 12단계가 실행됩니다. 처음에는 **부품 없이** 빈 손으로
동작만 보세요.

### 기록

| 지점 | x | y | z |
|---|---|---|---|
| bin1 놓는 곳 | | | |
| bin2 놓는 곳 | | | |
| bin3 놓는 곳 | | | |
| 창고 1 | | | |
| 창고 2 | | | |
| 창고 3 | | | |
| 창고 4 | | | |

---

## 4단계 · 웹캠

### 물리 설치

- 재고함 위 **50~70 cm 높이에서 수직 하방**
- 카메라를 **테이블/프레임에 고정** (로봇에 붙이면 진동이 실립니다)
- 팔이 홈 자세일 때 재고함을 가리지 않는지 확인
- 오토포커스·오토노출 끄기 (초점이 변하면 보정이 깨집니다)

### 기동

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
    video_device:=/dev/video0
ros2 topic hz /camera1/image_raw        # 30 근처가 나오면 정상
```

### 칸 영역 지정

```bash
ros2 run open_manipulator_playground stock_calibrate.py --ros-args \
    -p layout_file:=<stock_layout.yaml 경로>
```

창이 뜨면 재고함 칸을 순서대로 드래그합니다. `n` 다음 칸, `r` 다시, `s` 저장, `q` 취소.
저장하면 YAML의 `roi` 값이 자동으로 바뀝니다 (주석은 그대로 유지됩니다).

### 기준 사진 (모델 없이 인식하기)

**재고함을 완전히 비운 상태로** 찍습니다. 조명도 실제 운영할 상태로 맞춰 두세요.

```bash
ros2 topic pub --once /stock/capture_reference std_msgs/Empty {}
```

### 확인

```bash
ros2 topic echo /stock/status
```

부품을 넣었다 뺐다 하면서 `state`가 `filled` ↔ `empty`로 바뀌는지 봅니다. 안 바뀌면
`occupancy_threshold`(기본 0.08)를 조정하세요 — 값을 낮추면 더 민감해집니다.

### 통합 실행

```bash
ros2 launch open_manipulator_playground omx_stock.launch.py \
    port_name:=/dev/serial/by-id/<1번 팔> video_device:=/dev/video0
```

칸을 비우면 자동으로 보충하러 갑니다.

---

## 5단계 · 비글

### 연결 확인

동글을 꽂고 비글 전원을 켠 뒤:

```bash
python3 -c "
from roboid import Beagle
b = Beagle()
b.start_lidar(); b.wait_until_lidar_ready()
print('front:', b.front_lidar(), 'mm')
print('battery:', b.battery_state())
b.dispose()
"
```

거리 값이 나오면 연결된 것입니다. 안 되면 동글이 인식됐는지(`ls /dev/serial/by-id/`),
비글 전원이 켜졌는지 확인하세요.

### 엔코더 보정 ★필수

**이걸 안 하면 이동 거리가 전부 틀어집니다.** 비글 엔코더의 단위를 모르기 때문에
실측으로 환산 계수를 구해야 합니다.

바닥에 줄자를 놓고 비글을 출발선에 맞춘 뒤:

```bash
python3 -c "
from roboid import Beagle
import time
b = Beagle()
l0, r0 = b.left_encoder(), b.right_encoder()
b.wheels(20, 20); time.sleep(3.0); b.stop(); time.sleep(0.5)
dl = b.left_encoder() - l0
dr = b.right_encoder() - r0
print('엔코더 변화량:', (dl + dr) / 2)
b.dispose()
"
```

자로 **실제 이동한 거리를 mm로** 재고:

```
encoder_scale = 실제이동거리_mm / 엔코더변화량
```

이 값을 `config/beagle_route.yaml`의 `robot.encoder_scale`에 넣습니다.
(출력값이 이미 mm라서 실제 거리와 거의 같으면 `1.0` 그대로 두면 됩니다.)

### 경로 측정

두 스테이션 사이를 비글이 어떻게 가야 하는지 재서 `beagle_route.yaml`의 `routes`를
고칩니다. 지금 들어 있는 값(1.2 m 직진 등)은 임시값입니다.

각 경로는 **반드시 `approach` → `square`로 끝나야 합니다.** 이 두 단계가 그동안 쌓인
주행 오차를 벽 기준으로 리셋해 주기 때문에, 팔이 매번 같은 자리에서 트레이를 찾을 수
있습니다.

```yaml
routes:
  station_a->station_b:
    - {action: backward, distance_m: 0.15}   # 도킹에서 빠져나오기
    - {action: turn, degrees: -90}           # 양수 = 반시계
    - {action: forward, distance_m: 1.20}    # ← 실측값으로
    - {action: turn, degrees: 90}
    - {action: approach, target_mm: 150}     # 벽까지 15 cm
    - {action: square, tolerance_mm: 15}     # 벽과 수직 맞추기
```

### 실물 주행

**처음에는 반드시 시뮬레이터로 확인**한 뒤 실물로 넘어가세요.

```bash
# 1) 시뮬레이터 (비글 없이)
ros2 run open_manipulator_playground beagle_bridge_node.py --ros-args \
    -p dry_run:=true -p route_file:=<beagle_route.yaml 경로>

# 2) 실물
ros2 run open_manipulator_playground beagle_bridge_node.py --ros-args \
    -p dry_run:=false -p route_file:=<beagle_route.yaml 경로>

# 출발 지시
ros2 topic pub --once /beagle/goto std_msgs/String "{data: station_b}"

# 비상 정지
ros2 topic pub --once /beagle/estop std_msgs/Bool "{data: true}"
```

상태 확인:

```bash
ros2 topic echo /beagle/state
```

`ready_for_arm: true`는 **비글이 알려진 스테이션에 정상 도킹해 있다**는 뜻입니다.
이때만 팔이 트레이에 손을 뻗어도 됩니다. 도킹이 실패하면 `state: error`가 되고
`ready_for_arm`은 `false`로 남습니다.

### 도킹 정밀도 ★가장 큰 난관

주행으로 맞출 수 있는 정밀도는 cm 단위인데, 그리퍼는 mm를 요구합니다. **물리적인
도킹 지그(V자 가이드)를 만드는 것이 사실상 필수입니다.** 비글이 밀려 들어가면서
기계적으로 자리를 잡게 하는 방식이고, 이게 없으면 소프트웨어로는 한계가 있습니다.

### 기록

| 항목 | 값 |
|---|---|
| encoder_scale | |
| A→B 직진 거리 (m) | |
| A→B 회전 각도 (도) | |
| 도킹 벽까지 거리 (mm) | |

---

## 6단계 · OMX-F 2대 동시 (아직 코드 수정 필요)

지금 런치 파일로 2대를 동시에 띄우면 **노드 이름과 토픽이 충돌합니다.** 둘 다
`/arm_controller`, `/joint_states`를 쓰기 때문입니다.

네임스페이스 분리(`/omx_a`, `/omx_b`) 작업이 필요하며 **아직 하지 않았습니다.**
1대씩 검증이 끝나면 알려주세요. 그때 작업하겠습니다.

---

## 7단계 · 전체 미션 연결 (미착수)

A에서 적재 → 비글 이동 → B에서 하역까지 하나로 묶는 오케스트레이터와, 출발 전
사람 승인을 받는 게이트가 아직 없습니다. 위 단계들이 각각 확인된 뒤에 만드는 것이
순서상 맞습니다.

---

## 자주 겪는 문제

| 증상 | 확인할 것 |
|---|---|
| 노드끼리 서로 못 봄 | zenoh 라우터를 띄웠는지 (`ros2 run rmw_zenoh_cpp rmw_zenohd`) |
| `ros2 run`이 실행 파일을 못 찾음 | 스크립트에 실행 권한이 있는지 (`chmod +x`), `colcon build` 후 `source install/setup.bash` |
| 팔이 이상한 곳으로 감 | `stock_reach_check.py`를 돌렸는지, 좌표 부호(+y가 왼쪽)를 맞게 넣었는지 |
| 카메라가 팔에 가려짐 | 재고 판정은 팔이 홈에 있을 때만 수행됩니다. 홈 자세를 카메라 시야 밖으로 옮기세요 |
| 비글이 매번 다른 곳에 섬 | `encoder_scale` 보정을 했는지, 경로가 `approach`+`square`로 끝나는지 |
| 컨테이너 재시작 후 roboid 없음 | 0단계의 재설치 명령 참고 |
