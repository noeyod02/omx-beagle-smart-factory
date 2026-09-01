#!/usr/bin/env bash
# Bring the whole demo cell up, on both machines, from nothing.
#
# The cell is eleven processes across two PCs, a Docker container and a
# browser, and the order between some of them is not a preference:
#
#   1. MQTT broker first - the dashboard's backend and the bridge both refuse
#      to publish without it, and failing that way looks like a robot fault.
#   2. The Beagle bridge BEFORE the arms. Its dongle scan walks every serial
#      device on the machine, and doing that while the arms hold theirs
#      severs both Dynamixel buses (2026-08-06, twice).
#   3. Arms, then their task managers - a task manager with no controller
#      parks itself after 30 s of waiting.
#   4. Vision and the dashboard last: they only read.
#
# What this script deliberately does NOT start: the robot-side relay
# (`start_relay:=false`). The dashboard's backend conducts this cell, and two
# conductors put two transfers into one task manager, where the second
# cancels the first mid-trajectory.
#
# Usage:  ./demo_up.sh            # everything
#         ./demo_up.sh --status   # just report what is up
#
# Safe to re-run: every step stops its own leftovers first. That matters more
# than it sounds - a second task manager for the same station takes the same
# job and the two fight over the arm, which reads as "the robot ignored the
# dashboard" (2026-08-31).
# -u는 쓰지 않는다: ROS의 setup.bash가 미설정 변수를 참조해서, 켜 두면
# 환경을 읽는 순간 스크립트가 죽는다.
set -o pipefail

PC2=10.101.49.215
REPO=/home/itec/open_manipulator
WS=/home/itec/ros2_ws
PC2_WS=/home/itec/ros2_ws
DEMO="$REPO/open_manipulator_playground/config/demo"
LOGS=/home/itec/demo-logs
HW=/home/itec/kim/hw/Hardware
BE=/home/itec/kim/be/Backend
FE=/home/itec/kim/fe/Frontend/src/dashboard-frontend

# The arm boards are addressed by their own serial numbers: /dev/ttyACM0 moves
# between them whenever anything is re-plugged, and pointing a station at the
# wrong board is not a startup error, it is a taught point landing somewhere
# else. PC2's board was replaced on 2026-08-31; keep these in step with the
# hardware, not with the file's history.
PORT_C=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_220A5AA95157375037202020FF0D2116-if00

mkdir -p "$LOGS"

ros_env() {
  # The three lines without which nothing on this cell talks to anything.
  # The container defaults to zenoh on domain 30, which is invisible here.
  echo 'unset RMW_IMPLEMENTATION; export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4'
}

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()  { printf '   ok   %s\n' "$*"; }
bad() { printf '   FAIL %s\n' "$*"; }

# Kill by a pattern assembled at runtime. Written out because the obvious
# `pkill -f foo` also matches this very script when the pattern is spelled in
# it - which killed the running shell six times on 2026-08-28.
stop_local() {
  local pat="$1" pids
  pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
  [ -n "$pids" ] && kill $pids 2>/dev/null && sleep 2
  pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  return 0
}

TM_PAT=$(printf '%s%s' 'stock_task_man' 'ager_node')
ARRIVAL_PAT=$(printf '%s%s' 'stock_arri' 'val_node')
MONITOR_PAT=$(printf '%s%s' 'stock_moni' 'tor_node')
RELAY_PAT=$(printf '%s%s' 'stock_rel' 'ay_node')

status() {
  say "PC1"
  for entry in \
      "MQTT 브로커:1883" "백엔드:8000" "대시보드:5173" \
      "창고 카메라:8899" "A라인 스트림:8898"; do
    name=${entry%%:*}; port=${entry##*:}
    ss -ltn 2>/dev/null | grep -q ":$port " && ok "$name (:$port)" || bad "$name (:$port)"
  done
  pgrep -f "$(printf '%s%s' 'mqtt_bri' 'dge/lib')" >/dev/null && ok "MQTT 브리지" || bad "MQTT 브리지"
  pgrep -f "$ARRIVAL_PAT" >/dev/null && ok "도착·준비 감시" || bad "도착·준비 감시"
  docker ps --format '{{.Names}}' | grep -q open_manipulator && ok "비글 컨테이너" || bad "비글 컨테이너"

  say "PC2"
  ssh "$PC2" "pgrep -f $MONITOR_PAT >/dev/null" && ok "재고 카메라 판정" || bad "재고 카메라 판정"
  ssh "$PC2" "ss -ltn | grep -q ':8899 '" && ok "전체뷰 카메라" || bad "전체뷰 카메라"

  say "로봇"
  # shellcheck disable=SC1090
  source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
  export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  for ns in station_a station_b station_c; do
    st=$(timeout 6 ros2 topic echo "/$ns/stock/task_state" --once 2>/dev/null |
         tr -d '\n' | grep -o '"state": "[a-z]*"' | cut -d'"' -f4)
    [ -n "$st" ] && ok "$ns: $st" || bad "$ns: 응답 없음"
  done
  bg=$(timeout 6 ros2 topic echo /beagle/state --once 2>/dev/null | tr -d '\n' |
       grep -o '"station": "[a-z_]*"' | cut -d'"' -f4)
  [ -n "$bg" ] && ok "비글: $bg" || bad "비글: 응답 없음"

  # A duplicate task manager answers every check above exactly like a healthy
  # one, and only shows itself as jobs that cancel each other.
  # pgrep -fc prints "0" AND exits nonzero when nothing matches, so `|| echo 0`
  # would append a second line and break the -le test below.
  dupes=$(pgrep -fc "install/open_manipulator_playground/lib/open_manipulator_playground/${TM_PAT}" 2>/dev/null)
  dupes=${dupes:-0}
  [ "$dupes" -le 2 ] && ok "태스크 매니저 중복 없음" || bad "태스크 매니저가 $dupes 개 - 중복!"
}

if [ "${1:-}" = "--status" ]; then status; exit 0; fi

# 같은 스크립트가 두 벌 겹쳐 돌면 태스크 매니저가 두 개씩 떠서, 하나의 작업을
# 둘이 나눠 잡고 서로의 궤적을 취소한다 (2026-09-01 실측: 사람과 도우미가 1분
# 간격으로 각자 실행). 그래서 실행 자체를 잠금으로 하나로 제한한다.
exec 200>/tmp/demo_up.lock
if ! flock -n 200; then
  echo "demo_up.sh가 이미 실행 중이다 - 그쪽이 끝나기를 기다릴 것. (두 벌이 겹치면 태스크 매니저가 중복된다)"
  exit 1
fi

say "0/7  이전 프로세스 정리"
stop_local "$RELAY_PAT"; stop_local "$TM_PAT"; stop_local "$ARRIVAL_PAT"
stop_local "$(printf '%s%s' 'mqtt_bri' 'dge/lib')"
stop_local "$(printf '%s%s' 'ros_mjpeg' '_server')"
stop_local "$(printf '%s%s' 'cctv_ser' 'ver.py')"
stop_local "app.main:app"
# launch 래퍼와 상태 퍼블리셔까지 지운다. ros2_control_node만 죽이면 launch가
# 고아 자식(robot_state_publisher)을 남기고, 다음 기동과 이름이 겹친다.
stop_local "$(printf '%s%s' 'omx_stock_re' 'lay.launch')"
stop_local "ros2_control_node"
stop_local "robot_state_publisher"
# 프론트(vite)도 지운다 - 안 지우면 새 vite가 5174로 조용히 비켜 앉아,
# 화면은 5173의 낡은 코드를 계속 보여준다.
stop_local "node_modules/.bin/vite"
ssh "$PC2" "pkill -f $MONITOR_PAT; pkill -f $TM_PAT; pkill -f omx_station_c; \
  pkill -f ros2_control_node; pkill -f robot_state_publisher; pkill -f cctv_server; true" >/dev/null 2>&1
sleep 2; ok "정리 완료"

say "1/7  MQTT 브로커 + 컨테이너"
docker start t1be-mosquitto >/dev/null 2>&1
docker start open_manipulator >/dev/null 2>&1   # 비글 브리지가 이 안에서 돈다
sleep 1
ss -ltn | grep -q ':1883 ' && ok "1883 대기 중" || bad "브로커가 안 떴다"
docker ps --format '{{.Names}}' | grep -q open_manipulator && ok "비글 컨테이너" || bad "비글 컨테이너가 안 떴다"

say "2/7  비글 브리지 (팔보다 먼저)"
docker exec open_manipulator bash -c "pkill -f beagle_bridge_node; true" >/dev/null 2>&1
docker exec -d open_manipulator bash -c "$(ros_env); source /opt/ros/jazzy/setup.bash; \
  nohup python3 /root/ros2_ws/src/open_manipulator/open_manipulator_playground/scripts/beagle_bridge_node.py \
  --ros-args -p route_file:=/root/ros2_ws/src/open_manipulator/open_manipulator_playground/config/beagle_route.yaml \
  -p dry_run:=false -p start_station:=station_a > /tmp/bridge.log 2>&1 &"
printf '   비글 접속 대기'
for _ in $(seq 1 30); do
  if docker exec open_manipulator grep -q 'gyro bias' /tmp/bridge.log 2>/dev/null; then
    echo; ok "비글 연결·자이로 보정 완료"; break
  fi
  if docker exec open_manipulator grep -q 'could not connect' /tmp/bridge.log 2>/dev/null; then
    echo; bad "비글이 응답하지 않는다 - 본체 전원과 배터리, BLE 동글을 확인할 것"; break
  fi
  printf '.'; sleep 5
done

say "3/7  PC1 팔 2대 (A, B) + 태스크 매니저"
# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
nohup ros2 launch open_manipulator_playground omx_stock_relay.launch.py \
  start_beagle:=false start_camera:=false start_monitor:=false start_arrival:=false \
  start_relay:=false warehouse_cycles:=true > "$LOGS/arms_pc1.log" 2>&1 &
disown
# 'Controllers ready'는 태스크 매니저 하나당 한 줄이다. 두 대(A·B)가 다 떠야
# 성공이다 - 첫 줄에서 끝내면 B의 태스크 매니저가 죽어도 성공으로 보인다
# (2026-09-01 실측: B만 launch 인자 타입 버그로 매번 죽는데 기동은 ok로 표시).
for _ in $(seq 1 24); do
  ready=$(grep -c 'Controllers ready' "$LOGS/arms_pc1.log" 2>/dev/null)
  [ "${ready:-0}" -ge 2 ] && break || sleep 5
done
ready=$(grep -c 'Controllers ready' "$LOGS/arms_pc1.log" 2>/dev/null)
[ "${ready:-0}" -ge 2 ] && ok "A·B 기동 (태스크 매니저 ${ready}개)" \
  || bad "A·B 기동 불완전 (태스크 매니저 ${ready:-0}/2) - $LOGS/arms_pc1.log"

say "4/7  PC2 팔 (C) + 태스크 매니저"
ssh -f "$PC2" "source /opt/ros/jazzy/setup.bash && source $PC2_WS/install/setup.bash && \
  export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
  nohup ros2 launch open_manipulator_playground omx_station_c.launch.py port_c:=$PORT_C \
  > /tmp/station_c.log 2>&1 &"
for _ in $(seq 1 20); do
  ssh "$PC2" "grep -q 'Configured and activated joint_state_broadcaster' /tmp/station_c.log" 2>/dev/null && break || sleep 4
done
ssh "$PC2" "grep -q 'Failed to initialize' /tmp/station_c.log" 2>/dev/null \
  && bad "C팔 하드웨어 초기화 실패 - 서보 오류 래치일 수 있다 (scripts/dxl_clear_errors.py)" \
  || ok "C팔 기동"
ssh -f "$PC2" "source /opt/ros/jazzy/setup.bash && source $PC2_WS/install/setup.bash && \
  export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
  nohup ros2 run open_manipulator_playground stock_task_manager_node.py --ros-args \
  -r __node:=stock_task_manager -r __ns:=/station_c \
  --params-file $PC2_WS/src/open_manipulator/open_manipulator_playground/config/demo/station_c_tm.yaml \
  > /tmp/tm_c.log 2>&1 &"
sleep 6; ssh "$PC2" "grep -q 'accepting refill' /tmp/tm_c.log" && ok "C 태스크 매니저" || bad "C 태스크 매니저"

say "5/7  카메라 3대"
nohup python3 "$HW/scripts/cctv_server.py" \
  --camera cam-warehouse=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:10:1.0-video-index0 \
  --port 8899 --fps 15 > "$LOGS/cctv_pc1.log" 2>&1 &
disown
ssh -f "$PC2" "nohup python3 /home/itec/cctv_server.py \
  --camera cam-overview=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:9:1.0-video-index0 \
  --port 8899 --fps 15 > /tmp/cctv_server.log 2>&1 &"
ssh -f "$PC2" "source /opt/ros/jazzy/setup.bash && source $PC2_WS/install/setup.bash && \
  export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
  nohup ros2 run open_manipulator_playground stock_monitor_node.py --ros-args \
  --params-file $PC2_WS/src/open_manipulator/open_manipulator_playground/config/demo/stock_monitor.yaml \
  > /tmp/stock_monitor.log 2>&1 &"
sleep 6
nohup ros2 run open_manipulator_playground stock_arrival_node.py --ros-args \
  --params-file "$DEMO/arrival.yaml" > "$LOGS/arrival.log" 2>&1 &
disown
nohup python3 "$HW/scripts/ros_mjpeg_server.py" > "$LOGS/mjpeg.log" 2>&1 &
disown
# 카메라 서버는 뜬 뒤에도 장치 열기~첫 프레임까지 몇 초가 더 걸린다. 한 번에
# 찔러보면 정상인데도 FAIL로 읽힌다 (2026-09-01 검증 주행에서 오탐 2건).
wait_stream() {
  local url="$1" name="$2"
  for _ in $(seq 1 8); do
    curl -sf -m 4 -o /dev/null "$url" && { ok "$name"; return 0; }
    sleep 3
  done
  bad "$name"
}
wait_stream http://127.0.0.1:8899/cam/cam-warehouse.mjpg "창고 카메라"
wait_stream "http://$PC2:8899/cam/cam-overview.mjpg" "전체뷰 카메라"
ssh "$PC2" "grep -q 'Stock monitor started' /tmp/stock_monitor.log" && ok "재고 판정" || bad "재고 판정"

say "6/7  대시보드 (백엔드·프론트·MQTT 브리지)"
(cd "$BE" && nohup env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/uvicorn app.main:app \
  --host 0.0.0.0 --port 8000 > "$LOGS/backend.log" 2>&1 &)
(cd "$FE" && nohup npm run dev -- --host 0.0.0.0 > "$LOGS/frontend.log" 2>&1 &)
PYTHONPATH="$WS/install/mqtt_bridge/lib/python3.12/site-packages:${PYTHONPATH:-}" \
  nohup "$WS/install/mqtt_bridge/lib/mqtt_bridge/bridge_node" > "$LOGS/mqtt_bridge.log" 2>&1 &
disown
sleep 10
curl -sf -m 4 -o /dev/null http://127.0.0.1:8000/health && ok "백엔드" || bad "백엔드"
curl -sf -m 4 -o /dev/null http://127.0.0.1:5173/ && ok "대시보드" || bad "대시보드"
grep -q '브리지 시작' "$LOGS/mqtt_bridge.log" && ok "MQTT 브리지" || bad "MQTT 브리지"

say "7/7  점검"
status

cat <<'NOTE'

시연 전 확인 두 가지
  - 창고 박스에 부품을 세워서 넣을 것 (눕히면 카메라가 못 본다)
  - 재고함 네 칸이 다 차 있을 것 — 그게 시나리오의 시작 상태다

대시보드: http://10.101.49.216:5173
NOTE
