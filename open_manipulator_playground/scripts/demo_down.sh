#!/usr/bin/env bash
# Stop everything demo_up.sh started, on both machines.
#
# Leaves the MQTT broker and the Beagle's container running: the broker holds
# the retained state the dashboard reads on the way back up, and restarting
# the container means waiting through the Beagle's connect and gyro
# calibration again for no reason.
# -u는 쓰지 않는다: ROS의 setup.bash가 미설정 변수를 참조해서, 켜 두면
# 환경을 읽는 순간 스크립트가 죽는다.
set -o pipefail

PC2=10.101.49.215

# Patterns are assembled at runtime: spelling them out would make this script
# match itself, and `pkill -f` would then kill the shell running it.
TM_PAT=$(printf '%s%s' 'stock_task_man' 'ager_node')
ARRIVAL_PAT=$(printf '%s%s' 'stock_arri' 'val_node')
MONITOR_PAT=$(printf '%s%s' 'stock_moni' 'tor_node')
RELAY_PAT=$(printf '%s%s' 'stock_rel' 'ay_node')

stop() {
  local pids
  pids=$(pgrep -f "$1" 2>/dev/null | tr '\n' ' ')
  if [ -n "$pids" ]; then
    kill $pids 2>/dev/null; sleep 1
    pids=$(pgrep -f "$1" 2>/dev/null | tr '\n' ' ')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
    printf '   stopped %s\n' "$2"
  fi
  return 0
}

echo "== PC1"
stop "$RELAY_PAT" "릴레이"
stop "$TM_PAT" "태스크 매니저"
stop "$ARRIVAL_PAT" "도착·준비 감시"
stop "$(printf '%s%s' 'mqtt_bri' 'dge/lib')" "MQTT 브리지"
stop "$(printf '%s%s' 'ros_mjpeg' '_server')" "A라인 스트림"
stop "$(printf '%s%s' 'cctv_ser' 'ver.py')" "창고 카메라"
stop "app.main:app" "백엔드"
stop "vite" "대시보드"
stop "$(printf '%s%s' 'omx_stock_re' 'lay.launch')" "팔 launch"
stop "ros2_control_node" "팔 컨트롤러"
stop "robot_state_publisher" "상태 퍼블리셔"
stop "$(printf '%s%s' 'all_cams' '_view')" "로컬 카메라 창"

echo "== PC2"
# 원격 패턴은 대괄호로 자기-매칭을 끊는다 - 변수로 짜 넣으면 원격 셸의
# 명령줄에 패턴이 실려, 첫 pkill이 그 셸부터 죽이고 나머지는 산 채로 남는다
# (2026-09-01 실측: cctv_server와 station_c 전부가 그렇게 살아남았다).
ssh "$PC2" "pkill -f 'stock_monitor_nod[e]'; pkill -f 'stock_task_manager_nod[e]'; \
  pkill -f 'omx_station_[c]'; pkill -f 'ros2_control_nod[e]'; \
  pkill -f 'robot_state_publishe[r]'; pkill -f 'cctv_serve[r]'; true"
echo "   stopped PC2 노드들"

echo
echo "브로커와 비글 컨테이너는 그대로 둔다 (retain 상태 보존, 재연결 시간 절약)."
echo "다시 켜기: scripts/demo_up.sh"
