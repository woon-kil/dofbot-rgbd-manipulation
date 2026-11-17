#!/usr/bin/env bash
# P-A 상시 Gemini 드라이버 제어(p620에서 실행, ssh로 stdin 전달). 이 프로젝트 전용 컨테이너 dofbot-g01-pa만 조작. 로봇 명령 없음.
# 사용: ssh p620 "ACTION=start [WARMUP=12] [MAXSEC=1800] bash -s" < scripts/pa_driver.sh      (ACTION = start | stop | status)
# 드라이버 인자·Gemini 점유 검사·종료 대기(INT 후 4 s, 남으면 pkill 후 2 s)·정리는 원본 g01_gemini_check.sh와 같다. 컨테이너는 MAXSEC+60 s 뒤 스스로 끝나고 드라이버는 MAXSEC s 뒤 INT로 종료된다(최대 가동 상한).
set -euo pipefail
ACTION="${ACTION:?ACTION 필요(start|stop|status)}"
NAME=dofbot-g01-pa
IMAGE=cad-less-robotics:humble-p003
VOL=bytesize-robotics-camera-install
WORK="$HOME/dofbot-g01-work"
PADIR="$WORK/pa"
STATE="$PADIR/state.txt"
MAXSEC="${MAXSEC:-1800}"
WARMUP="${WARMUP:-12}"
running() { docker ps --format '{{.Names}}' | grep -q "^${NAME}$"; }
exists() { docker ps -a --format '{{.Names}}' | grep -q "^${NAME}$"; }
stop_all() {
  if running; then
    docker exec "$NAME" bash -lc 'pkill -INT -f "[o]rbbec_camera_node|[g]emini_330_series" >/dev/null 2>&1 || true' >/dev/null 2>&1 || true
    sleep 4
    docker exec "$NAME" bash -lc 'pgrep -f "[o]rbbec_camera_node" >/dev/null && { pkill -f "[o]rbbec_camera_node" >/dev/null 2>&1 || true; sleep 2; } || true' >/dev/null 2>&1 || true
    docker exec -u 0 "$NAME" chown -R "$(id -u):$(id -g)" /work >/dev/null 2>&1 || true
  fi
  if exists; then docker rm -f "$NAME" >/dev/null 2>&1 || true; fi
  rm -f "$STATE" 2>/dev/null || true
  echo "컨테이너 정리: $(docker ps -a --format '{{.Names}}' | grep -c "^${NAME}$" || true)개 남음"
}
case "$ACTION" in
  stop) stop_all; exit 0 ;;
  status)
    if running; then
      echo "컨테이너 실행 중"
      docker exec "$NAME" bash -lc 'pgrep -f "[o]rbbec_camera_node" >/dev/null && echo "드라이버 프로세스 있음" || echo "드라이버 프로세스 없음"' 2>/dev/null || true
      if [ -f "$STATE" ]; then cat "$STATE"; s="$(awk '$1=="started_epoch"{print $2}' "$STATE")"; [ -n "$s" ] && echo "경과 $(( $(date +%s) - ${s%.*} )) s"; fi
    elif exists; then echo "컨테이너 종료됨(상한 도달 등), 정리는 stop"
    else echo "컨테이너 없음"; fi
    exit 0 ;;
  start) ;;
  *) echo "알 수 없는 ACTION: $ACTION"; exit 2 ;;
esac
if exists; then echo "이미 $NAME 컨테이너가 있다. 먼저 stop"; exit 5; fi
OK=0
trap 'rc=$?; if [ "$OK" != 1 ]; then stop_all; fi' EXIT
line="$(lsusb -d 2bc5:0804 | head -n1)"; [ -n "$line" ] || { echo "Gemini 없음"; exit 4; }
node="/dev/bus/usb/$(echo "$line" | awk '{print $2}')/$(echo "$line" | awk '{print $4}' | tr -d ':')"
if fuser "$node" >/dev/null 2>&1; then echo "USB 사용 중: $node"; exit 3; fi
for n in $(docker ps --format '{{.Names}}'); do
  d="$(docker inspect --format '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$n")"
  if [[ "$d" == *"$node"* ]]; then echo "다른 컨테이너가 점유: $n"; exit 3; fi
done
docker volume inspect "$VOL" >/dev/null
mkdir -p "$PADIR"; : > "$PADIR/driver.log"
docker run -d --name "$NAME" --device "$node" -v "$WORK:/work" -v "$VOL:/opt/cad-less-install:ro" -e HOME=/tmp "$IMAGE" sleep "$((MAXSEC + 60))" >/dev/null
docker exec -d "$NAME" bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /opt/cad-less-install/setup.bash; set -u; exec timeout -s INT -k 15 '"$MAXSEC"' ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=false enable_colored_point_cloud:=false enable_publish_extrinsic:=true > /work/pa/driver.log 2>&1'
ok=0; for _ in $(seq 1 90); do grep -q "Serial number:" "$PADIR/driver.log" 2>/dev/null && { ok=1; break; }; sleep 1; done
[ $ok -eq 1 ] || { echo "드라이버가 시리얼을 보고하지 않음"; exit 4; }
sleep "$WARMUP"
printf 'started_epoch %s\nwarmup_sec %s\nmax_sec %s\n' "$(date +%s.%N)" "$WARMUP" "$MAXSEC" > "$STATE"
OK=1
echo "시작 완료: 워밍업 ${WARMUP} s, 최대 가동 ${MAXSEC} s"
