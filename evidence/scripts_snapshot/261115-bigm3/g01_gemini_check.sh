#!/usr/bin/env bash
# p620에서 실행(ssh로 stdin 전달). 이 프로젝트 전용 컨테이너만 조작. 로봇 명령 없음.
set -euo pipefail
NAME=dofbot-g01-camera
IMAGE=cad-less-robotics:humble-p003
VOL=bytesize-robotics-camera-install
WORK="$HOME/dofbot-g01-work"
cleanup() { docker exec -u 0 "$NAME" chown -R "$(id -u):$(id -g)" /work >/dev/null 2>&1 || true; docker rm -f "$NAME" >/dev/null 2>&1 || true; echo "컨테이너 정리: $(docker ps -a --format '{{.Names}}' | grep -c "^${NAME}$" || true)개 남음"; }
trap cleanup EXIT
line="$(lsusb -d 2bc5:0804 | head -n1)"; [ -n "$line" ] || { echo "Gemini 없음"; exit 4; }
node="/dev/bus/usb/$(echo "$line" | awk '{print $2}')/$(echo "$line" | awk '{print $4}' | tr -d ':')"
if fuser "$node" >/dev/null 2>&1; then echo "USB 사용 중: $node"; exit 3; fi
for n in $(docker ps --format '{{.Names}}'); do
  d="$(docker inspect --format '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$n")"
  if [[ "$d" == *"$node"* ]]; then echo "다른 컨테이너가 점유: $n"; exit 3; fi
done
docker volume inspect "$VOL" >/dev/null
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --device "$node" -v "$WORK:/work" -v "$VOL:/opt/cad-less-install:ro" -e HOME=/tmp "$IMAGE" sleep infinity >/dev/null
run_cfg() {
  local cfg="$1"; shift
  docker exec -e CFG="$cfg" -e EXTRA="$*" "$NAME" bash -lc '
    set +u; source /opt/ros/humble/setup.bash; source /opt/cad-less-install/setup.bash; set -u
    mkdir -p /work/$CFG
    setsid stdbuf -oL -eL ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=false enable_colored_point_cloud:=false enable_publish_extrinsic:=true $EXTRA > /work/$CFG/driver.log 2>&1 &
    echo $! > /tmp/drv.pid
    ok=0; for _ in $(seq 1 90); do grep -q "Serial number:" /work/$CFG/driver.log && { ok=1; break; }; sleep 1; done
    [ $ok -eq 1 ] || { echo "드라이버가 시리얼을 보고하지 않음"; exit 4; }
    sleep 12
    python3 /work/g01_gemini_probe.py /work/$CFG 15 camera
    kill -INT -- -$(cat /tmp/drv.pid) >/dev/null 2>&1 || true; sleep 4
    pgrep -f "[o]rbbec_camera_node" >/dev/null && { pkill -f "[o]rbbec_camera_node" >/dev/null 2>&1 || true; sleep 2; } || true
    pgrep -f "[o]rbbec_camera_node" >/dev/null && echo "드라이버 남음($CFG)" || echo "드라이버 없음($CFG)"
  '
}
run_cfg "${CFG_NAME:?CFG_NAME 필요}"
