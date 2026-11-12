#!/usr/bin/env bash
# G00 Jetson 읽기 전용 인벤토리. 로봇 이동/쓰기/조회, I2C 탐침, SDK import 없음.
# 사용: JETSON_HOST=user@host bash scripts/g00_jetson_readonly.sh precheck|inventory|urdf <원격경로>
# 모든 원격 명령과 출력은 evidence/G00/raw/ (gitignore)에 그대로 남는다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="${JETSON_HOST:?JETSON_HOST 없음}"
OUT="${ROOT}/evidence/G00/raw"; mkdir -p "${OUT}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${H}")
STAGE="${1:-}"; LOG="${OUT}/jetson-${STAGE}.log"
FORBIDDEN='import |Arm_serial|Arm_Device|Arm_Lib\(|smbus|i2c(set|get|detect|dump)|systemctl (start|stop|restart)|kill|(^|[ ;|])rm |>>|> /|ros2 (launch|run)|(^|[ ;|])mv |(^|[ ;|])cp |chmod'

run_ro() {  # 읽기 전용 명령 1개 실행. 금지 패턴이면 거부한다.
  local cmd="$1"
  if printf '%s' "${cmd}" | grep -Eq "${FORBIDDEN}"; then echo "REFUSED: ${cmd}" | tee -a "${LOG}"; return 1; fi
  { echo "\$ ${cmd}"; "${SSH[@]}" "${cmd}" 2>&1 || echo "[exit=$?]"; echo; } | tee -a "${LOG}"
}

: > "${LOG}"
echo "# stage=${STAGE} $(date -u +%FT%TZ)" >> "${LOG}"
case "${STAGE}" in
  precheck)
    run_ro 'ps -eo pid,etime,args --sort=pid | grep -Ei "ros|dofbot|yahboom|arm_|orbbec|color_grasp|color_detect" | grep -v grep || true'
    run_ro 'systemctl list-units --type=service --state=running --no-pager | head -60'
    run_ro 'docker ps --format "{{.Image}} {{.Status}} {{.Names}}" 2>&1 | head -20 || true'
    run_ro 'echo ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}; source /opt/ros/humble/setup.bash; ros2 node list --no-daemon --spin-time 3 2>&1 | head -40 || true'
    ;;
  inventory)
    run_ro 'uname -srm; cat /etc/nv_tegra_release 2>/dev/null | head -1 || true; grep -E "^(NAME|VERSION)=" /etc/os-release'
    run_ro 'python3 --version; free -m | head -2; df -h / | tail -1'
    run_ro 'ls -l /dev/i2c-* /dev/video* /dev/ttyUSB* /dev/ttyACM* 2>&1 || true'
    run_ro 'lsusb'
    run_ro 'lsusb -d 2bc5: -v 2>/dev/null | grep -E "idVendor|idProduct|iProduct|iSerial|bcdDevice" || true'
    run_ro 'ls ~/dofbot_pro_ws/src 2>&1 | head -40'
    run_ro 'source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash; ros2 pkg list 2>/dev/null | grep -iE "orbbec|dofbot|arm|yahboom" || true'
    run_ro 'pip3 list 2>/dev/null | grep -iE "arm|orbbec|numpy|opencv" || true'
    run_ro 'pip3 show -f Arm_Lib 2>&1 | head -25 || true'
    run_ro 'find ~/dofbot_pro_ws -iname "*.urdf" -o -iname "*.xacro" 2>/dev/null | head -30'
    run_ro 'find ~/dofbot_pro_ws -ipath "*orbbec_camera*" -iname "*dabai*launch*" 2>/dev/null | head -20'
    ;;
  urdf)
    P="${2:?원격 경로 필요}"
    [[ "${P}" =~ ^[A-Za-z0-9_./~-]+$ ]] || { echo "경로 문자 거부"; exit 2; }
    run_ro "grep -nE '<joint|<link|^ *name=|type=|link=|lower=|upper=|effort=|velocity=|<parent|<child' ${P} | grep -vE 'mesh|filename' | head -400"
    ;;
  sdk)
    # 고정 명령만(인자 없음). SDK 소스 텍스트를 읽을 뿐 import·실행하지 않는다. 텍스트에 API 이름이 들어 있어 FORBIDDEN 검사를 쓰지 않는다.
    EGG=/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg
    for cmd in "ls -la ${EGG}" "file ${EGG}" \
      "if [ -d ${EGG} ]; then find ${EGG} -name '*.py'; else unzip -l ${EGG}; fi" \
      "if [ -d ${EGG} ]; then grep -rnE 'def |sleep|range|180|270' ${EGG} --include=*.py | head -150; else for f in \$(unzip -Z1 ${EGG} | grep '\\.py\$'); do echo \"== \$f\"; unzip -p ${EGG} \$f | grep -nE 'def |sleep|range|180|270' | head -150; done; fi"; do
      { echo "\$ ${cmd}"; "${SSH[@]}" "${cmd}" 2>&1 || echo "[exit=$?]"; echo; } | tee -a "${LOG}"
    done
    ;;
  dabai)
    # DaBai 드라이버 기동 → camera_info·토픽 조회 → 반드시 종료. 로봇 명령 없음(precheck에서 팔 제어 프로세스 없음 확인 후에만 실행).
    SRC='source /opt/ros/humble/setup.bash; source ~/dofbot_pro_ws/install/setup.bash'
    fx() { { echo "\$ $1"; "${SSH[@]}" "$1" 2>&1 || echo "[exit=$?]"; echo; } | tee -a "${LOG}"; }
    stop_driver() { fx 'kill -INT $(cat /tmp/g00_dabai.pid 2>/dev/null) 2>/dev/null; sleep 2; pkill -INT -f "[_]_ns:=/dabai" 2>/dev/null; sleep 2; pgrep -fa "[_]_ns:=/dabai" || echo "드라이버 없음"'; }
    trap stop_driver EXIT
    fx "grep -nE 'depth_registration|enable_|width|height|fps|align' ~/dofbot_pro_ws/src/OrbbecSDK_ROS2-main/orbbec_camera/launch/dabai_dcw2.launch.py | head -60"
    fx 'pgrep -f "[_]_ns:=/dabai" >/dev/null && echo "이미 떠 있음" || echo "기동 전 드라이버 없음"'
    fx "${SRC}; nohup ros2 launch orbbec_camera dabai_dcw2.launch.py camera_name:=dabai enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false > /tmp/g00_dabai_drv.log 2>&1 & echo \$! > /tmp/g00_dabai.pid; sleep 10; pgrep -fa '[_]_ns:=/dabai' | head -3"
    fx "${SRC}; ros2 topic list --no-daemon --spin-time 3 2>&1 | grep -i dabai || true"
    fx "${SRC}; (timeout 30 ros2 topic hz /dabai/color/image_raw --no-daemon > /tmp/g00_color_hz.txt 2>&1 &); sleep 4; timeout 20 ros2 topic echo /dabai/color/camera_info --once --no-daemon 2>&1 | head -40; sleep 8; tail -3 /tmp/g00_color_hz.txt || true"
    fx "${SRC}; timeout 20 ros2 topic echo /dabai/depth/camera_info --once --no-daemon 2>&1 | head -40 || true"
    fx 'tail -40 /tmp/g00_dabai_drv.log 2>&1 || true'
    ;;
  *) echo "사용: JETSON_HOST=... $0 precheck|inventory|urdf <경로>|sdk|dabai" >&2; exit 2 ;;
esac
echo "로그: evidence/G00/raw/jetson-${STAGE}.log"
