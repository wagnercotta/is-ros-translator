#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
COMPOSE=deploy/e2e/compose.yaml
if ! docker compose version >/dev/null 2>&1; then
  echo 'Docker Engine and Compose v2 are required.' >&2
  exit 127
fi
DC=(docker compose -f "$COMPOSE")

diagnose() {
  echo '=== Docker containers ==='
  "${DC[@]}" ps -a || true
  echo '=== ROS 1/ROS 2 startup logs (last 120 lines) ==='
  "${DC[@]}" logs --tail=120 ros1a ros1b ros2 ros2b || true
  echo '=== Docker IPC donor / consumer modes ==='
  local donor_id rviz_id
  donor_id="$("${DC[@]}" ps -q ros2)"
  rviz_id="$("${DC[@]}" ps -q rviz)"
  if [[ -n "$donor_id" ]]; then
    docker inspect --format 'ros2: IPC={{.HostConfig.IpcMode}}; shm={{.HostConfig.ShmSize}}' "$donor_id" || true
  fi
  if [[ -n "$rviz_id" ]]; then
    docker inspect --format 'rviz: IPC={{.HostConfig.IpcMode}}; network={{.HostConfig.NetworkMode}}' "$rviz_id" || true
  fi
  echo '=== ROS2 Docker health log ==='
  local id
  id="$("${DC[@]}" ps -q ros2)"
  if [[ -n "$id" ]]; then
    docker inspect --format '{{range .State.Health.Log}}{{println .End .ExitCode .Output}}{{end}}' "$id" || true
  fi
  echo '=== ROS2 image embedded entrypoint (first 20 lines) ==='
  docker run --rm --entrypoint /bin/bash e2e-ros2 -c 'head -n 20 /opt/ros-lab/start.sh' || true
  echo '=== RabbitMQ resource alarms ==='
  "${DC[@]}" exec -T rabbitmq rabbitmq-diagnostics alarms 2>/dev/null || true
  echo '=== Gateway is-wire transport health ==='
  "${DC[@]}" exec -T gateway python -c "import urllib.request; print(urllib.request.urlopen('http://127.0.0.1:8080/health/ready', timeout=3).read().decode())" 2>/dev/null || true
  echo '=== Other services (last 40 lines) ==='
  "${DC[@]}" logs --tail=40 gateway rviz rabbitmq || true
}

up_stack() {
  if ! "${DC[@]}" up -d --build --force-recreate rabbitmq ros1a ros1b ros2 ros2b gateway rviz; then
    echo '[ERROR] Compose startup failed; collecting diagnostics.' >&2
    diagnose
    return 1
  fi
}

case "${1:-test}" in
  up)
    up_stack
    "${DC[@]}" ps -a
    ;;
  test)
    up_stack
    if ! "${DC[@]}" --profile test run --build --rm e2e; then
      echo '[ERROR] E2E failed; collecting diagnostics.' >&2
      diagnose
      exit 1
    fi
    echo 'Results: deploy/e2e/reports/e2e-report.json'
    ;;
  action)
    up_stack
    if ! E2E_ACTION_TEST=true "${DC[@]}" --profile test run --build --rm -e E2E_ACTION_TEST=true e2e; then
      echo '[ERROR] E2E action test failed; collecting diagnostics.' >&2
      diagnose
      exit 1
    fi
    ;;
  loop)
    up_stack
    "${DC[@]}" --profile loop up -d --build --force-recreate e2e-loop
    echo 'Continuous E2E started in the background. View: scripts/docker-ros-e2e.sh loop-logs'
    echo 'Reports: deploy/e2e/reports/e2e-loop-report.json and e2e-loop-history.jsonl'
    echo 'Stop only the random tester: scripts/docker-ros-e2e.sh loop-stop'
    ;;
  loop-logs)
    "${DC[@]}" --profile loop logs -f --tail=100 e2e-loop
    ;;
  loop-stop)
    "${DC[@]}" --profile loop stop -t 120 e2e-loop
    ;;
  logs)
    "${DC[@]}" logs -f --tail=120 "${@:2}"
    ;;
  ps)
    "${DC[@]}" ps -a
    ;;
  doctor|diagnose)
    diagnose
    ;;
  down)
    "${DC[@]}" --profile test --profile loop down --remove-orphans
    ;;
  clean)
    "${DC[@]}" --profile test --profile loop down --remove-orphans --volumes
    ;;
  *)
    echo 'Usage: scripts/docker-ros-e2e.sh [test|action|loop|loop-logs|loop-stop|up|ps|doctor|logs [service]|down|clean]' >&2
    exit 2
    ;;
esac
