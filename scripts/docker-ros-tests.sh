#!/usr/bin/env bash
# Starts the Docker ROS 1 + ROS 2 lab and optionally runs all real integration checks.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
ENV_FILE="deploy/ros-test/.env"
COMPOSE="deploy/ros-test/compose.yaml"
if ! command -v docker >/dev/null 2>&1; then
  echo 'Docker CLI not found. Install Docker Engine + Compose v2 plugin.' >&2
  exit 127
fi
if ! docker compose version >/dev/null 2>&1; then
  echo 'Docker Compose v2 is required.' >&2
  exit 127
fi
if [[ ! -f "$ENV_FILE" ]]; then
  cp deploy/ros-test/.env.example "$ENV_FILE"
  chmod 600 "$ENV_FILE"
  echo "Created $ENV_FILE. Replace both example secrets, then rerun." >&2
  exit 2
fi
if grep -Eiq '^(RABBITMQ_PASSWORD|GATEWAY_API_KEY)=replace-' "$ENV_FILE"; then
  echo "Edit $ENV_FILE and replace both example secrets before starting." >&2
  exit 2
fi
DC=(docker compose --env-file "$ENV_FILE" -f "$COMPOSE")
case "${1:-test}" in
  up)
    "${DC[@]}" up -d --build rabbitmq ros1 ros2 gateway
    "${DC[@]}" ps
    ;;
  test)
    "${DC[@]}" up -d --build rabbitmq ros1 ros2 gateway
    "${DC[@]}" --profile test run --build --rm tester
    ;;
  logs)
    "${DC[@]}" logs -f --tail=100 "${@:2}"
    ;;
  ps)
    "${DC[@]}" ps
    ;;
  down)
    "${DC[@]}" --profile test down
    ;;
  clean)
    "${DC[@]}" --profile test down --remove-orphans --volumes
    ;;
  *)
    echo 'Usage: scripts/docker-ros-tests.sh [test|up|logs|ps|down|clean]' >&2
    exit 2
    ;;
esac
