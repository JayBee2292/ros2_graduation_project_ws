#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
exec "$PWD/run_slam_navigation.sh" launch_frontier_explorer:=true auto_explore:=true "$@"
