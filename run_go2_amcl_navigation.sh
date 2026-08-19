#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

exec ./run_go2_amcl_manual_drive.sh "$@" launch_navigation:=true
