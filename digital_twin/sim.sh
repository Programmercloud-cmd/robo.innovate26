#!/usr/bin/env bash
# Scene and fingertips are independent explicit selections. No robot commands.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Easy presets: digital_twin/sim.sh slim | wide [--rebuild]'
  echo 'Independent choices: digital_twin/sim.sh factory|board [--fingers original|wide] [--rebuild]'
  echo 'Selector: http://127.0.0.1:18767/ · existing factory remains on :18766'
  exit 0
fi
variant_vm=robot-innovate26-place-rl3
variant_root=/home/ubuntu/workspace/fr3-digital-twin-20261026
case "$1" in
  slim|wide)
    variant_preset=$1; shift
    for arg in "$@"; do
      [[ $arg == --rebuild ]] || { echo 'Presets accept only --rebuild; use explicit environment/finger arguments for other combinations.' >&2; exit 2; }
    done
    if [[ $variant_preset == slim ]]; then set -- factory --fingers original "$@"; else set -- board --fingers wide "$@"; fi ;;
esac
printf 'Simulation: %s\n' "$*"
case "$1" in factory|board) ;; *) echo 'Unknown environment' >&2; exit 2 ;; esac
# Pass only the supported literal CLI choices to SSH, never arbitrary shell text.
for arg in "$@"; do
  case "$arg" in factory|board|--fingers|original|wide|--rebuild) ;; *) echo 'Unsupported argument' >&2; exit 2 ;; esac
done
ssh -T -o BatchMode=yes "$variant_vm" "mkdir -p '$variant_root/isaac/environments/assets' '$variant_root/isaac/lego_contact/task' '$variant_root/registration'"
scp -q digital_twin/isaac/{mirror.py,viewer.html} "$variant_vm:$variant_root/isaac/"
scp -q digital_twin/isaac/environments/{__init__.py,profiles.json,geometry.py,cad_fingers.py,ceiling.py,recording.py,snapshot.py,serve.py,manage.py,audit.py} "$variant_vm:$variant_root/isaac/environments/"
scp -q digital_twin/isaac/environments/assets/{original_fingertip,wide_lego_gripper}.stl "$variant_vm:$variant_root/isaac/environments/assets/"
scp -q digital_twin/isaac/lego_contact/task/{scene.py,stack_layout.py,config.json,plate_registration_candidate.json} "$variant_vm:$variant_root/isaac/lego_contact/task/"
scp -q digital_twin/registration/scene_registration.json "$variant_vm:$variant_root/registration/"
ssh -T -o BatchMode=yes "$variant_vm" "/home/ubuntu/workspace/robo26-runtime-isaac51/venv/bin/python '$variant_root/isaac/environments/manage.py' $*"
if ! curl -fsS --max-time 2 http://127.0.0.1:18767/selector.html >/dev/null 2>&1; then
  ssh -fNT -o BatchMode=yes -o ExitOnForwardFailure=yes -L 127.0.0.1:18767:127.0.0.1:8767 "$variant_vm"
fi
echo 'Simulation selector: http://127.0.0.1:18767/'
