#!/bin/bash
# Load every world and report which models actually spawn and which assets are missing.
# Usage: check_worlds.sh [--gui] [world ...]     HOLD_S=30 keeps each world up that long.
#
# Worlds are launched through roslaunch, not bare gzserver: gazebo_ros_paths_plugin is what
# turns the package.xml <gazebo_ros> export tags into GAZEBO_MODEL_PATH and
# GAZEBO_RESOURCE_PATH, so a bare gzserver never sees the MRS models and every world with an
# external mesh looks broken.
# Readiness is taken from /gazebo/get_world_properties, never from a log line: "Publicized
# address" is printed before the world is parsed and reports success for a world that has not
# loaded a single model.

gui=false
[ "${1:-}" = "--gui" ] && { gui=true; shift; }
hold=${HOLD_S:-0}

source /opt/ros/noetic/setup.bash
source "$(cd "$(dirname "$0")/../../.." && pwd)/devel/setup.bash" 2>/dev/null
W="$(cd "$(dirname "$0")/../worlds" && pwd)"


worlds=()
for a in "$@"; do worlds+=("$(cd "$(dirname "$a")" && pwd)/$(basename "$a")"); done
[ ${#worlds[@]} -eq 0 ] && worlds=("$W"/*.world)

# Resolve every asset URI the way gazebo does. A model whose mesh is missing still spawns as a
# named entity with no geometry, so the service check alone cannot see it: that is how the
# absent maze mesh passed as "5/5 models".
# The search paths come from rospack, not from the environment: gazebo_ros_paths_plugin adds
# the package.xml <gazebo_ros> export paths inside the gzserver process, so they never appear
# in the environment or in /proc/<pid>/environ.
GZ_MODEL_PATHS=$( { rospack plugins --attrib=gazebo_model_path gazebo_ros 2>/dev/null | awk '{print $2}'
                    echo "$GAZEBO_MODEL_PATH" | tr ':' '\n'; } | grep . | sort -u | paste -sd: )
GZ_MEDIA_PATHS=$( { rospack plugins --attrib=gazebo_media_path gazebo_ros 2>/dev/null | awk '{print $2}'
                    echo "$GAZEBO_RESOURCE_PATH" | tr ':' '\n'
                    echo /usr/share/gazebo-11; echo /usr/share/gazebo; } | grep . | sort -u | paste -sd: )

unresolved_uris() {
  MP="$GZ_MODEL_PATHS" RP="$GZ_MEDIA_PATHS" python3 - "$1" <<'PYURI'
import os, sys, xml.etree.ElementTree as ET
mp = [p for p in os.environ.get('MP','').split(':') if p]
rp = [p for p in os.environ.get('RP','').split(':') if p]

def found(roots, rest):
    for r in roots:
        if os.path.exists(os.path.join(r, rest)):
            return 'exact'
    # second pass ignoring case, so a pre-existing case wart is reported apart from a real miss
    for r in roots:
        cur, ok = r, True
        for part in rest.split('/'):
            try:
                hit = next((e for e in os.listdir(cur) if e.lower() == part.lower()), None)
            except OSError:
                ok = False
                break
            if hit is None:
                ok = False
                break
            cur = os.path.join(cur, hit)
        if ok:
            return 'case'
    return None

bad, case = [], []
for u in {e.text.strip() for e in ET.parse(sys.argv[1]).getroot().iter('uri') if e.text}:
    if u.startswith('model://'):
        rest, roots = u[len('model://'):], mp
    elif u.startswith('file://'):
        # gazebo falls back to the model paths for file:// too, which is how grass_plane
        # resolves: it sits under <pkg>/models/grass_plane, not at the media root
        rest, roots = u[len('file://'):], rp + mp
    else:
        continue
    r = found(roots, rest)
    if r is None:
        bad.append(u)
    elif r == 'case':
        case.append(u)
out = ''
if bad:
    out += 'UNRESOLVED ' + ' '.join(sorted(bad))
if case:
    out += ('  ' if out else '') + 'case-only ' + ' '.join(sorted(case))
print(out)
PYURI
}

world_models() {
  rosservice call /gazebo/get_world_properties 2>/dev/null |
    awk '/^model_names:/{f=1;next} f && /^  - /{sub(/^  - /,"");print;next} f{exit}'
}

expected_models() {
  python3 - "$1" <<'PY'
import sys, xml.etree.ElementTree as ET
w = ET.parse(sys.argv[1]).getroot().find('world')
out = []
for m in w.findall('model'):
    out.append(m.get('name'))
for i in w.findall('include'):
    u = i.find('uri')
    n = i.find('name')
    out.append(n.text if n is not None else (u.text.rstrip('/').split('/')[-1] if u is not None else '?'))
print('\n'.join(x for x in out if x))
PY
}

printf "%-34s %8s %7s %16s  %s\n" "world" "loaded" "secs" "models ok/exp" "problem"
port=11500

for w in "${worlds[@]}"; do
  n=$(basename "$w"); base=$(basename "$w" .world)
  log=/tmp/checkworld_$base.log
  port=$((port + 2))
  export ROS_MASTER_URI="http://127.0.0.1:${port}"
  export GAZEBO_MASTER_URI="http://127.0.0.1:$((port + 1))"

  exp=$(expected_models "$w"); nexp=$(echo "$exp" | grep -c . )

  t0=$(date +%s.%N)
  roslaunch -p "$port" gazebo_ros empty_world.launch \
      world_name:="$w" gui:="$gui" paused:=false verbose:=true > "$log" 2>&1 &
  pid=$!

  loaded=no; got=""
  for _ in $(seq 1 120); do
    sleep 0.5
    got=$(world_models | grep -c .)
    [ -n "$got" ] && [ "$got" -gt 0 ] 2>/dev/null && { loaded=yes; break; }
    kill -0 "$pid" 2>/dev/null || break
  done
  t1=$(date +%s.%N)
  [ -z "$got" ] && got=0

  badu=$(unresolved_uris "$w")

  missing=""
  [ "$loaded" = yes ] && {
    have=$(world_models)
    while read -r m; do
      [ -n "$m" ] && ! grep -qx "$m" <<<"$have" && missing="$missing $m"
    done <<<"$exp"
  }

  [ "$hold" != 0 ] && sleep "$hold"

  problem=$(grep -ohE "Unable to find (file|uri)\[[^]]+\]|does not exist \[[^]]+\]" "$log" 2>/dev/null | sort -u | tr '\n' ' ')
  [ -n "$missing" ] && problem="did not spawn:$missing  $problem"
  [ -n "$badu" ] && problem="$badu  $problem"
  [ -z "$problem" ] && problem="none"

  printf "%-34s %8s %7.1f %16s  %s\n" "$n" "$loaded" "$(echo "$t1-$t0" | bc)" "$got/$nexp" "$problem"

  kill -INT "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
  pkill -9 -f "gzserver .*$n" 2>/dev/null
  pkill -9 -f "gzclient" 2>/dev/null
  sleep 1
done
exit 0
