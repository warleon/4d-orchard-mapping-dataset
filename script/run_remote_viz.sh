#!/usr/bin/env bash
# Runs a roslaunch pipeline on anthony-gpu and views it locally in rviz,
# tunneled over sshuttle - the whole manual workflow from this project's
# remote-GPU grape-dataset work, wrapped into one script. See the launch
# files under launch/ for what's available (semantic_pointcloud_grape.launch,
# mapping_ouster_grape.launch, etc.) and their own doc="..." args.
#
# Usage:
#   script/run_remote_viz.sh [options] <launch_file> [roslaunch_arg...]
#
# Options:
#   -r, --rviz-config FILE   rviz config to open locally (default:
#                            config/rviz/semantic_pointcloud.rviz)
#   -n, --no-rviz            bring up the remote pipeline + tunnel, skip
#                            launching rviz (e.g. if you'll drive rviz
#                            yourself, or just want the data flowing)
#   -k, --no-tunnel          skip sshuttle - use when it's already running
#                            in another terminal
#   -h, --help               show this help and exit
#
# Example:
#   script/run_remote_viz.sh semantic_pointcloud_grape.launch rate:=0.2 \
#     data_bag:=/home/ubuntu/repos/rosbag_compressed_2022-09-15-14-23-20.bag
#
# Requires: the "anthony-gpu" ssh host alias (and its AWS SSM ProxyCommand
# setup), and nix (for `nix shell nixpkgs#sshuttle`). The remote pipeline and
# the sshuttle tunnel (daemonized, see /tmp/run_remote_viz_sshuttle.pid) both
# keep running in the background after this script exits (Ctrl+C only stops
# rviz) - rerun with --no-rviz --no-tunnel later to just reattach, or kill
# them yourself over ssh/locally.

set -euo pipefail

SSH_HOST="anthony-gpu"
REMOTE_REPO="~/repos/4d-orchard-mapping-dataset"
REMOTE_IP="172.31.1.233"   # anthony-gpu's private IP - advertised via ROS_IP
                           # so remote nodes are reachable through the tunnel
INSTANCE_ID="i-05151ab6735fec5a4"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RVIZ_CONFIG="config/rviz/semantic_pointcloud.rviz"
SKIP_RVIZ=0
SKIP_TUNNEL=0

usage() { sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r|--rviz-config) RVIZ_CONFIG="$2"; shift 2 ;;
    -n|--no-rviz) SKIP_RVIZ=1; shift ;;
    -k|--no-tunnel) SKIP_TUNNEL=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; usage; exit 1 ;;
    *) break ;;
  esac
done

if [[ $# -lt 1 ]]; then
  echo "error: missing <launch_file>" >&2
  usage
  exit 1
fi

LAUNCH_FILE="$1"
shift
LAUNCH_ARGS=("$@")

ssh_remote() {
  env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH ssh "$SSH_HOST" "$@"
}

echo "==> checking anthony-gpu instance state"
STATE="$(env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
  AWS_PROFILE=anthony-gpu AWS_REGION=us-east-2 \
  aws ec2 describe-instances --instance-ids "$INSTANCE_ID" \
  --query 'Reservations[0].Instances[0].State.Name' --output text)"

if [[ "$STATE" != "running" ]]; then
  echo "==> instance is '$STATE', starting it"
  env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
    AWS_PROFILE=anthony-gpu AWS_REGION=us-east-2 \
    aws ec2 start-instances --instance-ids "$INSTANCE_ID" >/dev/null
  env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
    AWS_PROFILE=anthony-gpu AWS_REGION=us-east-2 \
    aws ec2 wait instance-running --instance-ids "$INSTANCE_ID"
  echo "==> waiting for ssh"
  until ssh_remote -o ConnectTimeout=5 -o BatchMode=yes true 2>/dev/null; do
    sleep 5
  done
fi
echo "==> instance up, ssh reachable"

echo "==> stopping any leftover ROS processes on remote"
ssh_remote 'pkill -9 -f "roslaunch|rosmaster|rosout|laserMapping|run_mapping_online|data_bag_player|zed_front_decompress|semantic_pointcloud_node|rosbag|extract_image" 2>/dev/null; true'
sleep 1

echo "==> launching $LAUNCH_FILE on remote (args: ${LAUNCH_ARGS[*]:-none})"
REMOTE_LOG="/tmp/run_remote_viz_$(date +%s).log"
ssh_remote "
  cd $REMOTE_REPO
  setsid nohup env ROS_IP=$REMOTE_IP nix --extra-experimental-features 'nix-command flakes' develop ./flake --command bash -c '
    source ws/devel/setup.bash
    export ROS_IP=$REMOTE_IP
    roslaunch $LAUNCH_FILE rviz:=false ${LAUNCH_ARGS[*]}
  ' > $REMOTE_LOG 2>&1 < /dev/null &
  disown
"
echo "    remote log: $REMOTE_LOG (tail it over ssh if something looks wrong)"

echo "==> waiting for remote ROS master to listen on :11311"
until ssh_remote 'ss -tlnp 2>/dev/null | grep -q 11311' 2>/dev/null; do
  sleep 3
done
echo "==> master is up"

SSHUTTLE_PIDFILE=/tmp/run_remote_viz_sshuttle.pid
if [[ "$SKIP_TUNNEL" -eq 0 ]]; then
  if [[ -f "$SSHUTTLE_PIDFILE" ]] && sudo kill -0 "$(cat "$SSHUTTLE_PIDFILE")" 2>/dev/null; then
    echo "==> sshuttle tunnel already running (pid $(cat "$SSHUTTLE_PIDFILE")), reusing it"
  else
    echo "==> starting sshuttle tunnel to $REMOTE_IP (needs sudo)"
    nix --extra-experimental-features "nix-command flakes" shell nixpkgs#sshuttle --command \
      sudo env AWS_CONFIG_FILE="$HOME/.aws/config" AWS_SHARED_CREDENTIALS_FILE="$HOME/.aws/credentials" \
      sshuttle -r "$SSH_HOST" "$REMOTE_IP/32" \
      --ssh-cmd "ssh -F $HOME/.ssh/config -i $HOME/.ssh/anthony-gpu-key" \
      -D --pidfile="$SSHUTTLE_PIDFILE"
    echo "    tunnel backgrounded (pidfile: $SSHUTTLE_PIDFILE)"
    sleep 2
  fi
fi

export ROS_MASTER_URI="http://$REMOTE_IP:11311"

echo "==> waiting for topics to be reachable through the tunnel"
cd "$LOCAL_REPO"
source ws/devel/setup.bash
for _ in $(seq 1 20); do
  if timeout 5 rostopic list >/dev/null 2>&1; then
    echo "==> topics reachable"
    break
  fi
  sleep 3
done

if [[ "$SKIP_RVIZ" -eq 0 ]]; then
  echo "==> launching local rviz ($RVIZ_CONFIG)"
  nixGLIntel rviz -d "$RVIZ_CONFIG"
else
  echo "==> --no-rviz set; skipping. ROS_MASTER_URI=$ROS_MASTER_URI"
fi
