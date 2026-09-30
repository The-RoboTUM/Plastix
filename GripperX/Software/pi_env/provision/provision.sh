#!/usr/bin/env bash
# =============================================================================
#  GripperX — provision a FRESH Ubuntu 24.04 raspi card into the robot Pi
# =============================================================================
#  What:  turns a freshly flashed Ubuntu 24.04.4 server arm64 (raspi) card, on
#         which cloud-init has already run (hostname GripperX-1, user `ubuntu`
#         with passwordless sudo and the laptop key, netplan in place), into a
#         Pi that runs the GripperX stack exactly like the reference card did.
#  Who:   run ON THE PI as `ubuntu`, over SSH from the laptop. See README.md in
#         this directory for the full runbook (laptop steps included).
#  Safety (SR-1, motor-enable process rule): this script NEVER starts a
#         gripperx-* unit and by default does NOT enable the stack units either,
#         because the reboot that provisioning requires would then start the
#         stack — and a bringup start is a motion trigger that needs its own
#         per-test user approval. Enabling is a separate, explicit invocation:
#         `provision.sh --enable-units ...` (see README.md).
#         It also refuses to run while any gripperx-* unit is active.
#
#  Idempotent and re-runnable: every step checks the state first and only
#  changes what differs. After a failure, fix the cause and re-run the same
#  command; `--from N` skips the steps already done.
#
#  Reference machine: the old GripperX-1 card, inventoried read-only 2026-09-30.
#  The pinned values below (ROS snapshot date, agent image digest, SDK hash)
#  were read from it or verified against it that day — see README.md.
# =============================================================================

set -Eeuo pipefail

# ---- fixed, Pi-side facts (the same on every GripperX Pi) --------------------
readonly PI_USER=ubuntu
readonly PI_HOME=/home/ubuntu
readonly EXPECTED_HOSTNAME=GripperX-1
readonly WS="$PI_HOME/ws"
readonly BARE="$PI_HOME/repos/gripperx_ws.git"
readonly COLCON_ROOT="$WS/Software/ros2"
readonly PI_ENV="$WS/Software/pi_env"
readonly PROV="$PI_ENV/provision"
readonly LOG_DIR="$PI_HOME/provision_logs"
readonly TIMEZONE=Europe/Berlin

# ROS 2 Jazzy apt snapshot that contains EXACTLY the 405 ros-jazzy-* versions of
# the reference card (verified 2026-09-30: 0 missing). packages.ros.org main has
# moved on (e.g. controller_manager 4.45.2 -> 4.48.0), so it is NOT used here.
readonly ROS_SNAPSHOT_URL="http://snapshots.ros.org/jazzy/2026-06-18/ubuntu"
readonly ROS_SNAPSHOT_KEYRING=/usr/share/keyrings/ros-snapshots-archive-keyring.gpg
readonly ROS_SNAPSHOT_FPR=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
readonly ROS_SNAPSHOT_LIST=/etc/apt/sources.list.d/ros2-snapshot.list

# micro-ROS agent image, pinned by the digest the reference card runs.
readonly AGENT_REPO=microros/micro-ros-agent
readonly AGENT_TAG=jazzy
readonly AGENT_DIGEST=sha256:1d13bfebcd8ab6f8bbdd95ac64acf42a54eb97f2421ec559d871e7aa2c036f48

# PlatformIO, as on the reference card (6.1.19, platform espressif32 7.0.1), but in
# its own venv instead of the reference card's pip --user install, so its click
# pin (<8.4) cannot collide with anything in the system/ROS python.
readonly PIO_VERSION=6.1.19
readonly PIO_CLICK_SPEC='click>=8.0.4,<8.4'
readonly PIO_PLATFORM='platformio/espressif32@7.0.1'
readonly PIO_VENV="$PI_HOME/.platformio/penv"
readonly FW_PROJECT="$WS/Software/microros/firmware"
readonly FW_ENV=esp32-s3

# Units that make up the ROS stack (enabled on the reference card). Enabling
# them is the separate --enable-units action, never part of a normal run.
readonly STACK_UNITS=(gripperx-agent.service gripperx-bringup.service gripperx-mapping.service
                      gripperx-navigation.service gripperx-external.service gripperx-button.service)
# Enabled by a normal run: no motion path (wpa_cli reconnect only).
readonly SAFE_UNITS=(gripperx-wifi.timer)

# Top-level ROS packages = the reference card's manually installed ros-jazzy-*
# set, minus the MoveIt family (ros-jazzy-moveit: nothing in Software/ros2
# depends on it). Their dependency closure is checked version-by-version
# against reference/ros-jazzy-expected.txt in step 7.
readonly ROS_PKGS=(
  ros-jazzy-ros-base
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rmw-fastrtps-shared-cpp
  ros-jazzy-fastrtps ros-jazzy-fastrtps-cmake-module
  ros-jazzy-rmw-cyclonedds-cpp
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  ros-jazzy-controller-manager ros-jazzy-hardware-interface
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
  ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard
  ros-jazzy-tf2-tools ros-jazzy-gz-ros2-control
)

# Non-ROS packages. Deliberately NOT carried over from the reference card:
# clang-tidy (pulls llvm-18-dev, lint only), bison/flex/libncurses-dev (only
# for the unused ~/microros_ws source build), libserial-dev (no package in
# Software/ros2 links it), can-utils (no CAN bus on the robot).
readonly SYSTEM_PKGS=(
  git curl wget rsync gnupg ca-certificates
  build-essential cmake
  python3-pip python3-setuptools python3-wheel python3-venv
  python3-numpy python3-yaml python3-serial python3-websockets python3-libgpiod
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete
  docker.io avahi-daemon i2c-tools libraspberrypi-bin
  usbutils net-tools lsof strace htop tmux vim
)

# ---- arguments ---------------------------------------------------------------
NTP_SERVERS=""
BRANCH=""
AGENT_TAR=""
SKIP_UPGRADE=0
HOLD_AUTO_UPGRADES=0
SKIP_PLATFORMIO=0
FROM_STEP=0
ONLY_STEP=""
ACTION=provision

usage() {
  cat <<'EOF'
usage: provision.sh --branch <deploy-branch> --ntp-servers "<server> [<server>...]" [options]
       provision.sh --enable-units   (separate, deliberate action — see README.md)
       provision.sh --list

required for a provisioning run:
  --branch B           branch the Pi checkout ~/ws follows (LOCAL_ENV.md §2, "Working checkout on the Pi")
  --ntp-servers "S"    NTP sources for systemd-timesyncd, space-separated
                       (the laptop's hostname + address, LOCAL_ENV.md §2 "Internet path")
options:
  --agent-image-tar P  load the micro-ROS agent image from a `docker save` tarball instead of pulling it
  --skip-upgrade       skip step 5 (apt full-upgrade)
  --hold-auto-upgrades stop and disable apt-daily/unattended-upgrades (reversible, see below);
                       applied at the start of step 4
  --release-auto-upgrades  (separate action) undo --hold-auto-upgrades
  --skip-platformio    skip step 16 (PlatformIO firmware toolchain)
  --from N             start at step N (earlier steps are assumed done)
  --only N             run step N only
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --branch)          BRANCH="${2:-}"; shift 2 ;;
    --ntp-servers)     NTP_SERVERS="${2:-}"; shift 2 ;;
    --agent-image-tar) AGENT_TAR="${2:-}"; shift 2 ;;
    --skip-upgrade)    SKIP_UPGRADE=1; shift ;;
    --hold-auto-upgrades) HOLD_AUTO_UPGRADES=1; shift ;;
    --skip-platformio) SKIP_PLATFORMIO=1; shift ;;
    --release-auto-upgrades) ACTION=release; shift ;;
    --from)            FROM_STEP="${2:-}"; shift 2 ;;
    --only)            ONLY_STEP="${2:-}"; shift 2 ;;
    --enable-units)    ACTION=enable; shift ;;
    --list)            ACTION=list; shift ;;
    -h|--help)         usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

STEP_NAMES=(
  "preflight checks"
  "time sync, timezone, locale"
  "deploy repo (bare repo + working checkout)"
  "boot firmware config (config.txt, cmdline.txt, serial console)"
  "apt sources (Ubuntu + pinned ROS 2 Jazzy snapshot)"
  "apt full-upgrade"
  "apt: system packages"
  "apt: ROS 2 Jazzy packages + version check"
  "python user packages (Feetech SDK) + numpy check"
  "groups and membership"
  "udev rules"
  "docker + micro-ROS agent image"
  "shell environment (FastDDS profile, .bashrc, git config)"
  "/etc/gripperx configuration"
  "colcon build (plain, no --symlink-install)"
  "systemd units and scripts (install; stack NOT enabled)"
  "PlatformIO firmware toolchain (install + package prefetch; never builds or flashes)"
  "summary"
)

if [ "$ACTION" = list ]; then
  for i in "${!STEP_NAMES[@]}"; do printf '%2d  %s\n' "$i" "${STEP_NAMES[$i]}"; done
  exit 0
fi

# ---- logging & failure reporting ---------------------------------------------
CUR_STEP="-"
CUR_NAME="startup"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/provision_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1

ts()   { date '+%H:%M:%S'; }
log()  { echo "[$(ts)] [step $CUR_STEP] $*"; }
warn() { echo "[$(ts)] [step $CUR_STEP] WARNING: $*"; }
die()  {
  echo
  echo "[$(ts)] [step $CUR_STEP] ##### FAILED: $*"
  echo "[$(ts)] Log: $LOG"
  echo "[$(ts)] Fix the cause, then re-run with --from $CUR_STEP (same other arguments)."
  exit 1
}
on_err() {
  local rc=$? line=$1 cmd=$2
  echo
  echo "[$(ts)] [step $CUR_STEP] ##### FAILED (exit $rc) in step $CUR_STEP ($CUR_NAME), line $line:"
  echo "        $cmd"
  echo "[$(ts)] Log: $LOG"
  echo "[$(ts)] Fix the cause, then re-run with --from $CUR_STEP (same other arguments)."
  exit "$rc"
}
trap 'on_err $LINENO "$BASH_COMMAND"' ERR

# ---- helpers -----------------------------------------------------------------
# ROS setup scripts reference unset variables; source them with -u relaxed.
source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
}

apt_get() {
  sudo env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l NEEDRESTART_SUSPEND=1 \
    apt-get -o DPkg::Lock::Timeout=900 -o Acquire::Retries=5 \
            -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold "$@"
}

# install_file SRC DEST MODE OWNER:GROUP — installs only when content/mode differ.
install_file() {
  local src=$1 dest=$2 mode=$3 og=$4
  [ -f "$src" ] || die "source file missing: $src"
  if sudo test -f "$dest" && sudo cmp -s "$src" "$dest" \
     && [ "$(sudo stat -c '%a %U:%G' "$dest")" = "${mode#0} $og" ]; then
    log "unchanged  $dest"
  else
    sudo install -D -m "$mode" -o "${og%%:*}" -g "${og##*:}" "$src" "$dest"
    log "installed  $dest"
  fi
}

# install_text DEST MODE OWNER:GROUP <<content — same, from stdin.
install_text() {
  local tmp; tmp=$(mktemp)
  cat > "$tmp"
  install_file "$tmp" "$1" "$2" "$3"
  rm -f "$tmp"
}

pkg_installed() { dpkg-query -W -f='${db:Status-Abbrev}' "$1" 2>/dev/null | grep -q '^ii'; }

# On a first boot apt-daily / unattended-upgrades run in the background and hold
# the dpkg lock. Wait for them instead of fighting over the lock. (The resident
# unattended-upgrade-shutdown helper is deliberately not matched.)
wait_apt_idle() {
  local i busy
  for i in $(seq 1 120); do
    busy=$(systemctl is-active apt-daily.service apt-daily-upgrade.service unattended-upgrades-run.service 2>/dev/null | grep -cx 'active\|activating' || true)
    if [ "$busy" -eq 0 ] && ! pgrep -f '/usr/bin/unattended-upgrade( |$)|/usr/lib/apt/apt.systemd.daily' >/dev/null; then
      [ "$i" -eq 1 ] || log "apt background jobs finished"
      return 0
    fi
    [ "$i" -eq 1 ] && log "waiting for apt-daily / unattended-upgrades to finish (up to 20 min)"
    sleep 10
  done
  die "apt background jobs still running after 20 min (ps: $(pgrep -af '/usr/bin/unattended-upgrade( |$)|apt.systemd.daily' | head -3))"
}

# --hold-auto-upgrades: reversible, labelled, opt-in. Keeps automatic package
# changes off the robot until --release-auto-upgrades (e.g. after a demo).
readonly HOLD_CONF=/etc/apt/apt.conf.d/99gripperx-hold-auto-upgrades
readonly AUTO_TIMERS=(apt-daily.timer apt-daily-upgrade.timer)
hold_auto_upgrades() {
  log "[auto-upgrade HOLD] disabling ${AUTO_TIMERS[*]} and unattended-upgrades (undo: provision.sh --release-auto-upgrades)"
  sudo systemctl disable --now "${AUTO_TIMERS[@]}" >/dev/null 2>&1 || true
  install_text "$HOLD_CONF" 0644 root:root <<'EOF'
// GripperX — automatic apt activity HELD by provision.sh --hold-auto-upgrades.
// Undo with: provision.sh --release-auto-upgrades
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
EOF
  log "[auto-upgrade HOLD] active"
}
release_auto_upgrades() {
  sudo rm -f "$HOLD_CONF"
  sudo systemctl enable --now "${AUTO_TIMERS[@]}" >/dev/null
  log "[auto-upgrade HOLD] released: ${AUTO_TIMERS[*]} enabled, $HOLD_CONF removed"
}

# Resolve what `apt-get install ROS_PKGS` would install (dry run) and compare
# the resulting ros-jazzy-* set with the reference, BEFORE the long install.
# Expected packages the closure would not pull in are added to ROS_EXTRA as
# explicit name=version pins (all come from the pinned snapshot).
ROS_EXTRA=()
ros_plan() {
  local expected="$PROV/reference/ros-jazzy-expected.txt" sim planned missing unexpected wrongver
  sim=$(sudo apt-get -s -o Debug::NoLocking=1 install "${ROS_PKGS[@]}" 2>&1) || { echo "$sim" | tail -20; die "apt-get -s install of the ROS packages failed"; }
  # packages the dry run would install + ros-jazzy packages already installed
  planned=$( { echo "$sim" | awk '/^Inst ros-jazzy-/{v=$3; if (v ~ /^\[/) v=$4; gsub(/[()]/,"",v); print $2, v}';
               { dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'ros-jazzy-*' 2>/dev/null || true; } | awk '$1=="ii"{print $2, $3}'; } | sort -u)
  # (|| true: on a fresh card no ros-jazzy-* package exists yet, dpkg-query exits 1, and under
  #  pipefail that killed step 4 on the first real run, 2026-09-30.)
  wrongver=$(join <(echo "$planned") <(sort "$expected") | awk '$2!=$3{print "  " $1 ": plan " $2 ", reference " $3}')
  unexpected=$(comm -23 <(echo "$planned" | awk '{print $1}' | sort -u) <(awk '{print $1}' "$expected" | sort))
  missing=$(comm -13 <(echo "$planned" | awk '{print $1}' | sort -u) <(awk '{print $1}' "$expected" | sort))
  if [ -n "$wrongver" ] || [ -n "$unexpected" ]; then
    [ -z "$wrongver" ] || { echo "version differences:"; echo "$wrongver"; }
    [ -z "$unexpected" ] || { echo "packages the reference did not have:"; echo "$unexpected" | sed 's/^/  /'; }
    die "the ROS install plan does not match reference/ros-jazzy-expected.txt"
  fi
  ROS_EXTRA=()
  local n
  for n in $missing; do ROS_EXTRA+=("$n=$(awk -v n="$n" '$1==n{print $2}' "$expected")"); done
  if [ "${#ROS_EXTRA[@]}" -gt 0 ]; then
    warn "${#ROS_EXTRA[@]} reference packages are not in the dependency closure; they will be installed explicitly at the pinned version: ${ROS_EXTRA[*]}"
  fi
  log "ROS install plan matches the reference ($(echo "$planned" | wc -l) planned/installed + ${#ROS_EXTRA[@]} explicit)"
}

# cloud-init injected the old card's SSH host keys through /boot/firmware/user-data
# (CARRY_OVER.md). That file is on vfat and world-readable: once the first boot
# has consumed it, the private keys must leave it. Idempotent.
strip_userdata_host_keys() {
  local ud=/boot/firmware/user-data tmp
  sudo test -f "$ud" || { log "no $ud — nothing to strip"; return 0; }
  if ! sudo grep -q 'PRIVATE KEY' "$ud"; then
    log "$ud holds no private keys"
    return 0
  fi
  [ -f /etc/ssh/ssh_host_ed25519_key.pub ] || die "host keys not yet written by cloud-init — do not strip $ud before the first boot has completed"
  tmp=$(mktemp)
  # drop the top-level `ssh_keys:` mapping: from its line up to the next top-level key
  sudo cat "$ud" | awk '/^ssh_keys:/{skip=1; next} skip && /^[^[:space:]#]/{skip=0} !skip{print}' > "$tmp"
  grep -q 'PRIVATE KEY' "$tmp" && { rm -f "$tmp"; die "private key material in $ud outside a top-level ssh_keys: block — remove it by hand"; }
  python3 -c 'import sys, yaml; yaml.safe_load(open(sys.argv[1]))' "$tmp" \
    || { rm -f "$tmp"; die "stripping ssh_keys would leave $ud as invalid YAML — edit it by hand"; }
  sudo cp "$tmp" "$ud"   # vfat: plain cp
  rm -f "$tmp"
  sudo grep -q 'PRIVATE KEY' "$ud" && die "$ud still contains a private key after stripping"
  log "removed the ssh_keys: block (private host keys) from $ud"
}

# ---- steps -------------------------------------------------------------------

step_0() {
  [ "$(id -un)" = "$PI_USER" ] || die "run as '$PI_USER', not '$(id -un)' (the script uses sudo itself)"
  sudo -n true 2>/dev/null || die "passwordless sudo is not available for $PI_USER (the button daemon needs it too: sudo -n)"
  [ "$(uname -m)" = aarch64 ] || die "architecture is $(uname -m), expected aarch64"
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${VERSION_ID:-}" = "24.04" ] || die "OS is ${PRETTY_NAME:-unknown}, expected Ubuntu 24.04"
  local model; model=$(tr -d '\0' 2>/dev/null < /proc/device-tree/model || true)
  case "$model" in *"Raspberry Pi 5"*) ;; *) die "board is '$model', expected a Raspberry Pi 5" ;; esac
  [ "$(hostname)" = "$EXPECTED_HOSTNAME" ] || die "hostname is '$(hostname)', expected '$EXPECTED_HOSTNAME'"
  log "user=$(id -un) os='${PRETTY_NAME}' kernel=$(uname -r) board='$model'"

  # The one change step 0 makes, on purpose on every run: CARRY_OVER.md (c).
  strip_userdata_host_keys

  local active
  active=$(systemctl list-units --no-legend --state=active 'gripperx*' 2>/dev/null | awk '{print $1}' | grep -v '^gripperx-wifi' || true)
  [ -z "$active" ] || die "gripperx units are ACTIVE ($active). This script must not run under a live stack — stopping it is an SR-1 decision, not this script's."

  local free_gb; free_gb=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
  [ "$free_gb" -ge 12 ] || die "only ${free_gb} GB free on /, need >= 12 GB"
  log "free space on /: ${free_gb} GB"

  local host
  for host in ports.ubuntu.com snapshots.ros.org registry-1.docker.io pypi.org files.pythonhosted.org; do
    if getent hosts "$host" >/dev/null; then log "DNS ok: $host"; else warn "cannot resolve $host — the step that needs it will fail"; fi
  done
}

step_1() {
  [ -n "$NTP_SERVERS" ] || die "--ntp-servers is required (see LOCAL_ENV.md §2 on the laptop)"
  # The reference card uses systemd-timesyncd (chrony is not installed there).
  install_text /etc/systemd/timesyncd.conf.d/10-gripperx.conf 0644 root:root <<EOF
# GripperX — time source. Installed by Software/pi_env/provision/provision.sh.
# The Pi has no RTC: until one of these answers, the clock is wrong.
[Time]
NTP=$NTP_SERVERS
FallbackNTP=ntp.ubuntu.com
EOF
  sudo systemctl restart systemd-timesyncd
  if [ "$(timedatectl show -p Timezone --value)" != "$TIMEZONE" ]; then
    sudo timedatectl set-timezone "$TIMEZONE"; log "timezone set to $TIMEZONE"
  else
    log "timezone already $TIMEZONE"
  fi
  if ! locale -a 2>/dev/null | grep -qix 'en_US.utf8'; then
    sudo locale-gen en_US.UTF-8
  fi
  sudo update-locale LANG=en_US.UTF-8
  log "waiting up to 120 s for NTP sync (apt signatures and TLS fail on a wrong clock)"
  local i
  for i in $(seq 1 60); do
    [ "$(timedatectl show -p NTPSynchronized --value)" = yes ] && break
    sleep 2
  done
  [ "$(timedatectl show -p NTPSynchronized --value)" = yes ] \
    || die "clock not NTP-synchronised after 120 s (date: $(date)). Check that '$NTP_SERVERS' is reachable: timedatectl timesync-status"
  [ "$(date +%Y)" -ge 2026 ] || die "clock says $(date) — not plausible"
  log "clock synchronised: $(date) via $(timedatectl show-timesync -p ServerName --value 2>/dev/null || echo '?')"
}

step_2() {
  [ -n "$BRANCH" ] || die "--branch is required (see LOCAL_ENV.md §2 on the laptop)"
  command -v git >/dev/null || die "git is not installed on this image"
  if [ ! -d "$BARE" ]; then
    mkdir -p "$(dirname "$BARE")"
    git init --bare --quiet "$BARE"
    log "created bare deploy repo $BARE"
  fi
  [ "$(git -C "$BARE" rev-parse --is-bare-repository)" = true ] || die "$BARE exists but is not a bare repository"
  if ! git -C "$BARE" rev-parse --verify -q "refs/heads/$BRANCH" >/dev/null; then
    echo
    echo "  The deploy repo is empty. On the LAPTOP, in the workspace root, push the branches"
    echo "  (via the SSH alias from LOCAL_ENV.md §2 — mDNS/.local is not up until step 6):"
    echo "      git push <ssh-alias>:repos/gripperx_ws.git $BRANCH pi-history-2026-08-25 refs/remotes/pi/main:refs/heads/main"
    echo "  then re-run this script with --from 2."
    echo
    die "branch '$BRANCH' not yet pushed to $BARE"
  fi
  if [ ! -d "$WS/.git" ]; then
    [ ! -e "$WS" ] || die "$WS exists but is not a git checkout — move it away first"
    git clone --quiet --branch "$BRANCH" "$BARE" "$WS"
    log "cloned $BARE ($BRANCH) into $WS"
  fi
  [ "$(git -C "$WS" remote get-url origin)" = "$BARE" ] || die "$WS origin is $(git -C "$WS" remote get-url origin), expected $BARE"
  [ "$(git -C "$WS" rev-parse --abbrev-ref HEAD)" = "$BRANCH" ] || die "$WS is on branch $(git -C "$WS" rev-parse --abbrev-ref HEAD), expected $BRANCH"
  git -C "$WS" fetch --quiet origin
  if [ -n "$(git -C "$WS" status --porcelain --untracked-files=no)" ]; then
    die "$WS has uncommitted changes — commit them first (DEPLOYMENT.md: everything on the Pi is committed)"
  fi
  if [ "$(git -C "$WS" rev-parse HEAD)" != "$(git -C "$WS" rev-parse "origin/$BRANCH")" ]; then
    git -C "$WS" merge --ff-only --quiet "origin/$BRANCH" || die "$WS cannot fast-forward to origin/$BRANCH"
  fi
  git -C "$WS" branch --quiet --set-upstream-to="origin/$BRANCH" "$BRANCH"
  log "checkout: $WS at $(git -C "$WS" log --oneline -1)"
  check_self
}

# From step 2 on, the script reads its inputs from the checkout. Make sure the
# copy that is running is the committed one, so the log describes what was done.
check_self() {
  [ -f "$PROV/provision.sh" ] || die "$PROV/provision.sh not found — run step 2 first"
  local self; self=$(readlink -f "$0")
  if [ "$self" != "$(readlink -f "$PROV/provision.sh")" ] && ! cmp -s "$self" "$PROV/provision.sh"; then
    die "the running script ($self) differs from $PROV/provision.sh — run the committed copy: bash $PROV/provision.sh ..."
  fi
}

step_3() {
  local boot=/boot/firmware ref="$PROV/reference/boot" stamp; stamp=$(date +%Y%m%d_%H%M%S)
  [ -f "$boot/config.txt" ] || die "$boot/config.txt not found — is the boot partition mounted?"

  if cmp -s "$boot/config.txt" "$ref/config.txt"; then
    log "config.txt identical to the reference card"
  else
    diff -u "$boot/config.txt" "$ref/config.txt" || true
    die "config.txt differs from the reference card (diff above: - this card, + reference). Not overwritten: resolve by hand — the hardware interfaces (UART, I2C, SPI) depend on it"
  fi

  # cmdline.txt: the reference card is the stock line minus `console=serial0,115200`
  # (removed 2026-07-02 so the kernel console does not share the LiDAR UART).
  # Only that token is edited — root=/rootfstype= are never rewritten blindly.
  local cur new want
  cur=$(tr -s ' \n' ' ' < "$boot/cmdline.txt" | sed 's/^ //; s/ $//')
  new=$(echo "$cur" | tr ' ' '\n' | grep -vE '^console=(serial0|ttyAMA0),' | tr '\n' ' ' | sed 's/ $//')
  want=$(tr -s ' \n' ' ' < "$ref/cmdline.txt" | sed 's/^ //; s/ $//')
  if [ "$new" != "$want" ]; then
    echo "  current  : $cur"
    echo "  reference: $want"
    die "cmdline.txt differs from the reference beyond the serial console token — resolve by hand, not by guessing"
  fi
  if [ "$cur" != "$new" ]; then
    sudo cp "$boot/cmdline.txt" "$boot/cmdline.txt.pre-gripperx-$stamp"
    printf '%s\n' "$new" | sudo tee "$boot/cmdline.txt" >/dev/null
    log "removed the serial console from cmdline.txt (backup: cmdline.txt.pre-gripperx-$stamp)"
  else
    log "cmdline.txt identical to the reference card"
  fi

  if [ "$(systemctl is-enabled serial-getty@ttyAMA0.service 2>/dev/null || true)" != masked ]; then
    sudo systemctl mask serial-getty@ttyAMA0.service
    log "masked serial-getty@ttyAMA0.service (LiDAR UART)"
  else
    log "serial-getty@ttyAMA0.service already masked"
  fi
  log "boot changes take effect at the next reboot"
}

step_4() {
  [ "$HOLD_AUTO_UPGRADES" = 1 ] && hold_auto_upgrades
  wait_apt_idle
  if ! command -v gpg >/dev/null; then
    log "gpg missing — installing gnupg from the Ubuntu archive first"
    apt_get update
    apt_get -y install gnupg
  fi
  install_file "$PROV/keys/ros-snapshots-archive-keyring.gpg" "$ROS_SNAPSHOT_KEYRING" 0644 root:root
  local fpr
  fpr=$(gpg --show-keys --with-colons "$ROS_SNAPSHOT_KEYRING" 2>/dev/null | awk -F: '/^fpr:/{print $10; exit}')
  [ "$fpr" = "$ROS_SNAPSHOT_FPR" ] || die "ROS snapshot key fingerprint is '$fpr', expected $ROS_SNAPSHOT_FPR"
  log "ROS snapshot key fingerprint verified"

  # Any other ROS source would let apt pick newer ROS versions than the reference.
  local f
  for f in /etc/apt/sources.list.d/*; do
    [ -f "$f" ] || continue
    [ "$f" = "$ROS_SNAPSHOT_LIST" ] && continue
    if grep -q 'packages\.ros\.org' "$f"; then
      sudo mv "$f" "$f.disabled-by-gripperx"
      warn "disabled $f (ROS is pinned to the snapshot)"
    fi
  done
  install_text "$ROS_SNAPSHOT_LIST" 0644 root:root <<EOF
# GripperX — ROS 2 Jazzy pinned to the 2026-06-18 sync, the exact versions the
# reference card ran. Installed by Software/pi_env/provision/provision.sh.
deb [arch=arm64 signed-by=$ROS_SNAPSHOT_KEYRING] $ROS_SNAPSHOT_URL noble main
EOF
  apt_get update --error-on=any
  local cand want
  cand=$(apt-cache policy ros-jazzy-ros-base | awk '/Candidate:/{print $2}')
  want=$(awk '$1=="ros-jazzy-ros-base"{print $2}' "$PROV/reference/ros-jazzy-expected.txt")
  [ "$cand" = "$want" ] || die "apt candidate for ros-jazzy-ros-base is '$cand', expected '$want'"
  log "apt sources ok (ros-jazzy-ros-base candidate $cand)"
  ros_plan
}

step_5() {
  if [ "$SKIP_UPGRADE" = 1 ]; then log "skipped (--skip-upgrade)"; return; fi
  wait_apt_idle
  apt_get -y full-upgrade
  log "full-upgrade done; running kernel $(uname -r), installed $(dpkg-query -W -f='${Version}' linux-image-raspi 2>/dev/null || echo '?')"
}

step_6() {
  wait_apt_idle
  apt_get -y install "${SYSTEM_PKGS[@]}"
  local p missing=()
  for p in "${SYSTEM_PKGS[@]}"; do pkg_installed "$p" || missing+=("$p"); done
  [ "${#missing[@]}" -eq 0 ] || die "not installed after apt: ${missing[*]}"
  log "system packages installed (${#SYSTEM_PKGS[@]})"
}

step_7() {
  wait_apt_idle
  ros_plan
  apt_get -y install "${ROS_PKGS[@]}" "${ROS_EXTRA[@]}"
  local expected="$PROV/reference/ros-jazzy-expected.txt" name want have bad=0 n=0
  while read -r name want; do
    [ -n "$name" ] || continue
    n=$((n + 1))
    have=$(dpkg-query -W -f='${db:Status-Abbrev}|${Version}' "$name" 2>/dev/null || true)
    if [ "${have%%|*}" != "ii " ] || [ "${have#*|}" != "$want" ]; then
      echo "  MISMATCH $name: want $want, have '${have#*|}'"
      bad=$((bad + 1))
    fi
  done < "$expected"
  [ "$bad" -eq 0 ] || die "$bad of $n ROS packages differ from the reference card"
  local extra
  extra=$(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' 'ros-jazzy-*' 2>/dev/null \
          | awk '$1=="ii"{print $2}' | sort | comm -23 - <(awk '{print $1}' "$expected" | sort) || true)
  [ -z "$extra" ] || warn "ros-jazzy packages installed that the reference did not have: $(echo "$extra" | tr '\n' ' ')"
  log "all $n reference ROS packages installed at the reference version"
}

step_8() {
  # numpy: apt's 1.26.4 only. The reference card had numpy 2.2.6 in ~/.local
  # (left over from the removed LeRobot stack); nothing in the stack needs it —
  # see README.md "numpy decision".
  local user_site="$PI_HOME/.local/lib/python3.12/site-packages"
  if [ -d "$user_site/numpy" ]; then
    die "a user-site numpy exists in $user_site — it would shadow apt's numpy; remove it (python3 -m pip uninstall --break-system-packages numpy) and re-run"
  fi
  if python3 -m pip show feetech-servo-sdk 2>/dev/null | grep -q '^Version: 1\.0\.0$'; then
    log "feetech-servo-sdk 1.0.0 already installed"
  else
    python3 -m pip install --user --break-system-packages --no-deps --no-build-isolation \
      --require-hashes -r "$PROV/requirements-pi-user.txt"
    log "installed feetech-servo-sdk 1.0.0 (--user)"
  fi
  local out
  out=$(python3 -c 'import numpy, scservo_sdk, serial, gpiod, websockets, yaml; print(numpy.__version__, numpy.__file__, scservo_sdk.__file__)') \
    || die "python import check failed"
  log "python: $out"
  case "$out" in
    *" /usr/lib/python3/dist-packages/numpy/"*) ;;
    *) die "numpy does not resolve to the apt package: $out" ;;
  esac
  case "$out" in
    *"$user_site/scservo_sdk/"*) ;;
    *) die "scservo_sdk does not resolve to $user_site: $out" ;;
  esac
}

step_9() {
  # Nothing on the robot uses numeric gids (udev rules name groups), so the
  # reference gids 1000/1001/1002 are only reused when they happen to be free.
  local g pref
  for g in i2c:1000 spi:1001 gpio:1002; do
    pref=${g#*:}; g=${g%%:*}
    if getent group "$g" >/dev/null; then
      log "group $g exists (gid $(getent group "$g" | cut -d: -f3))"
    else
      if getent group "$pref" >/dev/null; then sudo groupadd "$g"; else sudo groupadd -g "$pref" "$g"; fi
      log "created group $g (gid $(getent group "$g" | cut -d: -f3))"
    fi
  done
  local want=(dialout docker i2c spi gpio video plugdev) have missing=()
  have=" $(id -nG "$PI_USER") "
  for g in "${want[@]}"; do
    getent group "$g" >/dev/null || die "group $g does not exist (docker.io installed? step 6)"
    case "$have" in *" $g "*) ;; *) missing+=("$g") ;; esac
  done
  if [ "${#missing[@]}" -gt 0 ]; then
    sudo usermod -aG "$(IFS=,; echo "${missing[*]}")" "$PI_USER"
    log "added $PI_USER to: ${missing[*]} (effective after re-login / reboot)"
  else
    log "$PI_USER already in: ${want[*]}"
  fi
}

step_10() {
  install_file "$PI_ENV/udev/99-gripperx.rules" /etc/udev/rules.d/99-gripperx.rules 0644 root:root
  install_file "$PI_ENV/udev/99-lidar.rules"    /etc/udev/rules.d/99-lidar.rules    0644 root:root
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=tty --action=change
  sudo udevadm settle --timeout=10 || true
  local d
  for d in esp32 steering_servo arm_servo lidar; do
    if [ -e "/dev/$d" ]; then log "/dev/$d -> $(readlink "/dev/$d")"
    else warn "/dev/$d not present now (device unplugged, or ttyAMA0 appears only after the reboot) — VERIFY.md checks it again"; fi
  done
}

step_11() {
  sudo systemctl enable --now docker.service >/dev/null
  local ref_digest="$AGENT_REPO@$AGENT_DIGEST"
  if [ -n "$AGENT_TAR" ]; then
    [ -f "$AGENT_TAR" ] || die "--agent-image-tar $AGENT_TAR: file not found"
    sudo docker load -i "$AGENT_TAR"
  elif sudo docker image inspect "$ref_digest" >/dev/null 2>&1; then
    log "agent image $ref_digest already present"
  else
    local i
    for i in 1 2 3; do
      sudo docker pull "$ref_digest" && break
      [ "$i" -lt 3 ] || die "docker pull $ref_digest failed 3 times (internet via the laptop NAT?). Alternative: --agent-image-tar, see CARRY_OVER.md"
      warn "docker pull failed (attempt $i), retrying in 10 s"; sleep 10
    done
  fi
  if [ -z "$AGENT_TAR" ]; then
    sudo docker tag "$ref_digest" "$AGENT_REPO:$AGENT_TAG"
  fi
  local info
  info=$(sudo docker image inspect "$AGENT_REPO:$AGENT_TAG" --format '{{.Id}} {{json .RepoDigests}}' 2>/dev/null) \
    || die "$AGENT_REPO:$AGENT_TAG not present after step 11"
  case "$info" in
    *"$AGENT_DIGEST"*) log "agent image ok: $AGENT_REPO:$AGENT_TAG = $AGENT_DIGEST" ;;
    *) if [ -n "$AGENT_TAR" ]; then
         warn "loaded image does not show the reference digest ($info) — acceptable only if the tarball was saved from the reference card"
       else
         die "$AGENT_REPO:$AGENT_TAG is not the reference image ($info)"
       fi ;;
  esac
}

step_12() {
  install_file "$PI_ENV/fastdds_udp_only.xml" "$PI_HOME/fastdds_udp_only.xml" 0664 "$PI_USER:$(id -gn "$PI_USER")"

  # One marked block, replaced as a whole on every run. `source ~/microros_ws/...`
  # from the reference .bashrc is deliberately dropped: ~/microros_ws is not
  # provisioned and no unit uses it (the agent runs from the Docker image).
  local bashrc="$PI_HOME/.bashrc" begin='# >>> gripperx provision >>>' end='# <<< gripperx provision <<<'
  local block tmp; tmp=$(mktemp)
  block=$(cat <<'EOF'
# >>> gripperx provision >>>
# Managed by Software/pi_env/provision/provision.sh — edits inside this block are overwritten.
# ROS 2 Jazzy, same domain/RMW/profile as the gripperx-*.service scripts (OP-8).
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
# <<< gripperx provision <<<
EOF
)
  [ -f "$bashrc" ] || cp /etc/skel/.bashrc "$bashrc"
  local rest
  # $(...) strips trailing newlines, so a re-run produces byte-identical output
  rest=$(awk -v b="$begin" -v e="$end" '$0==b{skip=1} !skip{print} $0==e{skip=0}' "$bashrc")
  printf '%s\n\n%s\n' "$rest" "$block" > "$tmp"
  if cmp -s "$tmp" "$bashrc"; then log "unchanged  $bashrc"; else cp "$tmp" "$bashrc"; log "updated    $bashrc"; fi
  rm -f "$tmp"

  # Same global git settings as the reference card (flaky NAT'd link).
  git config --global http.version HTTP/1.1
  git config --global http.postBuffer 524288000
  git config --global credential.helper ""
  log "git global config set (http.version HTTP/1.1, http.postBuffer, credential.helper empty)"
}

step_13() {
  local f
  sudo install -d -m 0755 -o root -g root /etc/gripperx
  for f in "$PI_ENV"/systemd/config/*; do
    install_file "$f" "/etc/gripperx/$(basename "$f")" 0644 root:root
  done
}

step_14() {
  [ -d "$COLCON_ROOT/src" ] || die "$COLCON_ROOT/src not found"
  cd "$COLCON_ROOT"
  source_ros

  # Warn (not fail) on package.xml dependencies that resolve to nothing in
  # /opt/ros — the build is the hard gate, this names a missing runtime dep early.
  python3 - "$COLCON_ROOT/src" <<'PY' || true
import pathlib, re, sys, subprocess
src = pathlib.Path(sys.argv[1])
ws = {}
for px in src.rglob("package.xml"):
    if any(p.name == "COLCON_IGNORE" for p in px.parent.iterdir()):
        continue
    text = px.read_text()
    name = re.search(r"<name>\s*([^<\s]+)\s*</name>", text).group(1)
    ws[name] = re.findall(r"<(?:depend|exec_depend|build_depend|buildtool_depend|build_export_depend)(?:\s[^>]*)?>\s*([^<\s]+)\s*<", text)
system = {"eigen": "libeigen3-dev", "cmake_modules": None}
missing = []
for pkg, deps in sorted(ws.items()):
    for d in deps:
        if d in ws:
            continue
        if d.startswith("python3-"):
            ok = subprocess.run(["dpkg-query", "-W", d], capture_output=True).returncode == 0
        elif d in system:
            ok = system[d] is None or subprocess.run(["dpkg-query", "-W", system[d]], capture_output=True).returncode == 0
        else:
            ok = pathlib.Path("/opt/ros/jazzy/share", d, "package.xml").exists()
        if not ok:
            missing.append(f"{pkg} -> {d}")
print("dependency pre-check: %d workspace packages, %s" % (len(ws), "all deps present" if not missing else "UNRESOLVED: " + ", ".join(missing)))
PY

  local blog="$LOG_DIR/colcon_build_$(date +%Y%m%d_%H%M%S).log"
  log "colcon build (plain, 2 workers) in $COLCON_ROOT — log: $blog"
  # NEVER --symlink-install on the Pi (DEPLOYMENT.md): it strips egg-info and
  # leaves the drive stack in a restart loop.
  # 2 workers: the SD card and 8 GB without swap, not the CPU, are the limit
  if ! colcon build --parallel-workers 2 2>&1 | tee "$blog"; then
    die "colcon build failed — see $blog"
  fi
  grep -q 'packages finished' "$blog" || die "colcon build produced no summary — see $blog"
  if grep -qE 'packages? (failed|aborted)' "$blog"; then die "colcon build reported failures — see $blog"; fi

  local listed built bad=0 name path type
  listed=$(colcon list --names-only | sort)
  built=$(ls "$COLCON_ROOT/install" | grep -vE '^(COLCON_IGNORE|setup\.|local_setup\.|_local_setup)' | sort)
  if [ "$listed" != "$built" ]; then
    diff <(echo "$listed") <(echo "$built") || true
    die "install/ does not contain exactly the packages colcon lists"
  fi
  while read -r name path type; do
    if [ "$type" = "(ros.ament_python)" ]; then
      if ! ls -d "$COLCON_ROOT/install/$name"/lib/python3.12/site-packages/*.egg-info >/dev/null 2>&1; then
        echo "  no egg-info for $name"; bad=1
      fi
    fi
  done < <(colcon list)
  [ "$bad" -eq 0 ] || die "ament_python packages without egg-info — was --symlink-install used? (DEPLOYMENT.md recovery)"
  log "build complete: $(echo "$listed" | wc -l) packages ($(echo "$listed" | grep -c '^gripperx_') gripperx_*), egg-info present for every ament_python package"
}

step_15() {
  local f
  for f in "$PI_ENV"/systemd/scripts/*; do
    install_file "$f" "/usr/local/bin/$(basename "$f")" 0755 root:root
  done
  for f in "$PI_ENV"/systemd/units/*; do
    install_file "$f" "/etc/systemd/system/$(basename "$f")" 0644 root:root
  done
  sudo systemctl daemon-reload
  local u
  for u in "${SAFE_UNITS[@]}"; do
    if systemctl is-enabled --quiet "$u" 2>/dev/null; then log "$u already enabled"
    else sudo systemctl enable "$u" >/dev/null; log "enabled $u (active after the reboot)"; fi
  done
  for u in "${STACK_UNITS[@]}"; do
    log "$u: $(systemctl is-enabled "$u" 2>/dev/null || true) (stack units are enabled only by --enable-units)"
  done
}

step_16() {
  if [ "$SKIP_PLATFORMIO" = 1 ]; then log "skipped (--skip-platformio)"; return; fi
  # The ESP32 is NEVER touched here: no `pio run`, no upload, no monitor. A firmware
  # change is motion-relevant and needs its own user approval.
  if [ -x "$PIO_VENV/bin/pio" ] && "$PIO_VENV/bin/pio" --version 2>/dev/null | grep -q "version $PIO_VERSION\$"; then
    log "PlatformIO $PIO_VERSION already installed in $PIO_VENV"
  else
    [ -d "$PIO_VENV" ] || python3 -m venv "$PIO_VENV"
    "$PIO_VENV/bin/python" -m pip install --quiet --upgrade "platformio==$PIO_VERSION" "$PIO_CLICK_SPEC" \
      || die "pip install platformio==$PIO_VERSION into $PIO_VENV failed"
    log "installed PlatformIO $PIO_VERSION into $PIO_VENV"
  fi
  mkdir -p "$PI_HOME/.local/bin"
  local b
  for b in pio platformio; do
    ln -sfn "$PIO_VENV/bin/$b" "$PI_HOME/.local/bin/$b"
  done
  local ver
  ver=$("$PI_HOME/.local/bin/pio" --version) || die "pio does not run"
  case "$ver" in *"version $PIO_VERSION") ;; *) die "unexpected PlatformIO version: $ver" ;; esac
  log "$ver; click $("$PIO_VENV/bin/python" -c 'import click; print(click.__version__)'); ~/.local/bin/pio -> $PIO_VENV/bin/pio"

  # Prefetch so a later flash does not depend on the network. Non-fatal: a slow
  # link must not fail the provisioning; VERIFY.md shows what is present.
  if "$PIO_VENV/bin/pio" pkg install --global --platform "$PIO_PLATFORM"; then
    log "platform $PIO_PLATFORM installed"
  else
    warn "prefetch of $PIO_PLATFORM failed — re-run with --only 16 when the network is better"
  fi
  if [ -f "$FW_PROJECT/platformio.ini" ] && "$PIO_VENV/bin/pio" pkg install -d "$FW_PROJECT" -e "$FW_ENV"; then
    log "firmware project packages installed ($FW_PROJECT, env $FW_ENV)"
  else
    warn "prefetch of the firmware project packages failed — re-run with --only 16 when the network is better"
  fi
}

step_17() {
  cat <<EOF

=============================================================================
 Provisioning finished — $(date)
 Log: $LOG
 Checkout: $WS @ $(git -C "$WS" log --oneline -1 2>/dev/null || echo '?')

 NEXT (in this order, see README.md):
  1. Reboot the Pi ONCE (boot config, serial console, group membership):
        sudo reboot
     The stack units are NOT enabled, so this reboot starts no gripperx
     stack unit — only gripperx-wifi.timer.
  2. Work through Software/pi_env/provision/VERIFY.md.
  3. Only then, and as a deliberate act:  bash $PROV/provision.sh --enable-units
     That makes the NEXT BOOT start the stack — a bringup start is a motion
     trigger under SR-1 and needs the user's approval for that test.
=============================================================================
EOF
}

# ---- actions -----------------------------------------------------------------
enable_units() {
  CUR_STEP=E; CUR_NAME="enable stack units"
  [ -x /usr/local/bin/gripperx-bringup.sh ] || die "units not installed — run the provisioning first"
  [ -f "$COLCON_ROOT/install/setup.bash" ] || die "workspace not built — run the provisioning first"
  sudo docker image inspect "$AGENT_REPO:$AGENT_TAG" >/dev/null 2>&1 || die "agent image missing — run step 11"
  local u
  for u in "${STACK_UNITS[@]}" "${SAFE_UNITS[@]}"; do
    sudo systemctl enable "$u" >/dev/null
    log "enabled $u (NOT started)"
  done
  cat <<'EOF'

  The stack units are ENABLED, NOT STARTED. The NEXT BOOT starts gripperx-agent,
  -bringup, -mapping, -navigation, -external and the shutdown button daemon.
  A bringup start is a motion trigger under SR-1 (motor-enable process rule):
  get the user's approval for that test before rebooting.
EOF
}

case "$ACTION" in
  release)
    CUR_STEP=R; CUR_NAME="release auto-upgrade hold"
    release_auto_upgrades
    exit 0 ;;
  enable)
    enable_units
    exit 0 ;;
esac

log "GripperX provisioning — log $LOG"
log "arguments: branch='$BRANCH' ntp='$NTP_SERVERS' agent_tar='${AGENT_TAR:-}' skip_upgrade=$SKIP_UPGRADE from=$FROM_STEP only=${ONLY_STEP:-all}"
for i in "${!STEP_NAMES[@]}"; do
  # step 0 (preflight) always runs; it changes nothing
  if [ -n "$ONLY_STEP" ]; then [ "$i" = "$ONLY_STEP" ] || [ "$i" = 0 ] || continue
  elif [ "$i" -lt "$FROM_STEP" ] && [ "$i" -ne 0 ]; then continue; fi
  CUR_STEP=$i; CUR_NAME=${STEP_NAMES[$i]}
  [ "$i" -lt 3 ] || check_self
  echo
  echo "================ [$(ts)] STEP $i/$(( ${#STEP_NAMES[@]} - 1 )): $CUR_NAME ================"
  start=$(date +%s)
  "step_$i"
  log "done in $(( $(date +%s) - start )) s"
done
