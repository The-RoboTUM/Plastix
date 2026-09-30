# CARRY_OVER — what has to come from the OLD card, and what deliberately does not

**What:** the exact list of things that cannot be recreated from git or from `provision.sh`, and
how they reach the new card. **For:** whoever swaps the Pi's SD card. **Status:** rewritten
2026-09-30 for the cloud-init route (the new card has **not booted yet**); not yet executed.
**Runbook position:** steps A–C of [README.md](README.md).

**Why cloud-init and not a file copy:** on its first boot cloud-init deletes and regenerates the SSH
host keys (`ssh_deletekeys` defaults to true) and creates `/home/ubuntu`. Files copied onto an
unbooted card are therefore overwritten or have nowhere to go. Instead, the old card's keys are
read over SSH from the **still-running** old Pi and handed to cloud-init through the new card's
`user-data`, which applies them on that first boot.

---

## 0. Why the list is this short

The inventory of 2026-09-30 compared every stack-relevant file on the old card with the repository:

| On the old card | Result | Comes from |
| --- | --- | --- |
| `/usr/local/bin/gripperx-*`, `/etc/systemd/system/gripperx-*`, `/etc/gripperx/gripperx-button.conf` | `tools/deploy_check.sh --ref Theo`: all 18 files **same** | git → `provision.sh` step 13/15 |
| `/etc/udev/rules.d/99-gripperx.rules`, `99-lidar.rules`, `~/fastdds_udp_only.xml` | sha256 identical to `Software/pi_env/` | git → step 10/12 |
| `/boot/firmware/config.txt`, `cmdline.txt` | copied into `reference/boot/` | git → step 3 (checks; changes nothing if identical) |
| `~/ws` (branch `Theo`, `a9a0264`), `~/repos/gripperx_ws.git` (`Theo`, `main`, `pi-history-2026-08-25`) | the laptop holds all three refs at the same commits | laptop push → step 2 |
| micro-ROS agent image | pinned by digest `sha256:1d13bfeb…` | Docker Hub → step 11 |
| `~/.local/.../scservo_sdk` | 7 files byte-identical to the PyPI `feetech-servo-sdk==1.0.0` sdist | PyPI → step 8 |

What is left cannot come from anywhere else: the SSH **host identity**, and the **authorized
keys** of people other than this laptop.

---

## A. Stage over SSH from the running old Pi (laptop)

Read-only on the old Pi. Staging directory: private (0700), created with `mktemp`, outside any
repository, owned by the laptop user (no `sudo` on the laptop side). Every path variable is guarded
with `${VAR:?}` so an empty variable fails instead of expanding to `/`.

```bash
ALIAS=<ssh-alias>                                   # LOCAL_ENV.md §2
STAGE=$(mktemp -d "$HOME/gripperx_carry.XXXXXX"); chmod 700 "${STAGE:?}"
echo "staging in $STAGE"
for t in ecdsa ed25519 rsa; do
  timeout 60 ssh -o BatchMode=yes "${ALIAS:?}" "sudo -n cat /etc/ssh/ssh_host_${t}_key" > "${STAGE:?}/ssh_host_${t}_key"
  timeout 60 ssh -o BatchMode=yes "${ALIAS:?}" "cat /etc/ssh/ssh_host_${t}_key.pub"      > "${STAGE:?}/ssh_host_${t}_key.pub"
done
timeout 60 ssh -o BatchMode=yes "${ALIAS:?}" 'cat ~/.ssh/authorized_keys'              > "${STAGE:?}/authorized_keys"
chmod 600 "${STAGE:?}"/ssh_host_*_key
```

**Integrity check** — the old card corrupts files silently, so each private key must reproduce its
public key, and the fingerprints must be the ones recorded from the running card on 2026-09-30:

```bash
for t in ecdsa ed25519 rsa; do
  ssh-keygen -y -f "${STAGE:?}/ssh_host_${t}_key" | cut -d' ' -f1,2 \
    | cmp - <(cut -d' ' -f1,2 "${STAGE:?}/ssh_host_${t}_key.pub") && echo "$t: pair ok"
  ssh-keygen -l -f "${STAGE:?}/ssh_host_${t}_key.pub"
done
awk '{print $1, $NF}' "${STAGE:?}/authorized_keys"   # 3 × ssh-ed25519: 2 × dev@GripperX, 1 × blaesse@itqlm104
```

Expected fingerprints:

```
256  SHA256:DK58H93L54m5nJ/CNJrAdJAsicepZlDiJETYWqQdd4A root@GripperX-1 (ECDSA)
256  SHA256:kbVmykn0nd8gz6/yd9TMcq2/o+k0Xrit9ARZMJPtL8M root@GripperX-1 (ED25519)
3072 SHA256:I8sc1Q15iLW9vIl6ut4EKmNBXtQ7TqJKLMYAC+X+ZWo root@GripperX-1 (RSA)
```

A mismatch means a corrupted key: leave that key type out of `ssh_keys:` below (cloud-init then
generates it) and replace its `known_hosts` entry on the laptop after the first boot.

### A5. Optional, before the old Pi is shut down

`ARCHIVE` is an ordinary laptop directory, **not** `STAGE` (which is shredded in step D).

| What | Why | Command |
| --- | --- | --- |
| `~/ws/runs/` | untracked crab/yaw/breakaway measurement runs, not in git | `rsync -a "${ALIAS:?}":ws/runs/ "${ARCHIVE:?}/ws_runs/"` |
| micro-ROS agent image | only if the new Pi will have no internet (step 11 fallback) | `ssh "${ALIAS:?}" 'docker save microros/micro-ros-agent:jazzy' \| gzip > "${ARCHIVE:?}/micro-ros-agent_jazzy.tar.gz"` (~755 MB); later `provision.sh --agent-image-tar <path on the Pi>` |
| Feetech SDK | only if PyPI will be unreachable (step 8 fallback) | `rsync -a "${ALIAS:?}":.local/lib/python3.12/site-packages/{scservo_sdk,feetech_servo_sdk-1.0.0.dist-info} "${ARCHIVE:?}/sdk/"`; later copied to the same path on the new Pi, and step 8 skips the download |

---

## B. Inject into the new card's `user-data` (laptop, card unbooted)

The new card's boot partition (`system-boot`, vfat) holds `user-data`, the cloud-config that
already configures hostname, user and netplan. Generate the fragment:

```bash
F=$(mktemp); chmod 600 "${F:?}"
{
  echo "ssh_deletekeys: false"
  echo "ssh_keys:"
  for t in ecdsa ed25519 rsa; do
    echo "  ${t}_private: |"
    sed 's/^/    /' "${STAGE:?}/ssh_host_${t}_key"
    echo "  ${t}_public: $(cat "${STAGE:?}/ssh_host_${t}_key.pub")"
  done
} > "${F:?}"
```

Merge it into `user-data`:

1. `ssh_deletekeys: false` and the `ssh_keys:` block go in at **top level** (column 0). If
   `user-data` already has an `ssh_deletekeys:` line, replace it — no key may appear twice.
   `provision.sh` later strips exactly this top-level `ssh_keys:` block (see C).
2. **Authorized keys:** add every line of `$STAGE/authorized_keys` that is not already there to the
   list that already carries this laptop's key — `ssh_authorized_keys:` under the `ubuntu` entry of
   `users:`, or the top-level `ssh_authorized_keys:` if the file configures the default user that
   way. Do not create a second list.
3. Validate before unmounting:

```bash
UD=/media/$USER/system-boot/user-data                 # wherever the boot partition is mounted
head -1 "${UD:?}"                                       # must be: #cloud-config
python3 -c 'import sys, yaml; d = yaml.safe_load(open(sys.argv[1])); print(sorted(d)); print(sorted(d["ssh_keys"]))' "${UD:?}"
grep -c 'PRIVATE KEY-----' "${UD:?}"                    # 6 (BEGIN + END for three keys)
shred -u "${F:?}"; sync
```

---

## C. After the first boot — mandatory

1. **The laptop reaches the robot with its unchanged `known_hosts`:**

   ```bash
   ssh -o BatchMode=yes "${ALIAS:?}" hostname          # GripperX-1, no host-key warning, no prompt
   ```

   A host-key warning means cloud-init did not apply the keys: do **not** delete the `known_hosts`
   entry reflexively — read `/var/log/cloud-init.log` for `ssh_keys` first.
2. **Remove the private host keys from `/boot/firmware/user-data`.** The file is on vfat, readable
   by every user on the Pi, and cloud-init has consumed it. `provision.sh` does this in **step 0 of
   every run** (strips the top-level `ssh_keys:` block, checks the rest is still valid YAML, fails if
   key material is left), so the first `provision.sh` call in the runbook does it. Verify:

   ```bash
   ssh "${ALIAS:?}" 'sudo grep -c "PRIVATE KEY" /boot/firmware/user-data'     # 0
   ```

   cloud-init keeps its own copy of the processed user-data under `/var/lib/cloud/instance/`
   (root-only, 0600) — the same protection the keys have in `/etc/ssh`.

## D. Clean up the laptop

After C has passed:

```bash
shred -u "${STAGE:?}"/ssh_host_*_key && rm -r "${STAGE:?}"
```

---

## 4. Deliberately NOT carried over

| Item on the old card | Why not |
| --- | --- |
| `~/steer_center.json`, `~/steer_map.json`, `~/steering_calibration.yaml` | Not referenced by any code, config, launch file or systemd script (grep of the repo and of the installed scripts, 2026-09-30). Superseded: the live calibration is `center_counts` in `gripperx_control/config/steer_servo.yaml` (re-calibrated 2026-09-18; the home files date from 2026-07-01 / 2026-08-13 and carry different values and servo ids). |
| `~/.local` numpy 2.2.6, setuptools 80.10.2, rerun_sdk, aiohttp, … | Left over from the removed LeRobot stack. See README.md "numpy decision". |
| `~/.local` platformio, esptool, `~/.platformio` | Reinstalled fresh by step 16 (same PlatformIO and platform versions, own venv). |
| `~/microros_ws`, `~/microros_ws2` | No unit uses them — the agent runs from the Docker image. The `.bashrc` line that sourced `~/microros_ws` is dropped. |
| `~/ws/Software/ros2/build`, `install`, `log` | Rebuilt by step 14; the old copies may be corrupted. |
| `~/ws_backup_20260708`, `~/ws_old_layout`, `~/build`, `~/install`, `~/log` | Obsolete workspace layouts. |
| Ad-hoc scripts and logs in `~` (`calibrate_steering.py`, `slalom.py`, `moder_*.sh`, `*.log`, …) | Test tools and records of past sessions, not part of the stack. |
| `~/.ssh/config`, `~/.ssh/known_hosts` | The config points at a dead hotspot address with a key that does not exist; the Pi does not SSH out in operation. |
| `~/.gitconfig`, `~/.bashrc` | Recreated by step 12 (same settings; the `microros_ws` line dropped). |
| `/etc/systemd/timesyncd.conf` | Recreated as a drop-in by step 1 with the same sources. |
| `/etc/netplan/*` | Configured through the new card's cloud-init, identical to the old card. |
| `/etc/sudoers.d/90-cloud-init-users` | Created by cloud-init (passwordless sudo, which the button daemon's `sudo -n` relies on — checked by step 0). |
| `~/gripperx_journal_mirror`, `~/gripperx_docs_mirror` | Mirrors; the laptop is the source. |
| `~/.ros/log`, `~/.bash_history` | Logs/history. |
| `/var/lib/docker` | Image re-pulled by digest (step 11). |
