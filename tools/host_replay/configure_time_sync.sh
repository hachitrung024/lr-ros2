#!/usr/bin/env bash
# Run once before the pipeline. Existing configurations are backed up.
set -euo pipefail
if [[ $# != 3 || ( "$1" != host && "$1" != jetson ) ]]; then
  echo "Usage: sudo bash $0 host|jetson HOST_IP JETSON_IP" >&2
  exit 2
fi
if [[ $EUID != 0 ]]; then
  echo 'Root is required to configure NTP services; run the command with sudo.' >&2
  exit 1
fi
time_sync_role=$1
time_sync_host=$2
time_sync_jetson=$3
if [[ ! $time_sync_host =~ ^[0-9.]+$ || ! $time_sync_jetson =~ ^[0-9.]+$ ]]; then
  echo 'Supply IPv4 addresses.' >&2
  exit 2
fi
time_sync_backup=$(date -u +%Y%m%dT%H%M%SZ)
if [[ $time_sync_role == host ]]; then
  time_sync_config=/etc/chrony.conf
  [[ -f $time_sync_config ]] || time_sync_config=/etc/chrony/chrony.conf
  cp -a "$time_sync_config" "$time_sync_config.lr-backup-$time_sync_backup"
  if ! grep -Fxq "allow $time_sync_jetson/32" "$time_sync_config"; then
    echo "allow $time_sync_jetson/32" >> "$time_sync_config"
  fi
  if systemctl cat chronyd.service >/dev/null 2>&1; then
    systemctl restart chronyd
  else
    systemctl restart chrony
  fi
  if command -v firewall-cmd >/dev/null && firewall-cmd --state >/dev/null 2>&1; then
    time_sync_rule="rule family=ipv4 source address=$time_sync_jetson/32 port port=123 protocol=udp accept"
    firewall-cmd --add-rich-rule="$time_sync_rule"
    firewall-cmd --permanent --add-rich-rule="$time_sync_rule"
  fi
  chronyc tracking
else
  time_sync_config=/etc/systemd/timesyncd.conf.d/lr-host.conf
  mkdir -p /etc/systemd/timesyncd.conf.d
  if [[ -f $time_sync_config ]]; then
    cp -a "$time_sync_config" "$time_sync_config.lr-backup-$time_sync_backup"
  fi
  cat > "$time_sync_config" <<EOF
[Time]
NTP=$time_sync_host
FallbackNTP=
PollIntervalMinSec=16
PollIntervalMaxSec=64
EOF
  timedatectl set-ntp true
  systemctl restart systemd-timesyncd
  timedatectl timesync-status || true
fi
