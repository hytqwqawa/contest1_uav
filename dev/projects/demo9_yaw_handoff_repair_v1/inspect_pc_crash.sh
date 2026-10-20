#!/usr/bin/env bash
# Read-only crash triage. No services restarted, no param changes.
set -u
printf '%s\n' '==== BOOT WINDOWS ===='
journalctl --list-boots --no-pager | tail -n 8
printf '%s\n' '==== INCIDENT 21:07-21:10 SYSTEM LOG ===='
journalctl -b -2 --since '2026-10-19 21:07:00' \
  --until '2026-10-19 21:10:00' -o short-iso --no-pager | tail -n 160
printf '%s\n' '==== KERNEL, SAME WINDOW ===='
journalctl -k -b -2 --since '2026-10-19 21:07:00' \
  --until '2026-10-19 21:10:00' --no-pager | tail -n 90
printf '%s\n' '==== NODELET CRASH FILE METADATA (NO DUMP CONTENT) ===='
find /var/crash -maxdepth 1 -type f -iname '*nodelet*' -exec stat -c '%n %s bytes  modified=%y' {} \; 2>/dev/null
printf '%s\n' '==== DISK STATUS ===='
df -h "$HOME" /var 2>/dev/null
