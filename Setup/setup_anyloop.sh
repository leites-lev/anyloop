#!/usr/bin/env bash
# Ubuntu/Debian, Intel/AMD 64-bit. Run inside the copied anyloop folder:
#   bash setup_anyloop.sh
set -euo pipefail                         # Stop if a command fails.
[[ -f devices/parport_dac.c && -f meson.build ]] || { echo 'Run inside anyloop.'; exit 1; }
[[ $(uname -m) == x86_64 && $EUID != 0 ]] || { echo 'Use 64-bit Intel/AMD Linux, as your normal user.'; exit 1; }

# 1. Install the compiler, build tools, USB support, and calibration/report tools.
sudo apt-get update
sudo apt-get install -y build-essential meson ninja-build pkg-config curl unzip bzip2 \
  libjson-c-dev libgsl-dev libusb-1.0-0-dev pciutils poppler-utils \
  python3 python3-numpy python3-scipy python3-matplotlib

# 2. Download ZWO's official SDK bundle and unpack its Linux SDK.
temp=$(mktemp -d)
# Keep the downloads and extracted files; this script performs no cleanup deletion.
echo "SDK download and extracted files will remain in: $temp"
curl -fL --retry 3 'https://dl.zwoastro.com/software?app=DeveloperCameraSdk&platform=linux&region=Overseas' -o "$temp/zwo.zip"
unzip -q "$temp/zwo.zip" '*ASI_linux_mac_SDK*.tar.bz2' -d "$temp"
tar -xjf "$temp"/ASI_Camera_SDK/ASI_linux_mac_SDK*.tar.bz2 -C "$temp"
sdk=$(find "$temp" -name asi.rules -printf '%h\n')
[[ -f "$sdk/x64/libASICamera2.so" ]] || { echo 'Unexpected ZWO SDK layout.'; exit 1; }

# 3. Install the 64-bit camera library and ZWO's USB access rules.
sudo cp -a "$sdk"/x64/libASICamera2.so* /usr/local/lib/
sudo install -m 644 "$sdk/asi.rules" /etc/udev/rules.d/99-asi.rules
sudo ldconfig
sudo udevadm control --reload-rules
# Loading the library detects missing dependencies without commanding the DAC.
python3 -c 'import ctypes; ctypes.CDLL("libASICamera2.so"); print("ZWO library loads successfully.")'
echo 'Camera library installed. Reconnect the camera to apply the USB rules.'

# 4. Show the parallel card and ask for its address (the first column).
lspci -Dnn | grep -Ei 'parallel|AX99100|ASIX' || true
read -r -p 'Printer card address, e.g. 0000:06:00.2: ' DAC_PCI
[[ $DAC_PCI =~ ^[[:xdigit:]]{4}:[[:xdigit:]]{2}:[[:xdigit:]]{2}\.[0-7]$ && -d /sys/bus/pci/devices/$DAC_PCI ]] || { echo 'Invalid PCI address.'; exit 1; }
[[ $(cat "/sys/bus/pci/devices/$DAC_PCI/class") == 0x0701* && -e /sys/bus/pci/devices/$DAC_PCI/resource2 ]] || { echo 'Choose the parallel-port function with BAR2.'; exit 1; }
export DAC_PCI

# 5. Back up changed files, then replace the old computer's paths and PCI address.
python3 - <<'PY'
import json, os, re
from pathlib import Path
root = Path.cwd().resolve()
def save(path, text):
    if text == path.read_text(encoding='utf-8'): return
    backup = path.with_name(path.name + '.before-new-pc')
    if not backup.exists(): backup.write_bytes(path.read_bytes())
    path.write_text(text, encoding='utf-8')
p = root / 'meson.build'
text, n = re.subn(r'^asi_sdk_dir\s*=.*$', "asi_sdk_dir = '/usr/local/lib'", p.read_text(encoding='utf-8'), flags=re.M)
assert n == 1, 'Camera SDK setting not found in meson.build'
save(p, text)
for p in (root / 'contrib').rglob('*.json'):
    try: config = json.loads(p.read_text(encoding='utf-8'))
    except (ValueError, UnicodeError): continue
    if not isinstance(config, dict): continue
    changed = False
    for stage in config.get('pipeline', []):
        uri = stage.get('uri', '')
        if uri.startswith('file:') and uri.endswith('/asi_source.so'):
            stage['uri'] = 'file:' + str(root / 'build/asi_source.so')
            changed = True
        if uri == 'anyloop:parport_dac':
            stage.setdefault('params', {})['pci'] = os.environ['DAC_PCI']
            changed = True
    if changed: save(p, json.dumps(config, indent=2) + '\n')
PY

# 6. Preserve any old build, rebuild for this PC, and run the offline tests.
if [[ -e build ]]; then mv -- build "build-old-$(date +%Y%m%d-%H%M%S)-$$"; fi
meson setup build
meson compile -C build
test -f build/asi_source.so
meson test -C build --print-errorlogs

# 7. Show whether Linux permits the direct PCI access needed by the DAC.
if [[ -r /sys/kernel/security/lockdown ]]; then
  cat /sys/kernel/security/lockdown
  echo 'Direct PCI access requires [none]. Resolve lockdown before using the DAC.'
fi
echo 'Setup finished. Check DAC voltages and calibrate before steering.'
echo 'No outputs were driven. CPU tuning is unchanged; calibration assumes CPU 2.'
