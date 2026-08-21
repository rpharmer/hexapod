# Xbox Wireless Controller on WSL2 (for hexapod-server)

This is the path that works for the **Xbox Wireless Adapter for Windows** (`045e:02e6`) into Ubuntu WSL2, then into `hexapod-server` via **evdev**.

---

## What you are trying to achieve

```
Windows USB dongle (045e:02e6)
  → usbipd attach into WSL
  → xone kernel driver (+ embedded firmware)
  → paired Xbox pad
  → /dev/input/eventN
  → hexapod-server --controller-device /dev/input/eventN
```

The adapter is **not** a HID pad. It is a MediaTek radio. Stock `xpad`/`usbhid` will not give you `/dev/input/eventN` from the dongle alone. You need **xone** + **firmware**, and a WSL kernel that can load that firmware.

---

## 1. Custom WSL kernel

Stock WSL often lacks a usable gamepad/firmware path. Build from Microsoft’s WSL kernel source (example: `~/WSL2-Linux-Kernel-linux-msft-wsl-6.18.40.1`).

### Start from WSL config

```bash
cd ~/WSL2-Linux-Kernel-linux-msft-wsl-6.18.40.1
cp arch/x86/configs/config-wsl .config
make menuconfig
```

### Required options (`*` preferred over `M`)

**Input**

- `CONFIG_INPUT=y`
- `CONFIG_INPUT_EVDEV=y`
- `CONFIG_INPUT_JOYDEV=y` (optional)
- `CONFIG_INPUT_JOYSTICK=y`
- `CONFIG_JOYSTICK_XPAD=y` (useful for wired pads)

**HID / USB HID**

- `CONFIG_HID=y`
- `CONFIG_HIDRAW=y`
- `CONFIG_HID_GENERIC=y`
- `CONFIG_USB_HID=y`
- `CONFIG_HID_MICROSOFT=y` (recommended)

**USB / USBIP**

- `CONFIG_USB=y`
- `CONFIG_USBIP_CORE=y`
- `CONFIG_USBIP_VHCI_HCD=y`
- `CONFIG_USBIP_HOST=y` (optional)

**Firmware loader (critical for xone under WSL)**

- `CONFIG_FW_LOADER=y`
- `CONFIG_FW_LOADER_USER_HELPER=y`
- `CONFIG_FW_LOADER_USER_HELPER_FALLBACK=y`
- Embed the dongle firmware:
  - `CONFIG_EXTRA_FIRMWARE="xow_dongle.bin"`
  - `CONFIG_EXTRA_FIRMWARE_DIR="/usr/lib/firmware"`

Name must be exactly **`xow_dongle.bin`** (not `xone_dongle.bin` / `xone_dongle_02e6.bin`).

### Install firmware into Ubuntu *before* building the kernel

```bash
sudo apt install -y curl cabextract
# after xone is cloned/installed once:
sudo xone-get-firmware.sh
ls -l /usr/lib/firmware/xow_dongle.bin   # must exist (~70KB)
```

If `/lib/firmware` is missing on a fresh Ubuntu, create it:

```bash
sudo mkdir -p /lib/firmware /usr/lib/firmware
```

### Build and use the kernel

```bash
make -j$(nproc)
# install modules if anything is still modular:
sudo make modules_install
```

In Windows `%UserProfile%\.wslconfig`:

```ini
[wsl2]
kernel=C:\\path\\to\\your\\vmlinux
```

Then:

```powershell
wsl --shutdown
```

Reopen Ubuntu and confirm:

```bash
uname -r
```

---

## 2. Install xone (against the kernel you are running)

```bash
sudo apt install -y dkms git curl cabextract
git clone https://github.com/medusalix/xone ~/xone
cd ~/xone
sudo ./install.sh
sudo xone-get-firmware.sh
```

**After every custom-kernel rebuild**, reinstall xone modules for that kernel:

```bash
uname -r
cd ~/xone
sudo ./install.sh
# or DKMS force rebuild for current kernel
```

Otherwise you get:

```
xone_gip: disagrees about version of symbol module_layout
```

Blacklist the wrong Wi‑Fi driver that steals this VID/PID:

```bash
echo "blacklist mt76x2u" | sudo tee /etc/modprobe.d/blacklist-mt76x2u.conf
```

---

## 3. Attach the dongle with usbipd (Windows)

Install [usbipd-win](https://github.com/dorssel/usbipd-win), then:

```powershell
usbipd list
usbipd bind --busid 1-1
usbipd attach --wsl --busid 1-1
```

Look for `045e:02e6` **Xbox Wireless Adapter** → **Attached**.

In WSL, good `dmesg` should bind `xone-dongle` **without**:

- `xow_dongle.bin failed with error -2`
- `module_layout`
- `mt76x2u` claiming the device

---

## 4. Pair the controller

The Gen1 stick has a **small pair button on the dongle**. There is no useful sysfs pairing file until the driver is bound.

1. Press the **dongle** pair button (~30s pairing window).
2. Power on the pad, hold the **controller pair button** until it blinks.
3. Confirm:

```bash
ls -l /dev/input
cat /proc/bus/input/devices
```

You need `/dev/input/eventN` for the **pad**, not just the adapter.

Note: a pad previously paired only to Windows often must be **re-paired** for Linux/xone.

---

## 5. Run hexapod-server with the pad

```bash
cd ~/pico/hexapod/hexapod-server
./build-tests/hexapod-server --controller-device /dev/input/eventN
```

(`--xbox-device` is an alias.)

`EvdevGamepadController` reads Linux `input_event`s, maps Xbox-style codes to sticks/buttons, and `InteractiveInputMapper` turns that into `MotionIntent` (walk / body pose / calibration).

---

## Common failure checklist

| Symptom | Cause | Fix |
|--------|--------|-----|
| USBIP Attached, but `mt76x2u` in dmesg | Wrong driver | Blacklist `mt76x2u`, reattach |
| `xow_dongle.bin ... error -2` | Firmware not visible to WSL kernel | Embed via `CONFIG_EXTRA_FIRMWARE="xow_dongle.bin"` |
| `No rule to make target ... xone_dongle.bin` | Wrong EXTRA_FIRMWARE name | Use **`xow_dongle.bin`** |
| `disagrees about version of symbol module_layout` | xone built for old kernel | Reinstall xone after kernel change |
| Dongle up, no `/dev/input/eventN` | Pad not paired / not on | Pair dongle + pad again |
| Adapter “dead” / won’t pair | Probe never succeeded | Fix firmware/modules first |

---

## Minimal “fresh install” order

1. Clone/build custom WSL kernel with input/HID/USBIP + `EXTRA_FIRMWARE="xow_dongle.bin"`.
2. Install firmware file under `/usr/lib/firmware/xow_dongle.bin`, then build kernel so the blob is embedded.
3. Point `.wslconfig` at new `vmlinux`, `wsl --shutdown`.
4. Install/rebuild **xone** for `uname -r`; blacklist `mt76x2u`.
5. `usbipd attach --wsl` the `045e:02e6` adapter.
6. Pair pad; find `/dev/input/eventN`.
7. Launch server with `--controller-device /dev/input/eventN`.

---

## Optional easier path

A **wired** Xbox pad over USBIP usually works with mainline `xpad` and does **not** need xone/firmware. The wireless Windows adapter is what forced the xone + embedded-firmware path.

---

## Related code

- `hexapod-server/include/input/evdev_gamepad_controller.hpp`
- `hexapod-server/src/input/evdev_gamepad_controller.cpp`
- `hexapod-server/include/control/interactive_input_mapper.hpp`
- `hexapod-server/src/control/mode_runners.cpp`
- `docs/EXTENDING_IO_AND_HARDWARE.md`
