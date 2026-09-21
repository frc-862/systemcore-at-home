# Systemcore@Home on Raspberry Pi 5 + PICAN2 — Build Plan

**Target:** Raspberry Pi 5 (4 GB) running the Limelight Systemcore OS beta, with a
Copperhill/SK Pang **PICAN2** (single-channel MCP2515) providing a real CAN bus,
so WPILib 2027 robot code deploys and drives a real CAN motor controller.

**Source of truth:** Preston Childress' `Systemcore@Home.pdf` (FRC 401), plus the newer
and more complete [BobcatRobotics/SystemCore-Clone](https://github.com/BobcatRobotics/SystemCore-Clone)
guide and the [ChiefDelphi thread](https://www.chiefdelphi.com/t/diy-systemcore-for-early-learning/520408).
The PDF is a few versions behind; this plan uses current versions.

---

## 0. Why the PICAN2 works (and what changes vs. the deck)

The deck and the Bobcat guide both assume the **Waveshare 2-CH CAN HAT** (two MCP2515s on
SPI0 CE0 + CE1). The PICAN2 is the **same chip on the same bus, just one channel**:

| | PICAN2 | Waveshare 2-CH |
|---|---|---|
| Controller | MCP2515 + MCP2551 | MCP2515 ×2 |
| Crystal | 16 MHz | 16 MHz |
| SPI device | `spi0.0` (CE0) | `spi0.0` + `spi0.1` |
| INT pin | GPIO 25 | GPIO 25 + GPIO 24 |
| CAN FD | No (classic 1 Mbps) | No |

The DIY CAN bring-up script (`diy-can-setup.sh`) already handles a partially-populated board.
Its logic, verbatim from the repo:

```
setup_bus spi0.0 can_s0   # physical if present, else virtual
setup_bus spi0.1 can_s1
setup_bus ""      can_s2  # always virtual
setup_bus ""      can_s3
setup_bus ""      can_s4
```

The WPILib HAL **aborts unless all of `can_s0..can_s4` exist**, so the script fabricates the
missing ones as `vcan` dummies. With a PICAN2 you get:

- `can_s0` → **PHYSICAL**, 1 Mbps, your PICAN2 ✅
- `can_s1` … `can_s4` → virtual dummies (harmless; HAL is satisfied)

**No script modification is required.** The only file that needs PICAN2-specific content is
`config.txt` (one `dtoverlay` line instead of two).

### The one real gotcha

The stock Systemcore `config.txt` assigns **GPIO 25 to SPI1 CS0** and GPIO 24 / 22 / 26 / 17 /
9 / 27 to its own `sc-mcp2518` overlays. GPIO 25 is exactly the PICAN2 interrupt pin, and
enabling `dtparam=spi=on` claims GPIO 7–11, which collides with the stock SPI3 block.

→ **Replace the stock CAN overlay block, do not append to it.** (The PDF says "append";
that is what tripped up several people in the CD thread.) The config in §5 is the Bobcat
`config_no_fd.txt` with the CAN lines swapped for PICAN2's single line.

---

## 1. Hardware checklist

Already have:
- [ ] Raspberry Pi 5, 4 GB
- [ ] PICAN2 CAN board

Still needed (must-have):
- [ ] **32 GB microSD**, decent brand. Several people in the CD thread had boot failures on
      cheap cards — avoid Amazon Basics. SanDisk Extreme / Samsung EVO Select.
- [ ] **USB-C PSU, 5 V @ 5 A** (official Pi 5 27 W supply). The Pi will brown out USB
      peripherals on a 3 A supply.
- [ ] microSD reader on your Windows PC
- [ ] **120 Ω termination**: PICAN2 has an on-board terminator — verify the jumper/solder
      link is set, and put a second 120 Ω at the far end of the bus (most CTRE/REV devices
      do not self-terminate).
- [ ] A CAN device to test with (Kraken/TalonFX or SPARK MAX) + 12 V supply for it.

Nice to have:
- [ ] Active cooler for the Pi 5 — ⚠ check mechanical clearance under the PICAN2, the deck
      flags this. A low-profile heatsink or an external blower is the usual answer.
- [ ] Ethernet cable (more reliable than the Pi's Wi-Fi AP for deploys)
- [ ] For vision: powered **USB 3 hub** + USB webcam. On a Pi 5, all cameras must go through
      a hub plugged into the **upper USB 3 port** to match Systemcore's USB topology.

Not needed: CANivore (that's for people who want extra buses). Skip the `.ipk` driver step.

---

## 2. Downloads

Do these on the Windows PC before touching the Pi.

| Item | Version (as of 2026-09-17) | Link |
|---|---|---|
| Systemcore OS image | `limelightosr-2027.0.0-beta14-210`, asset **`limelightsystemcorebetacm5-limelightosr-2027.0.0-beta14.zip`** (745 MB) | [releases](https://github.com/LimelightVision/systemcore-os-public/releases) |
| Raspberry Pi Imager | latest | [raspberrypi.com/software](https://www.raspberrypi.com/software/) |
| WPILib 2027 | **`v2027.0.0-alpha-7`** — beta14 *requires* alpha-7; pick the build for your PC (below) | [allwpilib releases](https://github.com/wpilibsuite/allwpilib/releases) |
| FIRST Driver Station | `v2027.0.0-alpha-8` | [releases](https://github.com/wpilibsuite/FirstDriverStation-Public/releases) |
| CAN setup scripts | `no-fd/systemcore-can-nofd-setup/` | [SystemCore-Clone](https://github.com/BobcatRobotics/SystemCore-Clone) |
| AdvantageScope Lite (optional) | `v27.0.0-alpha-5` | [releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases) |
| Phoenix Tuner X *(if using CTRE)* | latest — desktop app, for device IDs + firmware | Microsoft Store, or [ctr-electronics.com](https://store.ctr-electronics.com/software/) |
| REV Hardware Client *(if using REV)* | latest — desktop app, for device IDs + firmware | [revrobotics.com](https://www.revrobotics.com/rev-software/) |

**Vendor libraries.** Versions that match WPILib alpha-7:

| Vendordep | Version for alpha-7 | How to get it |
|---|---|---|
| Phoenix 6 (CTRE) | `26.70.0-alpha-2` | **Download the JSON manually** - see below |
| REVLib | `2027.0.0-alpha-7` | VS Code, or JSON URL `https://software-metadata.revrobotics.com/REVLib-2027.json` |

> **Phoenix 6 alpha vendordeps are not in the VS Code online list.** Grab the JSON from the
> Phoenix 6 examples repo on the matching alpha branch and drop it in your project's
> `vendordeps/` folder **before building**:
>
> [`Phoenix6-26.70.0-alpha-2.json`](https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/2027_alpha7/java/TalonFXSGadgeteer/vendordeps/Phoenix6-26.70.0-alpha-2.json)
> (repo [`CrossTheRoadElec/Phoenix6-Examples`](https://github.com/CrossTheRoadElec/Phoenix6-Examples), branch `2027_alpha7`)
>
> ```bash
> curl -L -o vendordeps/Phoenix6-26.70.0-alpha-2.json https://raw.githubusercontent.com/CrossTheRoadElec/Phoenix6-Examples/2027_alpha7/java/TalonFXSGadgeteer/vendordeps/Phoenix6-26.70.0-alpha-2.json
> ```
>
> For a future alpha, switch the branch to `2027_alphaN` and search the repo for
> `vendordeps/*.json`. Delete any older `Phoenix6-*.json` (e.g. the `26.50.0-alpha-1` one
> bundled in the Bobcat example projects) first.

You also download the **desktop tuning app** for whichever brand you own - Phoenix Tuner X or
REV Hardware Client - used to set CAN IDs and flash device firmware. Skip both if you have no
CAN devices yet. Compatibility tables:
[CTR](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md) |
[REV](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/REV.md).

### Reading the Limelight releases page

Three separate version numbers collide in one release title. The newest release is named:

> **(REQUIRES WPILIB ALPHA 7)** Limelight SYSTEMCORE **BETA** Release limelightosr-2027.0.0-**beta14** (210)

- `ALPHA 7` = the **WPILib** version required -> `allwpilib v2027.0.0-alpha-7`
- `BETA` / `beta14` = the **image** version
- `(210)` = build number

So "Alpha 7" on that page is *not* an image version. The image is beta14.

**Two parallel tracks ship on the same day**, one per Systemcore hardware revision:

| Track | Newest tag | GitHub label | Flashable asset |
|---|---|---|---|
| **BETA** hardware | `limelightosr-2027.0.0-beta14-210` | **Latest** | `limelightsystemcorebetacm5-...beta14.zip` |
| ALPHA hardware | `limelightosr-2027.0.0-alpha14-380` | Pre-release | `limelightsystemcorecm5-...alpha14.zip` |

Both published 2026-09-02; both require WPILib alpha-7. Prior pairs: beta13/alpha13 (Aug 7),
beta-12/alpha-12 (Jul 16) - the deck's "Beta 12 image (176 or higher)" is that July pair.

**Use the BETA track.** It carries GitHub's "Latest" badge, it is what the ChiefDelphi DIY
thread has standardized on, and the Bobcat `config_no_fd.txt` this plan builds on is derived
from the beta image's `config.txt` (it still carries the `# BETA SYSTEMCORE: CAN2 on SPI3`
lineage). For a Pi 5 clone the practical difference is small - you replace the CAN overlay
block either way - but matching the community means matching their troubleshooting.

**Pick the WPILib installer for YOUR development machine, not for the Systemcore.** The
installer sets up VS Code, the JDK, Gradle and the cross-compiler on your workstation;
`gradle deploy` cross-compiles there and pushes the binary to the Pi over the network.
Nothing from this installer is ever installed on the Pi.

Go to the [allwpilib releases page](https://github.com/wpilibsuite/allwpilib/releases), open
the newest `2027.0.0-alpha-N` release, and scroll to the download links in the release notes.
Pick the row that matches your computer:

| Your computer | Installer to pick |
|---|---|
| **Windows PC** (Intel/AMD - most PCs) | **Windows (x64)** - `.iso` |
| **Mac, M1 / M2 / M3 / M4 / M5** (Apple Silicon) | **Mac (Arm)** - `.dmg` |
| **Mac, older** (Intel) | **Mac (Intel)** - `.dmg` |
| Windows on ARM (Snapdragon X / Surface Pro X) | Windows (arm64) - `.iso` |
| Linux PC (Intel/AMD) | Linux (x64) - `.tar.gz` |
| Linux on ARM (dev *machine* is ARM) | Linux (arm64) - `.tar.gz` |

Notes on the installer:
- **This box is Windows AMD64 -> take Windows (x64).**
- `Linux (arm64)` is a common wrong pick. The Systemcore *is* arm64 Linux, so that row looks
  right - but it is for people whose **dev machine** is arm64 Linux. It cannot run on Windows.
- Mac, not sure which: Apple menu -> About This Mac. "Apple M-something" = Arm64,
  "Intel Core..." = Intel.
- Windows ships as an **.iso** - mount it in Explorer (double-click) and run the installer
  inside. macOS ships as a `.dmg`, Linux as a `.tar.gz`.
- The download links live in the release notes **body**, not as GitHub release assets, so the
  release's asset list looks empty. Scroll down past the changelog.
- Match the alpha number to the image: beta14 needs alpha-7. If a newer alpha has shipped,
  check the Limelight release title before upgrading - it names the WPILib version it wants.

Other notes:
- Take the **`cm5` .zip**, not the `.llupdate` - the latter is an update package for real
  Systemcore hardware, not a flashable image.
- Ignore `systemcorebeta-aarch64-toolchain.tar.gz` (1.6 GB) on the Limelight release - that is
  for building the OS itself, not robot code.
- Version-lock matters: image beta14 <-> WPILib alpha-7. Mixing versions is the most common
  "deploy succeeds but DS shows no robot code" cause.
- Do **not** use the prebuilt Bobcat Google Drive images - they're built for the Waveshare
  HAT's pinout and are a redistribution of Limelight's image.

---

## 3. Flash and first boot

1. Unzip the CM5 image.
2. Raspberry Pi Imager → **Use custom** → the `.img` → target the microSD.
   Skip Imager's OS-customization prompt (it would write a Raspberry Pi OS `firstrun` that
   this image ignores).
3. Card into Pi 5, connect PICAN2 to the 40-pin header **now** (before first boot), power up.
   Give it ~2 minutes on first boot.
4. Connect one of two ways:
   - **Wi-Fi AP** the Pi broadcasts: SSID `SYSTEMCORE`, password `PASSWORD` (case-sensitive) →
     browse to `http://172.30.0.1` or `http://robot.local`
   - **Ethernet** to your LAN (defaults to DHCP) → `http://robot.local`
5. SSH login for later steps: `systemcore` / `systemcore`.

**Checkpoint A:** the Systemcore web UI loads and shows Home / System / Motioncore / Cameras /
Configure pages.

---

## 4. Establish a baseline before changing anything

Worth two minutes so you can tell a CAN problem from an image problem later:

```bash
uname -a
ls /dev/mrccan/
ip -br link show | grep can_s
systemctl is-active robot
```

Expect no `can_s*` interfaces yet, and `robot` likely failing or restarting.

---

## 5. Configure the PICAN2 in `config.txt`

Do this **from the Pi's own terminal** (web UI has a Terminal app, or SSH). Editing from
Windows is possible but the card shows three partitions and it's easy to edit the wrong
`config.txt` — the right one is the ~60-line file, not the empty one.

```bash
sudo mkdir -p /mnt/hardware_boot
sudo mount /dev/mmcblk0p2 /mnt/hardware_boot
sudo cp /mnt/hardware_boot/config.txt /mnt/hardware_boot/config.txt.orig   # keep a backup
sudo nano /mnt/hardware_boot/config.txt
```

**Replace the entire file** with this (Bobcat's no-FD config, with a single MCP2515 on CE0 /
GPIO 25 for the PICAN2):

```ini
[all]
start_file=start4.elf
fixup_file=fixup4.dat
over_voltage=2
usb_max_current_enable=1

boot_delay=1
kernel=Image

disable_overscan=1
hdmi_ignore_hotplug=1
hdmi_ignore_edid=0xa5000080
hdmi_blanking=2
ignore_lcd=1
dtoverlay=miniuart-bt
disable_splash=1

dtoverlay=pi3-disable-bt
dtoverlay=disable-bt
dtparam=audio=off
enable_uart=1
arm_64bit=1
force_eeprom_read=0

dtparam=ant2

dtoverlay=dwc2,dr_mode=peripheral
dtparam=pciex1_gen3

camera_auto_detect=0
display_auto_detect=0
dtparam=i2s=off
dtparam=i2c_csi_dsi0=on

# --- PICAN2: single MCP2515 on SPI0 CE0, 16 MHz crystal, INT on GPIO25 ---
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25,spimaxfrequency=10000000
```

> **On `usb_max_current_enable=1`:** at boot the Pi 5 caps *total* downstream USB current
> across all ports at **600 mA** unless it can confirm a 5 V/5 A supply via PD negotiation.
> This line forces the full **1.6 A** budget instead. Needed if you power from a plain
> 5 V/5 A buck converter (no PD controller) and want to run a USB hub + webcam for vision.
> Only set it if your supply can actually deliver; otherwise the Pi browns out and resets.

> **On the `dtoverlay=dwc2,dr_mode=peripheral` line:** this is inherited from the real
> Systemcore, where the USB-C port is a dedicated data port that presents USB gadget
> *ethernet* - plug in a cable and the host gets a virtual NIC at `172.26.0.1` (Windows,
> RNDIS function) or `172.27.0.1` (Mac/Linux, CDC-ECM function).
>
> **This will not work on a Pi 5 clone.** The Pi 5's USB-A ports hang off the RP1, which has
> no OTG; the SoC's OTG peripheral is exposed *only through the USB-C power connector*, so
> using it as a gadget requires powering the Pi from the GPIO header 5V/GND - which the
> PICAN2 occupies. It is also USB 2 only, and Pi 5 bootloader init gaps are reported.
> See [Raspberry Pi's OTG white paper](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009276-WP/Using-OTG-mode-on-Raspberry-Pi-SBCs).
>
> Leave the line in for parity with the real image - it is harmless - but connect over
> **Ethernet** (best) or the `SYSTEMCORE` Wi-Fi AP. There is no adb on Systemcore; it is
> embedded Linux, so remote access is SSH, SCP and the web UI.

Save (`Ctrl+O`, `Enter`, `Ctrl+X`), then:

```bash
sudo umount /mnt/hardware_boot
sudo reboot
```

**Checkpoint B:** after reboot,

```bash
dmesg | grep -i mcp251
ls -l /sys/class/net/*/device        # one entry should resolve to .../spi0.0
```

`dmesg` should show `mcp251x spi0.0 can0: MCP2515 successfully initialized`. If it says
`failed to acquire irq` or probes nothing, jump to §10.

---

## 5b. Disable the stock CAN service

Do this **before** §6, or Checkpoint C will show `can_s0` stuck DOWN and no CAN traffic.

The stock `limelight_canbusprocess.service` cannot work on a clone - it crash-loops and
re-downs your bus every 5 s. To see why, print the unit and read the commands it runs:

```bash
systemctl cat limelight_canbusprocess
```

Two of them fail on this hardware: `ethtool -G` / `-C` are unsupported by the `mcp251x`
(MCP2515) driver, and `ip link set ... type can bitrate` is invalid on the `vcan` dummies.
Either breaks its `&&` chain right after it has taken `can_s0` down.

```bash
sudo systemctl disable --now limelight_canbusprocess
```

Leave the other three alone - `limelight_canbusloadmon`, `limelight_canbussniffer` and
`limelight_canbuswatchdog` are read-only monitors and do not fight the interface.

---

## 5c. Optional: drop the vcan wait from `robot.service`

`robot.service` waits for all five CAN buses to report `state UP` before starting. `vcan`
interfaces always report `state UNKNOWN`, so the check never passes and start is delayed by
15 seconds - on every boot and every deploy. It is advisory only, so the robot still starts.

**Fix** - edit `/etc/systemd/system/robot.service` and narrow the loop to the bus you
actually have:

```bash
sudo nano /etc/systemd/system/robot.service
```

Change the bus list in `ExecStartPre` from:

```
for can in can_s0 can_s1 can_s2 can_s3 can_s4; do
```

to just the physical one:

```
for can in can_s0; do
```

Then:

```bash
sudo systemctl daemon-reload
sudo systemctl restart robot
```

Now the wait ends as soon as the PICAN2 is up instead of timing out.

> Alternatively, accept `UNKNOWN` as valid by changing `grep -q "state UP"` to
> `grep -qE "state (UP|UNKNOWN)"`, which keeps all five buses in the check. Either works;
> narrowing to `can_s0` is the smaller edit.

> An OS update will restore the stock unit and bring the 15 s delay back. A drop-in at
> `/etc/systemd/system/robot.service.d/10-vcan-ok.conf` (with an empty `ExecStartPre=` line
> first, to clear the inherited one) survives updates if you prefer.

---

## 6. Install the CAN bring-up service

Gets the interface renamed to `can_s0`, set to 1 Mbps, and creates the four virtual buses the
HAL demands. Also loads `robot_heartbeat` at the right moment (the stock
`modules-load.d` entry loads it too early, before any CAN interface exists — the installer
removes it).

From your Windows PC, with the repo cloned/downloaded:

```bash
scp -r no-fd/systemcore-can-nofd-setup systemcore@robot.local:/home/systemcore/
```

Then on the Pi:

```bash
cd ~/systemcore-can-nofd-setup
sudo chmod +x install.sh diy-can-setup.sh
sudo ./install.sh
sudo reboot
```

**Checkpoint C:**

```bash
ip -br link show | grep can_s
# can_s0  UP       <NOARP,UP,LOWER_UP,ECHO>      <- physical PICAN2
# can_s1..can_s4   UNKNOWN <NOARP,UP,LOWER_UP>   <- virtual, expected

ls /dev/mrccan/
# controldata  controldataro  enabledro  matchinfo  matchinforo

systemctl is-active can-bringup robot
# active
# active

journalctl -u can-bringup -b --no-pager
# "diy-can: can_s0 = PHYSICAL (can0 on spi0.0) @ 1Mbps"
```

`can_s0` showing `UP` with the `ECHO` flag is the physical one; the `UNKNOWN` ones are vcan.
Confirm live traffic with a device powered on the bus:

```bash
candump can_s0        # or:  ip -details -statistics link show can_s0
```

---

## 7. Configure and deploy code

1. Web UI → **Configure** → set your **team number**, set network properties. Reboot if asked.
2. Install WPILib 2027 alpha-7 on Windows; open its VS Code.
3. New WPILib project → set the same team number.
4. Deploy exactly as you would to a roboRIO.
5. Launch FIRST Driver Station alpha-8 → it should show robot code present and allow enable.

Starting point for CAN code: the repo's `project_examples/ctre/ctre-commands-v2` and
`-v3` (Phoenix 6 `26.50.0-alpha-1` vendordep). Address the physical bus by name — the
PICAN2 is **`can_s0`**.

**Checkpoint D:** Driver Station shows communications + robot code; you can enable teleop.

---

## 8. Spin a motor

1. Phoenix Tuner (or REV Hardware Client) → connect to `robot.local`.
2. Flash each CAN device to the alpha firmware. In Phoenix Tuner X, select the device ->
   Firmware, then **change the season selector from `2026` to `2027-alpha`** - the alpha
   builds only appear under that season.
   > ‼ **Tuner's built-in `2027-alpha` list was stale** and did not offer 26.70. Workaround:
   > download the firmware zip from the CTR Electronics site (search their downloads for
   > Phoenix 6 firmware), unzip it, and load the **Talon 26.70** `.crf` with Tuner's
   > file / manual firmware option instead of the dropdown.
   >
   > Check the dropdown first - if 26.70 is listed, just use it. Untested ideas for avoiding
   > the manual route: update Phoenix Tuner X to the newest version, or look for a
   > refresh/re-download control for its firmware database.
   > Repeat for **every** motor/device on the bus; there is no bulk update.
   >
   > Tuner X cannot push a temporary diagnostic server to Systemcore. Deploy a robot program
   > with a Phoenix 6 device constructed in it first, or Tuner sees no devices.
   >
   > The "incompatible firmware" warning at runtime is the check that this satisfies -
   > Phoenix 6 `26.70.0-alpha-2` will not talk to 2026 release firmware.
3. Set device ID, run a Tuner self-test, then drive it from your deployed code.

**Checkpoint E:** motor turns under deployed robot code, on `can_s0`.

---

## 9. Optional extras

**Vision.** Powered USB 3 hub into the Pi's **upper USB 3 port**, cameras into the hub
(max 4). Web UI → Cameras. Budget for CPU: one 1280×720 @ 30 FPS AprilTag pipeline costs
47 % / 43 % / 37 % / 25 % / 13 % CPU at detector downscale 1 / 1.5 / 2 / 3 / 4.
LimelightLib vendordep publishes to NetworkTables; `botpose` still needs manual conversion
to `Pose2d` for AdvantageScope Lite ([notes](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/LimelightVision.md)).

**Performance comparison.** Log loop time and compare against the deck's numbers:
0.2 ms median loop, ~100× faster than a roboRIO, 4 GB RAM. Idle memory on the image is
around 600 MB used, so a 4 GB Pi has plenty of headroom for Java.

**Packages.** Home screen → Add Package → drag the `.ipk` in
([video](https://vimeo.com/1095497571)). You only need this for CANivore drivers, which you
don't have — skip it.

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| No `can0` in `dmesg`, no `spi0.0` | SPI not enabled or overlay line typo'd. Confirm `dtparam=spi=on` precedes the `dtoverlay` line. Re-check you edited `mmcblk0p2`'s `config.txt`, not a sibling partition's. |
| `mcp251x: failed to acquire irq` | GPIO 25 still claimed by a leftover stock overlay — you appended instead of replacing. Redo §5 from the full config above. |
| MCP2515 probes but bus is flaky / error frames | Drop `spimaxfrequency` to `1000000` (SK Pang's documented PICAN2 value). Then check termination — exactly two 120 Ω, one of which is PICAN2's jumper. |
| `can_s0` shows VIRTUAL in the service log | Controller didn't probe in the 12 s window. Fix §5 first; the service is doing the right thing given no hardware. |
| `can_s0` comes UP then goes DOWN, `state STOPPED` | `limelight_canbusprocess` is crash-looping and re-downing the bus. See §5b. |
| `robot.service` won't stay active | Usually a missing `can_s*`. Re-run `sudo systemctl restart can-bringup && sudo systemctl restart robot` and read `journalctl -u robot -b`. |
| Deploy succeeds, DS shows no code | Image/WPILib version mismatch. beta14 ⇄ alpha-7. |
| `sudo ./install.sh` → permission denied | `sudo chmod +x install.sh` first (CD thread #170). |
| Pi won't boot at all | Try a different, better microSD card; update the Pi 5 bootloader firmware. |

---

## 11. Known limitations of Systemcore@Home

Set expectations before starting:

- **No SmartIO / RP2350.** No PWM, DIO, analog, encoders-on-board, or RSL. CAN-attached
  devices only. (Pin 13 high on a real RP2350 suppresses RSL faults — irrelevant here, but
  it means you'll see RSL-related complaints in logs.)
- **Classic CAN only.** The MCP2515 is not CAN FD. Real Systemcore buses are FD-capable.
  Fine for Kraken/TalonFX/SPARK at 1 Mbps; you cannot exercise FD-specific behavior.
- **One physical bus.** Multi-bus code paths can be written but only `can_s0` is real; writes
  to `can_s1..4` vanish into vcan.
- **Not competition legal.** Bench and code-development use only.
- **Alpha software.** Everything here moves weekly; re-check versions before starting.

---

## 12. Order of operations (summary)

```
[ ] 1. Buy: 32 GB SD, 5V/5A USB-C PSU, 120 Ω term, (hub + webcam)
[ ] 2. Download beta14 CM5 image + WPILib alpha-7 + DS alpha-8 + Bobcat repo
[ ] 3. Flash SD, mount PICAN2, boot                         → Checkpoint A
[ ] 4. Baseline `ls /dev/mrccan/`, `ip -br link show`
[ ] 5. Replace config.txt with the PICAN2 version, reboot   → Checkpoint B
[ ] 6. Disable limelight_canbusprocess (see §5b)
[ ] 7. scp + install systemcore-can-nofd-setup, reboot      → Checkpoint C
[ ] 7. Set team number, install WPILib, deploy, open DS     → Checkpoint D
[ ] 8. Flash motor to 2026 Phoenix firmware, spin it        → Checkpoint E
[ ] 9. Optional: vision, AdvantageScope Lite, perf logging
```

---

## References

- Systemcore@Home deck — Preston Childress, FRC 401 (`Systemcore@Home.pdf`, this folder)
- [ChiefDelphi: DIY Systemcore for early learning](https://www.chiefdelphi.com/t/diy-systemcore-for-early-learning/520408) (222 posts; #37 original scripts, #87 config.txt, #113 alternate tooling, #169–204 troubleshooting)
- [BobcatRobotics/SystemCore-Clone](https://github.com/BobcatRobotics/SystemCore-Clone) — no-FD guide, scripts, example projects
- [wpilibsuite/SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting) — vendor docs, CAN bus naming
- [bit-turtle/canlosetup](https://github.com/bit-turtle/canlosetup) — alternative image patcher
- [WPILib docs: New for 2027](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html#new-for-2027)
- [PICAN2 product page](https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/)
