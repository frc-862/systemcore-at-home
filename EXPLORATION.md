# Systemcore System Exploration

Notes from exploring a live Systemcore@Home build: a Raspberry Pi 5 running the Limelight
Systemcore OS beta with a PICAN2 (MCP2515) on `can_s0`.

Explored 2026-09-21 against `systemcore@169.254.143.34`.

```
Kernel   6.12.77-v8-16k  #1 SMP PREEMPT_RT  Wed Sep 2 05:45:10 UTC 2026  aarch64
OS       LIMELIGHTOS_SYSTEMCORE_BETA
Host     Raspberry Pi 5 Model B Rev 1.0, 4 cores, 8 GB RAM, no swap
Root     /dev/mmcblk0p5
```

---

## 1. Kernel and real-time configuration

It is a genuine `PREEMPT_RT` build, not `PREEMPT_DYNAMIC` or `PREEMPT_RT` in name only.

```
/proc/sys/kernel/sched_rt_runtime_us  = 950000
/proc/sys/kernel/sched_rt_period_us   = 1000000
/proc/sys/kernel/sched_rr_timeslice_ms= 100
/proc/sys/kernel/threadirqs           = (empty)
```

RT throttling is at the default 95% — RT tasks may consume at most 950 ms of every 1 s,
leaving 50 ms for everything else. This is the safety valve that keeps a runaway RT thread
from locking the box.

`threadirqs` is empty because under `PREEMPT_RT` threaded IRQs are **forced on**; the boot
parameter is redundant. Every hardware interrupt runs in a schedulable kernel thread
(`irq/NNN-name`), which is what makes IRQ priority tuning possible at all.

Kernel command line (no RT-specific tuning):

```
reboot=w coherent_pool=1M 8250.nr_uarts=1 pci=pcie_bus_safe cgroup_disable=memory
numa_policy=interleave nvme.max_host_mem_size_mb=0 root=/dev/mmcblk0p5 rootwait
console=tty1 console=ttyAMA10,115200 ipv6.disable=1 net.ifnames=0
pcie_aspm.policy=performance
```

Notable for what is **absent**: no `isolcpus`, no `nohz_full`, no `rcu_nocbs`. All four
cores are shared between RT and normal work. Latency is managed purely by scheduling
priority, not by core partitioning. `pcie_aspm.policy=performance` disables PCIe link power
saving, which removes a wakeup-latency source on the RP1 I/O chip.

---

## 2. The real-time priority ladder

Measured live from `/proc/*/task/*/stat` (BusyBox `ps` cannot show scheduling class, so
these were read from the raw proc fields).

| Prio | Policy | Thread | What it is |
|-----:|--------|--------|------------|
| 99 | FIFO | `migration/0-3` | Kernel core-migration threads. Untouchable, kernel-owned. |
| **60** | FIFO | `irq/164-spi0.0` | **CAN controller interrupt** (PICAN2 MCP2515, GPIO 25) |
| **60** | FIFO | `irq/118-1f00050000.spi` | SPI host controller interrupt |
| 50 | FIFO | `irq/*` (all others) | Default threaded-IRQ priority: USB, MMC, I2C, UART, DMA, PCIe, codec |
| 50 | FIFO | `watchdogd` | Hardware watchdog feeder |
| **50** | **RR** | `java` | **Robot code main loop** |
| 40 | RR | `java` | Robot code secondary thread |
| **40** | FIFO | `MrcCommDaemon` | Driver Station comms + enable/heartbeat |
| 39 | FIFO | `iodaemon`, `libusb_event` | SmartIO / RP2350 USB link |
| 3, 2 | RR | `java` | Lower-priority robot threads |
| 0 | OTHER | everything else | Vision, web UI, Docker, logging |

### Reading the ladder

The design intent is legible from the numbers:

1. **CAN IRQ (60) outranks the robot code (50).** Frame reception must never be delayed by
   the robot loop, because a missed CAN frame is lost data — there is no retransmit buffer
   deep enough to survive a scheduling stall. Servicing the controller promptly matters more
   than running user code on time.
2. **Robot code (RR 50) outranks comms (FIFO 40).** The control loop takes precedence over
   Driver Station packet handling. A late DS packet is recoverable; a late motor command is
   a control-loop disturbance.
3. **Robot code is RR, not FIFO.** Round-robin means equal-priority threads timeshare via
   the 100 ms quantum rather than one starving the others. For user code — which may
   misbehave — that is the safer choice. The system's own daemons use FIFO.
4. **SmartIO (39) sits just below comms.** On real hardware this is the RP2350 link
   carrying PWM/DIO/analog. It is *below* CAN because the USB link has its own buffering.
5. **Vision is not real-time at all.** All four `visionserver` processes run SCHED_OTHER at
   nice 0. Frame processing is throughput work, not deadline work — a dropped frame costs
   nothing, and AprilTag latency is dominated by exposure and processing anyway.

### What must be real-time, ranked

**Hard requirement** — missing a deadline corrupts state or loses data:
- `irq/*-spi*` — CAN controller service. Priority 60.
- `MrcCommDaemon` — the enable/disable heartbeat. Motor controllers coast after ~100 ms of
  silence, so this thread starving means the robot stops.
- `robot_heartbeat` kernel module — the in-kernel periodic CAN frame. Lives in kernel
  context precisely so it cannot be preempted by userspace.

**Soft requirement** — missing a deadline degrades control quality:
- Robot code main loop (`TimedRobot`, 20 ms default period). RR 50.
- `iodaemon` — SmartIO I/O refresh. FIFO 39.

**Explicitly not real-time**, and correctly so:
- Vision (`visionserver`, `visionaggregator`), web UI (`apache`, `llttyd`), package manager,
  update manager, Docker/containerd, logging, mDNS, DHCP, `hailort` (NPU).

---

## 3. What assigns the priorities

Three different mechanisms, which is worth knowing when something lands at the wrong
priority.

**systemd unit properties** — only one service uses them:

```ini
# /etc/systemd/system/limelight_iodaemon.service
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=39
```

**`RLIMIT_RTPRIO` + self-promotion** — how the Java robot code gets RT:

```ini
# /etc/systemd/system/robot.service
LimitRTPRIO=50
```

This only raises the *ceiling*. The JVM runs as the unprivileged `systemcore` user and
cannot exceed 50. WPILib then calls `HAL_SetCurrentThreadPriority` (confirmed present in
`libwpiHal.so`) to promote individual threads via `pthread_setschedparam`. That is why the
`java` process shows a spread of RR 50 / 40 / 3 / 2 across its threads rather than one
uniform value — each WPILib subsystem picks its own.

**Self-managed by the daemon** — `MrcCommDaemon` has no scheduling directives in its unit
file, yet runs FIFO 40. It links `pthread_setschedparam`, `sched_setattr`,
`sched_get_priority_max` and `setpriority`, and runs as root, so it promotes itself.

**IRQ thread promotion** — `/usr/local/bin/hwmon/hwmon` is the only binary on the system
containing both `sched_setscheduler` and `/proc/irq/` strings, alongside literals like
`irq/120` and `irq/171`. It appears to be what raises the SPI/CAN interrupt threads from the
default 50 to 60. The hardcoded IRQ numbers are for real Systemcore hardware; on this Pi the
boosted threads are 118 and 164, so it is presumably matching by name rather than by number.
*Not fully confirmed — the binary is closed source.*

---

## 4. Service inventory

44 services running. Grouped by role:

### Robot control — the critical path
| Service | Binary | Notes |
|---|---|---|
| `robot` | `bash /home/systemcore/robotCommand` | Your Java code. `LimitRTPRIO=50`. |
| `mrccomm` | `/usr/bin/MrcCommDaemon` | DS comms, enable state, heartbeat. FIFO 40. |
| `limelight_iodaemon` | `iodaemon` | SmartIO/RP2350 over USB. FIFO 39. |
| `powerdistribution` | `PowerDistributionDaemon` | PDH/PDP telemetry |
| `expansionhubdaemon` | `ExpansionHubDaemon` | FTC Expansion Hub over USB |
| `limelight_motioncoredaemon` | `motioncoredaemon` | FTC Motioncore (`can_d0`–`can_d19`) |

### CAN bus management
| Service | Purpose |
|---|---|
| `limelight_canbusprocess` | Configures all five buses at boot. **Disabled on this build** — incompatible with MCP2515 and vcan. |
| `limelight_canbuswatchdog` | Monitors `can_s0`–`can_s4`, restarts on fault |
| `limelight_canbusloadmon` | Bus utilization for the web UI |
| `limelight_canbussniffer` | Frame capture for the CAN monitor page |
| `limelight_motioncoresniffer` | Same, for Motioncore buses |

### Vision — four independent pipelines
`limelight_visionserver`, `visionserver1`, `visionserver2`, `visionserver3` (one per camera,
~450–470 MB VSZ each), plus `limelight_visionaggregator` to merge results and
`limelight_networkresponder`. `hailort` drives the Hailo NPU. All SCHED_OTHER.

### Web UI and tooling
`apache` (the dashboard), `llttyd` (browser terminal on port 4901 — this is the web
Terminal app), `elastic_dashboard` (Python), `blocks` (FTC block-based programming backend),
`limelight_packagemanager` (`.ipk` installs), `limelight_updatemanager`,
`limelight_supportbundle`, `limelight_diagnosticsprocess`, `limelight_picoflasherprocess`
(RP2350 firmware), `limelight_hwmon`.

### Networking
`limelight_accesspoint` (hostapd — the `SYSTEMCORE` AP), `limelight_dnsmasq` +
`limelight_dnsmasqwifi`, `dhcpcd`, `avahi-daemon` + `avahi-dnsconfd` (mDNS `robot.local`),
`radiodaemon`, `sshd`.

### Surprising inclusions
**Docker and containerd are running.** ~1.4 GB VSZ each, idle. Not obviously used by
anything else on the system — presumably for user-installed packages. On a 4 GB Pi this is
worth knowing about; on this 8 GB unit it is harmless.

---

## 5. Build artifacts and deployment

### Toolchain

The project targets **Java 25** with the `org.wpilib.GradleRIO` plugin at `2027.0.0-alpha-7`:

```gradle
java {
    sourceCompatibility = JavaVersion.VERSION_25
    targetCompatibility = JavaVersion.VERSION_25
}
def ROBOT_MAIN_CLASS = "first.Main"
```

The deploy target is `getTargetTypeClass('SystemCore')` with `useDefaultSystemcoreHostName()`,
which resolves the host from the team number in `.wpilib/wpilib_preferences.json`
(`teamNumber: 862`, `projectYear: 2027_alpha7`).

### What the build produces

| Artifact | Local path | Contents |
|---|---|---|
| Robot jar | `build/libs/<project>.jar` | ~16 KB. Your compiled classes only. Also embeds `src/`, `vendordeps/` and `build.gradle` under `backup/` inside the jar. |
| Desktop JNI | `build/jni/release/*.dll` | Windows natives for **simulation only** — never deployed. |
| Systemcore natives | pulled from Maven, not built | `*-linuxsystemcore.zip` artifacts from `C:\Users\Public\wpilib\2027_alpha7\maven\` and `~/.gradle/caches/` |

The `linuxsystemcore` classifier is the key: WPILib and vendors publish prebuilt `aarch64`
shared objects. Nothing is cross-compiled locally — Gradle downloads `.zip` artifacts and
unpacks the `.so` files during deploy.

### What lands on the Pi

`gradle deploy` pushes four groups over SSH into `/home/systemcore/`:

```
/home/systemcore/
├── robotCommand                 # 49 bytes: /usr/bin/java @/home/systemcore/robotCommand.args
├── robotCommand.args            # JVM flags + full classpath + main class
├── wpilib/
│   ├── classpath/               # 34 jars, ~5.9 MB
│   │   ├── wpilibj-java-2027.0.0-alpha-7.jar
│   │   ├── wpimath, ntcore, cscore, hal, apriltag, telemetry, tunables, ...
│   │   ├── wpiapi-java-26.70.0-alpha-2.jar      # Phoenix 6 vendordep
│   │   ├── ejml-*, avaje-*, opencv, quickbuf     # third-party deps
│   │   └── <project>.jar                         # YOUR code, named after the gradle project
│   └── third-party/lib/         # 37 aarch64 .so files
│       ├── libwpiHal.so, libwpiHaljni.so, libntcore.so, libwpimath.so, ...
│       ├── libCTRE_Phoenix6.so, libCTRE_PhoenixTools.so
│       └── libopencv_*.so.413, libapriltag.so
├── deploy/                      # FileTreeArtifact: src/main/deploy/ verbatim
└── logs/<timestamp>/            # DataLog output, created at runtime
```

Note the project jar is named after the **Gradle project name**, not a fixed filename — this
unit currently holds `systemcore-test.jar` from an earlier project while the local checkout
builds `systemcore-at-home.jar`. Stale jars from renamed projects are not cleaned up
automatically and will sit in `classpath/` indefinitely.

### How the program is launched

`robotCommand` is a one-line shell script invoking the JVM with an `@argfile`:

```
/usr/bin/java @/home/systemcore/robotCommand.args
```

`robotCommand.args`:

```
-XX:+UseZGC
-Djava.library.path=/home/systemcore/wpilib/third-party/lib
--add-opens java.base/jdk.internal.vm=ALL-UNNAMED
--add-opens java.base/java.lang=ALL-UNNAMED
--enable-native-access=ALL-UNNAMED
-cp "...34 jars, colon-separated..."
first.Main
```

Three details worth noting:

- **`-XX:+UseZGC`.** The Z Garbage Collector, chosen for sub-millisecond pause times
  regardless of heap size. This is the single most important JVM setting for a robot control
  loop — a stop-the-world pause is indistinguishable from a missed deadline. The `Z*` threads
  (`ZDriverMajor`, `ZWorkerYoung`, `ZStat`) are visible in the thread list.
- **`--add-opens java.base/jdk.internal.vm`.** WPILib reaches into JVM internals, almost
  certainly for thread priority / scheduling control via `HAL_SetCurrentThreadPriority`.
- **`--enable-native-access=ALL-UNNAMED`.** Suppresses the Java 24+ restricted-native-access
  warnings from the Panama FFI calls into the HAL.

`robot.service` supervises it with `Restart=always`, `RestartSec=3`, `TimeoutStopSec=1`.
Redeploying restarts this unit; the JVM is killed and relaunched from the same argfile.

### The CAN precondition

`robot.service` has an `ExecStartPre` that waits up to 15 s for all five buses:

```bash
for can in can_s0 can_s1 can_s2 can_s3 can_s4; do
  if ! ip link show $can up 2>/dev/null | grep -q "state UP"; then good=false; fi
done
```

**On a DIY build this check always fails.** It greps for `state UP`, but `vcan` interfaces
report `state UNKNOWN`:

```
can_s0  UP       <NOARP,UP,LOWER_UP,ECHO>     <- physical, passes
can_s1  UNKNOWN  <NOARP,UP,LOWER_UP>          <- vcan, reported "down"
```

Observed in the journal:

```
Sep 19 17:01:57 robot bash[809]: can_s1 down
...
Sep 19 17:01:58 robot bash[809]: could not find can bus interfaces
Sep 19 17:01:58 robot systemd[1]: Started robot.
```

It is **advisory only** — the script's final statement is an `echo`, so it exits 0 and the
robot starts anyway. The cost is a fixed **15-second delay on every boot and every deploy**.
Harmless but annoying; a candidate for patching if boot time matters.

Startup then proceeds normally:

```
NT: starting network client
NT: CONNECTED NT4 to 127.0.0.1 port 6810
********** Robot program starting **********
NT: Listening on port 5810
NT: mDNS announcing as service 'robot' on port 5810
********** Robot program startup complete **********
```

NetworkTables connects to `127.0.0.1:6810` (the local vision/dashboard bus) and listens on
5810 for external clients.

---

## 6. Findings relevant to the DIY build

**CAN interface naming is udev-driven on real hardware.** `/etc/udev/rules.d/70-can-interface-names.rules`
maps SPI devices to bus names:

```
KERNELS=="spi2.0", NAME="can_s0"     # CAN0, CAN1 on SPI2
KERNELS=="spi2.1", NAME="can_s1"
KERNELS=="spi3.0", NAME="can_s2"     # CAN2 on SPI3 (beta only)
KERNELS=="spi1.0", NAME="can_s3"     # CAN3, CAN4 on SPI1
KERNELS=="spi1.1", NAME="can_s4"
```

The PICAN2 is on `spi0.0`, which matches no rule — which is exactly why `diy-can-setup.sh`
renames it manually with `ip link set ... name can_s0`. An alternative cleaner approach
would be adding a `spi0.0` line to this rules file, letting udev do the naming at probe time
and removing one race from the bring-up path. **Untested.**

**The IRQ priority boost works on the clone.** `irq/164-spi0.0` came out at FIFO 60, the same
tier the real hardware gives its CAN controllers. Whatever matching logic `hwmon` uses, it
caught the PICAN2's interrupt. This is a genuinely good sign for latency on this build.

**The interrupt is on one core.** `/proc/interrupts` shows all 511,424 `spi0.0` interrupts
landed on CPU2. Default affinity, no explicit pinning.

**Memory is not a constraint here.** 343 MB used of 8 GB with the robot program running.
Even accounting for Docker and four vision servers, a 4 GB unit has ample headroom — the
deck's "8x more RAM than the roboRIO" claim holds.

---

## Appendix: reconnecting

```bash
ssh systemcore@169.254.143.34          # password: systemcore
```

The link-local `169.254.x.x` address indicates direct Ethernet with no DHCP server.
`robot.local` via mDNS also works when avahi is reachable.

Useful one-liners:

```bash
# Real-time thread inventory (BusyBox ps can't show scheduling class)
for t in /proc/[0-9]*/task/[0-9]*/stat; do cat "$t"; done | awk '{
  comm=$0; sub(/^[0-9]+ \(/,"",comm); sub(/\).*/,"",comm);
  rest=$0; sub(/^[^)]*\) /,"",rest); split(rest,f," ");
  if (f[38]+0>0) printf "%-4s %-5s %s\n", f[38], (f[39]==1?"FIFO":"RR"), comm
}' | sort -rn

ip -br link show | grep can_s        # bus state
systemctl list-units --type=service --state=running --no-legend
journalctl -u robot.service -f       # live robot program output
```
