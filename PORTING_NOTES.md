# Mirage → SystemCore (WPILib 2027 alpha7) porting notes

What changed when porting the 2026 Mirage robot code to WPILib 2027 alpha7, Commands v3 and
Telemetry. The port builds (`./gradlew build`) but has not run on a robot.

## Commands v3

| v2 | v3 |
|---|---|
| `SubsystemBase` | `implements Mechanism` (an interface) |
| `periodic()` / `simulationPeriodic()` | No hook. Call `Scheduler.getDefault().addPeriodic(this::periodic)` in the constructor |
| `CommandScheduler.getInstance().run()` | `Scheduler.getDefault().run()` |
| `new StartEndCommand(start, end, this)` | `run(co -> { start(); co.park(); }).whenCanceled(end).named("...")` |
| `run(...)` / `RunCommand` | `runRepeatedly(...)`, or a `while (true) { ...; co.yield(); }` loop |
| `runOnce` / `InstantCommand` | `run(co -> doIt())` (body runs once, then the command ends) |
| `FunctionalCommand` | init code, then a yielding loop, then `whenCanceled(end)` |
| `WaitCommand`, `WaitUntilCommand` | `co.wait(time)`, `co.waitUntil(cond)` |
| `.until(cond)` on StartEnd | `setX(); co.waitUntil(cond);` |
| Command groups | Straight-line coroutine code: `co.await(cmd)`, `co.fork(cmd)`, `co.awaitAll/awaitAny` |
| `.alongWith / .deadlineFor` | `Command.parallel(...)` / `Command.race(...)`, then `.named(...)` |
| No-requirement commands | `Command.noRequirements(co -> ...)` |
| Commands requiring several subsystems | `Command.requiring(a, b).executing(co -> ...)` |
| `InterruptionBehavior.kCancelIncoming` | `.withPriority(n)`: lower-priority commands can't interrupt |
| `ignoringDisable(true)` | Not needed: v3 doesn't cancel commands when the robot is disabled |
| `withTimeout(double)` | `withTimeout(Time)` |
| `button.Trigger` | `org.wpilib.command3.Trigger` (same `onTrue/whileTrue/toggleOnTrue`) |

Things to know:

- **`finally` does not run on cancel.** A canceled coroutine is never resumed, so
  `try { start(); co.park(); } finally { end(); }` never calls `end()` when a trigger releases.
  Use `whenCanceled(end)` instead. `CoroutineCancelTest` shows both cases.
- **Every command needs a name.** The builders make you call `.named(...)` (or
  `withAutomaticName()` on groups) before you get a `Command`.
- **Default commands may only require their own mechanism.** v2 let the shooter default to
  `cannon.shootOTF()`, which also required the turret and hood. v3 throws, so the shooter default is now a
  shooter-only command.
- **Child commands can't outlive their parent.** A command started with `fork`/`await` is
  canceled with its parent. v3 has no `ScheduleCommand`.
- **A class without hardware isn't a Mechanism.** `Cannon` is now a plain class, and its commands
  require the real mechanisms directly.
- **The scheduler is single-threaded.** Coroutines use JDK continuations and must run on the
  thread that calls `Scheduler.run()`.
- **SysId is v2 only** (`org.wpilib.command2.sysid`), so the swerve SysId routines were dropped.
- `Mechanism` provides `run`, `runRepeatedly`, `idle` and `setDefaultCommand`, so outside code can
  write `indexer.run(co -> ...)` (used for the unjam binding).

## Telemetry (replaces DataLog entries, NetworkTables publishers, SmartDashboard, Shuffleboard, SignalLogger)

- `Telemetry.log("Shooter/Bias", value)` or `Telemetry.getTable("Shooter").log("Bias", value)`.
- One `log(name, value)` covers primitives, arrays, `Measure`s (with units), struct types such as
  `Pose2d` and `ChassisVelocities`, and `TelemetryLoggable`s. `Mechanism2d`, `Field2d`,
  `PIDController` and `XboxController` are all `TelemetryLoggable` now.
- You don't declare or create entries up front; you just log values every loop.
- By default `RobotBase` sends telemetry to NetworkTables under `/Telemetry`.
  `DataLogManager.start()` records it to a file.
- Values the dashboard can edit are **Tunables** (a separate `tunables` library):
  `Tunables.addDouble("Cannon/Shooter Distance Bias", 8).get()` replaces
  `LightningShuffleboard.getDouble`. They publish under `/Tunables`.
- `SendableChooser`, `SmartDashboard` and `LiveWindow` are gone. `OpModeRobot` with
  `@Autonomous`/`@Teleop` classes is the new way to choose a mode. This port stays on `TimedRobot`
  and has one hard-coded auto.

## WPILib API changes

- Packages: `edu.wpi.first.*` → `org.wpilib.*`. Examples: `wpilibj.DriverStation` →
  `driverstation.*`, `wpilibj.Timer` → `system.Timer`, `math.MathUtil` → `math.util.MathUtil`,
  `math.Matrix/VecBuilder` → `math.linalg`, `Rectangle2d` → `math.shape`.
- `DriverStation` is split up. Robot mode is in `RobotState` (`isEnabled`, `isTeleop`,
  `isFMSAttached`). Match info is in `MatchState` (`getAlliance`, `getMatchTime`, `getGameData`).
  `Alliance` is a top-level enum with `RED`/`BLUE`.
- Test mode is now called **utility** mode (`utilityInit`).
- `ChassisSpeeds` → `ChassisVelocities` with fields `vx/vy/omega`. `SwerveModuleState` →
  `SwerveModuleVelocity` (`.velocity`).
- Mutable measures (`MutAngle`, `mut_plus`) are gone. Reassign immutable measures instead.
- `LinearSystemId` → `Models` (`flywheelFromPhysicalConstants`, `flywheelFromSysId`, ...).
  `createDCMotorSystem` has no direct replacement; this port uses the arm model instead.
- Constant naming changed from `kFoo` to `FOO` (`Rotation2d.ZERO`, `Color.WHITE`,
  `TimedRobot.DEFAULT_PERIOD`).
- `Translation2d.getAngle()` returns `Optional<Rotation2d>`. `pose.exp(twist)` →
  `pose.plus(twist.exp())`. `MathUtil.clamp` → Java's `Math.clamp`.
- Sim getters lost their unit suffixes: `getAngleRads()` → `getAngle()`,
  `getVelocityRadPerSec()` → `getVelocity()`.
- `XboxController`: Back/Start → `View`/`Menu`. `getLeftTriggerAxis()` → `getLeftTrigger()`.
  The d-pad is available as buttons (`getDpadUpButton()`). `RumbleType.kBothRumble` is gone.
- `AddressableLED` no longer has `start()`. `Timer.getFPGATimestamp()` → `Timer.getTimestamp()`.
- The project uses Java 25, and deploys to `/home/systemcore` instead of `/home/lvuser`.

## SystemCore / CTRE

- There is no CANivore. Use `new CANBus(CANPort.CAN_S0)`. Swerve constants use
  `withNetwork(bus)` instead of `withCANBusName`.
- CTRE's swerve state exposes `Velocity` and `ModuleVelocities`. `setOperatorPerspectiveForward`
  → `setOperatorForwardDirection`. `Utils.fpgaToCurrentTime` is gone.

## Dropped in this port

- PathPlanner, PhotonVision, maple-sim: none has a 2027 release. This removed autos, vision pose
  updates, the Mac Mini vision code and the maple-sim physics. Swerve sim now uses CTRE's
  `updateSimState`.
- Oasis (the second robot) constants, `LightningShuffleboard`, `FieldLayoutHelper`, `ShooterTest`.
- The on-the-fly lead compensation in the shooter's default command (see default command rule above).
