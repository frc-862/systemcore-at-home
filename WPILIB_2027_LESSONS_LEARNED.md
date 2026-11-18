# +WPILib 2027 lessons learned

Lessons from porting the 2026 Mirage robot to WPILib 2027 alpha7, Commands v3, Telemetry and
SystemCore. Each item shows the old pattern (2026) and the new pattern (2027).

---

## Commands v3

### Subsystems are now Mechanisms

```java
// 2026
public class Shooter extends SubsystemBase { }

// 2027
public class Shooter implements Mechanism { }
```

### There is no periodic() hook

```java
// 2026
@Override
public void periodic() { log(); }

// 2027: register it in the constructor
Scheduler.getDefault().addPeriodic(this::periodic);
```

### Simulation periodic is registered too

```java
// 2027
if (RobotBase.isSimulation()) {
    Scheduler.getDefault().addPeriodic(this::simulationPeriodic);
}
```

### Running the scheduler

```java
// 2026
CommandScheduler.getInstance().run();

// 2027
Scheduler.getDefault().run();
```

### Scheduling and canceling

```java
// 2026
CommandScheduler.getInstance().schedule(auto);
auto.cancel();

// 2027
Scheduler.getDefault().schedule(auto);
Scheduler.getDefault().cancel(auto);
```

### Every command needs a name

```java
// 2027: builders only become a Command after .named()
run(co -> stowPivot()).named("Collector Stow");
```

### Instant command (runOnce)

```java
// 2026
runOnce(() -> stowPivot());

// 2027: the body runs once, then the command ends
run(co -> stowPivot()).named("Stow");
```

### Instant command without requirements

```java
// 2026
new InstantCommand(() -> changeBias(bias));

// 2027
Command.noRequirements(co -> changeBias(bias)).named("Change Bias");
```

### Repeating command (run / RunCommand)

```java
// 2026
run(() -> setVelocity(supplier.get()));

// 2027
runRepeatedly(() -> setVelocity(supplier.get())).named("Shoot");
```

### Repeating command written as an explicit loop

```java
// 2027: every loop must yield
run(co -> {
    while (true) {
        setPower(power.getAsDouble());
        co.yield();
    }
}).named("Collect");
```

### Start/end command

```java
// 2026
startEnd(() -> setPower(p), this::stop);

// 2027
run(co -> {
    setPower(p);
    co.park();              // wait here until canceled
}).whenCanceled(() -> stop()).named("Index");
```

### finallyDo

```java
// 2026
new RunCommand(() -> stowPivot()).finallyDo(() -> deployPivot());

// 2027
Command.noRequirements(co -> {
    while (true) { stowPivot(); co.yield(); }
}).whenCanceled(() -> deployPivot()).named("Driver Stow");
```

### FunctionalCommand (init / execute / end)

```java
// 2026
new FunctionalCommand(this::deploy, this::spin, i -> stop(), () -> false, this);

// 2027
run(co -> {
    deploy();                                  // init
    while (true) { spin(); co.yield(); }       // execute
}).whenCanceled(() -> stop()).named("Collect");  // end
```

### Waiting for a condition

```java
// 2026
new WaitUntilCommand(this::isOnTarget);

// 2027: inside a coroutine
co.waitUntil(this::isOnTarget);
```

### Waiting for a time

```java
// 2026
new WaitCommand(0.25);

// 2027
co.wait(Seconds.of(0.25));
```

### Ending a command when a condition is met

```java
// 2026
startEnd(() -> setVelocity(v), () -> {}).until(this::isOnTarget);

// 2027
run(co -> {
    setVelocity(v);
    co.waitUntil(this::isOnTarget);
}).named("Spin Up");
```

### Sequences become straight-line code

```java
// 2026
new InstantCommand(() -> setTransfer(t))
    .andThen(new WaitCommand(0.25))
    .andThen(startEnd(() -> setPower(s, t), this::stop));

// 2027
run(co -> {
    setTransfer(t);
    co.wait(Seconds.of(0.25));
    setPower(s, t);
    co.park();
}).whenCanceled(() -> stop()).named("Auto Index");
```

### Timeout followed by another step

```java
// 2026
index(-0.5).withTimeout(0.1).andThen(index(1));

// 2027
indexer.run(co -> {
    indexer.setPower(-0.5);
    co.wait(Seconds.of(0.1));
    indexer.setPower(1);
    co.park();
}).whenCanceled(() -> indexer.stop()).named("Unjam");
```

### withTimeout takes a unit

```java
// 2026
cmd.withTimeout(2);

// 2027
cmd.withTimeout(Seconds.of(2));
```

### Running a child command and waiting for it

```java
// 2027
co.await(indexer.autoIndex(power, transfer));
```

### Running a child command in the background

```java
// 2027: fork starts it without waiting
co.fork(shooter.runShootCommand(speed));
co.waitUntil(this::isReady);
```

### Child commands can't outlive their parent

A child started with `fork` or `await` is canceled with its parent. v3 has no `ScheduleCommand`.

### Parallel groups

```java
// 2026
a.alongWith(b);

// 2027
Command.parallel(a, b).named("Both");
```

### Deadline and race groups

```java
// 2026
shot.deadlineFor(rumble());

// 2027
Command.race(shot, rumble()).named("Shot");
```

### Commands requiring several mechanisms

```java
// 2026
new RunCommand(() -> aim(), turret, shooter, hood);

// 2027
Command.requiring(turret, shooter, hood).executing(co -> {
    while (true) { aim(); co.yield(); }
}).named("Shoot OTF");
```

### Commands without requirements

```java
// 2026
new StartEndCommand(() -> retracted = true, () -> retracted = false);  // no subsystem

// 2027
Command.noRequirements(co -> { retracted = true; co.park(); })
    .whenCanceled(() -> retracted = false).named("Retract");
```

### Interruption behavior becomes priority

```java
// 2026
cmd.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

// 2027: lower-priority commands can't interrupt this one
run(...).withPriority(Command.DEFAULT_PRIORITY + 1).named("Collect");
```

### ⚠️ Default commands may only require their own mechanism

```java
// 2026: allowed. shootOTF also requires the turret and hood
shooter.setDefaultCommand(cannon.shootOTF());

// 2027: throws IllegalArgumentException, so use a shooter-only command
shooter.setDefaultCommand(shooter.runShootCommand(speed));
```

### ignoringDisable is gone

v3 doesn't cancel commands when the robot is disabled, so `.ignoringDisable(true)` is simply
dropped.

### A class without hardware isn't a Mechanism

A coordinator like `Cannon` becomes a plain class. Its commands require the real mechanisms.

### Mechanisms expose command factories publicly

```java
// 2027: outside code can build a command on a mechanism
indexer.run(co -> { ... }).named("Unjam");
```

### Triggers

```java
// 2026
import edu.wpi.first.wpilibj2.command.button.Trigger;

// 2027: same onTrue / whileTrue / toggleOnTrue API
import org.wpilib.command3.Trigger;
```

### Cleanup replaces onFalse(stop)

```java
// 2026
trigger.whileTrue(autoIndex()).onFalse(new InstantCommand(indexer::stop));

// 2027: autoIndex already has whenCanceled(stop)
trigger.whileTrue(autoIndex());
```

### Single-threaded scheduler

Coroutines use JDK continuations. Only schedule or cancel commands from the thread that calls
`Scheduler.run()`.

### SysId is v2 only

`SysIdRoutine` exists only in `org.wpilib.command2.sysid`.

### Vendordep

```text
2026: WPILibNewCommands.json
2027: CommandsV3.json   (conflicts with CommandsV2.json)
```

---

## +Telemetry

Telemetry has been simplified. One call to send values to the dashboard and to a file.

- NetworkTables under `/Telemetry` by default.
- `DataLogManager.start()` records it to a file.

### +Logging a value

```java
// 2026
DoubleLogEntry entry = new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Bias");
entry.append(bias);

// 2027: no entries to declare up front
Telemetry.log("Shooter/Bias", bias);
```

### +Using a table per mechanism

```java
// 2027
private final TelemetryTable telemetry = Telemetry.getTable("Shooter");
telemetry.log("Bias", bias);
```

### Replacing SmartDashboard

```java
// 2026
SmartDashboard.putNumber("Hood/Angle", angle.in(Degrees));

// 2027
Telemetry.log("Hood/Angle", angle);
```

### +Logging measures (units are kept)

```java
// 2026
log.append(velocity.in(RotationsPerSecond));

// 2027
telemetry.log("Velocity", velocity);
```

### +Logging struct types

```java
// 2026
StructPublisher<Pose2d> pub = table.getStructTopic("Pose", Pose2d.struct).publish();
pub.set(pose);

// 2027
telemetry.log("Pose", pose);
```

### Logging struct arrays

```java
// 2027
telemetry.log("ModuleVelocities", state.ModuleVelocities, SwerveModuleVelocity.struct);
```

### Logging a Mechanism2d or Field2d

```java
// 2026
SmartDashboard.putData("Hood/Mech2d", mech2d);

// 2027: they are TelemetryLoggable; log them every loop
telemetry.log("Mech2d", mech2d);
```

### +Other TelemetryLoggable types

`Field2d`, `Mechanism2d`, `PIDController` and `XboxController` implement `TelemetryLoggable`.


### +Dashboard inputs are Tunables

```java
// 2026
double bias = LightningShuffleboard.getDouble("Cannon", "Bias", 8);

// 2027
TunableDouble bias = Tunables.addDouble("Cannon/Bias", 8);   // create once
bias.get();                                                  // read anywhere
```

Tunables are published under `/Tunables`. The robot loop updates them automatically, so `get()`
always returns the latest dashboard value.

### +Other tunable types

```java
// 2027
TunableInt id = Tunables.addInt("Cannon/Id", 1);
TunableBoolean enabled = Tunables.addBoolean("Cannon/Enabled", false);

// Expose an existing field without a Tunable object
Tunables.publishDouble("Cannon/Bias", () -> bias, v -> bias = v);
```

### +Running a command from the dashboard

This is useful for zeroing a mechanism or running a calibration routine from Elastic without
binding a gamepad button. In Elastic, open `/Tunables` and drag the entry onto a tab. Clicking
it starts the command, and clicking it again cancels it.

```java
// 2026
SmartDashboard.putData("Hood/Zero", hood.zero());

// 2027, Commands v2: a Command is a ComplexTunable, so publish it directly
Tunables.publish("Hood/Zero", hood.zero().ignoringDisable(true));
```

### Gone

`SmartDashboard`, `Shuffleboard`, `SendableChooser`, `Sendable` and `LiveWindow`.

### Choosing an auto without SendableChooser

Use `OpModeRobot` with `@Autonomous` classes. This port stays on `TimedRobot` and has one
hard-coded auto.

---

## Packages

| 2026 | 2027 |
|---|---|
| `edu.wpi.first.*` | `org.wpilib.*` |
| `wpilibj.TimedRobot`, `RobotBase` | `framework.*` |
| `wpilibj.DriverStation`, `XboxController`, `GenericHID` | `driverstation.*` |
| `wpilibj.Timer`, `Notifier`, `RobotController`, `DataLogManager` | `system.*` |
| `wpilibj.DigitalInput` | `hardware.discrete.DigitalInput` |
| `wpilibj.AddressableLED` | `hardware.led.AddressableLED` |
| `wpilibj.PowerDistribution` | `hardware.power.PowerDistribution` |
| `wpilibj.simulation.*` | `simulation.*` |
| `wpilibj.smartdashboard.Mechanism2d` | `smartdashboard.Mechanism2d` |
| `wpilibj.util.Color` | `util.Color` |
| `math.MathUtil` | `math.util.MathUtil` |
| `math.Matrix`, `VecBuilder`, `Vector` | `math.linalg.*` |
| `math.geometry.Rectangle2d` | `math.shape.Rectangle2d` |
| `math.system.plant.DCMotor` | `math.system.DCMotor` |
| `util.datalog.*` | `datalog.*` |
| `units.*` | `units.*` (under `org.wpilib`) |
| `wpilibj2.command.*` | `command3.*` |

---

## +Driver station

Getting info about the match or robot state.

### +Robot mode

```java
// 2026
DriverStation.isEnabled();
DriverStation.isTeleop();

// 2027
RobotState.isEnabled();
RobotState.isTeleop();
```

### +Alliance

```java
// 2026
DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

// 2027
MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE;
```

### +Match time and game data

```java
// 2026
DriverStation.getMatchTime();
DriverStation.getGameSpecificMessage();

// 2027
MatchState.getMatchTime();
MatchState.getGameData().orElse("");
```

### +Test mode is now utility mode

```java
// 2026
public void testInit() { }

// 2027
public void utilityInit() { }
```

---

## +Controllers

### +Button names

```java
// 2026
driver.getStartButton();
driver.getBackButton();

// 2027
driver.getMenuButton();
driver.getViewButton();
```

### +Trigger axes

```java
// 2026
driver.getLeftTriggerAxis();

// 2027
driver.getLeftTrigger();
```

### +D-pad

```java
// 2026
driver.getPOV() == 0;

// 2027
driver.getDpadUpButton();
```

### Rumble

```java
// 2026
hid.setRumble(RumbleType.kBothRumble, 1);

// 2027
hid.setRumble(RumbleType.LEFT_RUMBLE, 1);
hid.setRumble(RumbleType.RIGHT_RUMBLE, 1);
```

---

## +Math and geometry

### +Constants use UPPER_CASE

```java
// 2026
Rotation2d.kZero;   Color.kWhite;   TimedRobot.kDefaultPeriod;

// 2027
Rotation2d.ZERO;    Color.WHITE;    TimedRobot.DEFAULT_PERIOD;
```

### +Field coordinates are changing

Not in alpha7 yet. This was merged in
[allwpilib#9590](https://github.com/wpilibsuite/allwpilib/pull/9590) for a later 2027 release.

- The origin moves to the center of the field instead of the blue alliance corner.
- +X points toward the blue alliance wall, so field-relative poses are rotated 180° compared to 2026.
- AprilTag layouts, including past years, are converted to the new system.
- It's a silent break: NetworkTables and DataLog poses don't say which system they use, so
  dashboards and any hard-coded field poses must be updated.

### +Translation2d angle is optional

```java
// 2026
delta.getAngle();

// 2027: empty for a zero vector
delta.getAngle().orElse(Rotation2d.ZERO);
```

### Pose exponential

```java
// 2026
pose.exp(twist);

// 2027
pose.plus(twist.exp());
```

### Clamp

```java
// 2026
MathUtil.clamp(x, -1, 1);

// 2027: use Java's clamp
Math.clamp(x, -1, 1);
```

---

## +Units

### +Mutable measures are gone

```java
// 2026
MutAngle bias = Degrees.mutable(0);
bias.mut_plus(delta);
bias.mut_replace(Degrees.zero());

// 2027
Angle bias = Degrees.zero();
bias = bias.plus(delta);
bias = Degrees.zero();
```

---

## +Simulation

### +LinearSystemId is Models

```java
// 2026
LinearSystemId.createFlywheelSystem(motor, J, gearing);
LinearSystemId.identifyVelocitySystem(kV, kA);

// 2027
Models.flywheelFromPhysicalConstants(motor, J, gearing);
Models.flywheelFromSysId(kV, kA);
```

### DC motor system

```java
// 2026
LinearSystemId.createDCMotorSystem(motor, J, gearing);

// 2027: no direct replacement; a gravity-free arm model is the same system
Models.singleJointedArmFromPhysicalConstants(motor, J, gearing);
```

### +Sim getters lost unit suffixes

```java
// 2026
armSim.getAngleRads();   armSim.getVelocityRadPerSec();
motorSim.getAngularPositionRad();

// 2027
armSim.getAngle();       armSim.getVelocity();
motorSim.getAngularPosition();
```

---

## Hardware

### AddressableLED starts automatically

```java
// 2026
leds.setLength(n);
leds.start();

// 2027
leds.setLength(n);
```

### Timestamps

```java
// 2026
Timer.getFPGATimestamp();

// 2027
Timer.getTimestamp();
```

---

## +SystemCore and CTRE Phoenix 6

### +CAN buses are ports, not names

```java
// 2026
new CANBus("Canivore");

// 2027
new CANBus(CANPort.CAN_S0);
```

### +Swerve CAN bus

```java
// 2026
new SwerveDrivetrainConstants().withCANBusName(bus.getName());

// 2027
new SwerveDrivetrainConstants().withNetwork(bus);
```

### +Swerve state fields

```java
// 2026
state.Speeds;   state.ModuleStates;

// 2027
state.Velocity; state.ModuleVelocities;
```

### +ChassisSpeeds is now ChassisVelocities

```java
// 2026
ChassisSpeeds s;  s.vxMetersPerSecond;  s.omegaRadiansPerSecond;

// 2027
ChassisVelocities v;  v.vx;  v.omega;
```

### +Operator perspective

```java
// 2026
setOperatorPerspectiveForward(rotation);

// 2027
setOperatorForwardDirection(rotation);
```

### Time conversion

`Utils.fpgaToCurrentTime()` is gone.

### Phoenix 6 vendordep

```text
Phoenix6-26.70.0-alpha-2.json   (wpilibYear 2027_alpha7)
```

---

## Project setup

- Gradle plugin: `org.wpilib.GradleRIO` version `2027.0.0-alpha-7`.
- Java 25.
- The main class is set with `application.mainClass`.
- Deploy directory: `/home/systemcore` instead of `/home/lvuser`.
- The build is offline-capable against `C:\Users\Public\wpilib\2027_alpha7\maven`.

---

## +Third-party libraries (no 2027 release yet)

- **PathPlanner:** autos, `AutoBuilder` and `NamedCommands` removed.
- **PhotonVision:** vision pose updates removed.
- **maple-sim:** physics sim removed; swerve uses CTRE `updateSimState` instead.
- **YASS:** 4 libraries for swerve and mechanisms https://www.yassrobotics.com/
