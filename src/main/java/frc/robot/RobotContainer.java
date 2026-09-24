// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.MathUtil;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.RobotMap;
import frc.robot.mechanisms.Cannon;
import frc.robot.mechanisms.Cannon.CannonConstants;
import frc.robot.mechanisms.Collector;
import frc.robot.mechanisms.Collector.CollectorConstants;
import frc.robot.mechanisms.Hood;
import frc.robot.mechanisms.Indexer;
import frc.robot.mechanisms.Indexer.IndexerConstants;
import frc.robot.mechanisms.Shooter;
import frc.robot.mechanisms.Shooter.ShooterConstants;
import frc.robot.mechanisms.Swerve;
import frc.robot.mechanisms.SwerveTelemetry;
import frc.robot.mechanisms.Turret;
import frc.robot.mechanisms.Turret.TurretConstants;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDBooleanSupplier;
import frc.util.leds.LEDMechanism;


public class RobotContainer {

    private final XboxController driver;
    private final XboxController copilot;

    private final Swerve drivetrain;
    private final Collector collector;
    private final Indexer indexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final LEDMechanism leds;
    private final SwerveTelemetry logger;
    private final Cannon cannon;

    public RobotContainer() {
        driver = new XboxController(RobotMap.DRIVER_PORT);
        copilot = new XboxController(RobotMap.COPILOT_PORT);

        drivetrain = DriveConstants.createDrivetrain();

        logger = new SwerveTelemetry(DriveConstants.MaxSpeed.in(MetersPerSecond));
        leds = new LEDMechanism(LED_STATES.values().length, LEDConstants.LED_COUNT, LEDConstants.LED_PWM_PORT);

        collector = new Collector();
        indexer = new Indexer();
        shooter = new Shooter();
        hood = new Hood();
        turret = new Turret(drivetrain);
        cannon = new Cannon(shooter, turret, hood, drivetrain, indexer);

        configureDefaultCommands();
        configureBindings();
        configureLeds();
    }

    private void configureDefaultCommands() {
        /* Driver */
        /*
        * Note that X is defined as forward according to WPILib convention,
        * Y is defined as to the left according to WPILib convention.
        */
        drivetrain.setDefaultCommand(
            drivetrain.driveCommand(
                () -> MathUtil.copyDirectionPow(
                    MathUtil.applyDeadband(
                        VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()),
                        DriveConstants.JOYSTICK_DEADBAND
                    ),
                        DriveConstants.CONTROLLER_POW).times(driver.getRightTrigger() > DriveConstants.TRIGGER_DEADBAND
                        ? DriveConstants.SLOW_MODE_MULT : 1.0), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband((-driver.getRightX()),
                        DriveConstants.JOYSTICK_DEADBAND), DriveConstants.CONTROLLER_POW)
                        * (driver.getRightTrigger() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0)));

        // v3 default commands may only require their own mechanism, so the shooter can no longer
        // default to cannon.shootOTF() (which also requires turret and hood). Use a shooter-only
        // version without the on-the-fly lead compensation.
        shooter.setDefaultCommand(shooter.runShootCommand(() -> ShooterConstants.VELOCITY_MAP.get(cannon.getTargetDistance())));
        hood.setDefaultCommand(cannon.hoodAim());
        turret.setDefaultCommand(cannon.turretAim());
    }

    private void configureBindings() {

        /* Driver */
        new Trigger(driver::getXButton).whileTrue(drivetrain.brakeCommand());

        new Trigger(() -> copilot.getBButton() && !cannon.isInNoPassingZone())
            .whileTrue(Command.parallel(cannon.shootOTF(), cannon.indexWhenOnTarget()).named("Shoot OTF And Index"));

        // change biases for the driver
        new Trigger(driver::getDpadDownButton).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(driver::getDpadUpButton).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA));
        new Trigger(driver::getDpadLeftButton).onTrue(shooter.clearBiasCommand());
        new Trigger(driver::getDpadRightButton).onTrue(shooter.clearBiasCommand());

        new Trigger(() -> driver.getLeftBumperButton() && !driver.getRightBumperButton()).onTrue(turret.changeBias(TurretConstants.BIAS_DELTA));
        new Trigger(() -> driver.getRightBumperButton() && !driver.getLeftBumperButton()).onTrue(turret.changeBias(TurretConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(() -> driver.getLeftBumperButton() && driver.getRightBumperButton()).onTrue(turret.clearBias());

        // reset the field-centric heading. Start and Back are now called Menu and View.
        new Trigger(() -> (driver.getMenuButton() && driver.getViewButton()))
            .onTrue(drivetrain.resetFieldCentricCommand())
            .onTrue(leds.enableStateWithTimeout(LEDConstants.LED_STATES.SEED_FIELD_FORWARD.id(), 1));

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(() -> driver.getLeftTrigger() > DriveConstants.TRIGGER_DEADBAND).whileTrue(collector.driverStowPivotCommand());

        new Trigger(driver::getBButton).toggleOnTrue(turret.manual());

        /* Copilot */
        new Trigger(() -> drivetrain.isNearTrench()).whileTrue(hood.retractCommand());

        new Trigger(copilot::getLeftBumperButton).whileTrue(indexer.indexCommand(-IndexerConstants.SPINDEXDER_POWER,
            -IndexerConstants.TRANSFER_POWER));
        new Trigger(copilot::getRightBumperButton).whileTrue(indexer.indexCommand(IndexerConstants.SPINDEXDER_POWER,
            IndexerConstants.TRANSFER_POWER));

        new Trigger(copilot::getMenuButton).onTrue(collector.stowPivotCommand());

        // Interruption behavior kCancelIncoming is replaced by command priorities (see collectCommand)
        new Trigger(() -> (copilot.getRightTrigger() > DriveConstants.TRIGGER_DEADBAND || copilot.getLeftTrigger() > DriveConstants.TRIGGER_DEADBAND)  && !(driver.getLeftTrigger() > DriveConstants.TRIGGER_DEADBAND))
            .whileTrue(collector.collectCommand(() -> (copilot.getRightTrigger() - copilot.getLeftTrigger()) *  CollectorConstants.COLLECT_MULT));

        new Trigger(copilot::getViewButton).whileTrue(turret.manual()); // disable turret

        new Trigger(() -> Math.abs(copilot.getRightX()) > TurretConstants.MANUAL_CONTROL_DEADBAND).whileTrue(turret.setManualPowerCommand(() -> copilot.getRightX()));

        // Temp Cand shots
        new Trigger(copilot::getDpadRightButton).whileTrue(Command.race(cannon.createCandShotCommand(CannonConstants.RIGHT_SHOT), rumble()).named("Right Shot"));
        new Trigger(copilot::getDpadLeftButton).whileTrue(Command.race(cannon.createCandShotCommand(CannonConstants.LEFT_SHOT), rumble()).named("Left Shot"));
        new Trigger(copilot::getDpadUpButton).whileTrue(Command.race(cannon.createCandShotCommand(CannonConstants.MIDDLE_SHOT), rumble()).named("Middle Shot"));

        new Trigger(() -> (hood.isOnTarget() && shooter.isOnTarget() && turret.isOnTarget()))
            .whileTrue(Command.noRequirements(co -> {
                setCopilotRumble(1);
                co.park();
            }).whenCanceled(() -> setCopilotRumble(0)).named("On Target Rumble"));

        // autoIndex stops the indexer when canceled, so the old onFalse(stop) binding isn't needed
        new Trigger(copilot::getAButton).whileTrue(indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER));

        // Unjam: reverse briefly, then feed. A v2 withTimeout().andThen() chain is straight-line code.
        new Trigger(copilot::getYButton).whileTrue(indexer.run(co -> {
            indexer.setPower(-0.5);
            co.wait(Seconds.of(0.1));
            indexer.setPower(1);
            co.park();
        }).whenCanceled(indexer::stop).named("Unjam"));

        new Trigger(driver::getXButton).whileTrue(Command.noRequirements(co -> drivetrain.resetPose(new Pose2d(12.566, 0.713, new Rotation2d(0.057)))).named("Reset Pose"));
    }

    /**
     * Without PathPlanner and SendableChooser there is a single hard-coded auto: the old
     * SMART_SHOOT named command. (OpModeRobot with @Autonomous classes is the 2027 replacement
     * for an auto chooser.)
     */
    public Command getAutonomousCommand() {
        return Command.race(
            Command.parallel(
                cannon.shootOTF(),
                hood.ignoreRetractCommand(),
                indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER)
            ).named("Smart Shoot Parts"),
            leds.enableState(LED_STATES.SHOOT.id())
        ).named("Smart Shoot Auto");
    }

    private void configureLeds() {
        leds.setDefaultBehavior(LEDBehaviorFactory.swirl(LEDConstants.stripAll, 10, 5, Color.ORANGE, Color.BLUE));

        leds.setBehavior(LED_STATES.TEST.id(), LEDBehaviorFactory.testStrip(LEDConstants.stripAll,
                () -> turret.getMaxLimitSwitch(),
                () -> turret.getZeroLimitSwitch()
        ));
        leds.setBehavior(LED_STATES.TURRET_MANUAL.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.GREY));
        leds.setBehavior(LED_STATES.VISION_BAD.id(), LEDBehaviorFactory.solid(LEDConstants.stripUnderglow, Color.RED));
        leds.setBehavior(LED_STATES.TURRET_BAD.id(), LEDBehaviorFactory.pulse(LEDConstants.stripShooter, 2, Color.ORANGE));

        leds.setBehavior(LED_STATES.SEED_FIELD_FORWARD.id(), LEDBehaviorFactory.rainbow(LEDConstants.stripAll, 2));
        leds.setBehavior(LED_STATES.HOOD_STOWED.id(), LEDBehaviorFactory.solid(LEDConstants.stripShooter, Color.BLUE));

        leds.setBehavior(LED_STATES.SHOOT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.PURPLE));
        leds.setBehavior(LED_STATES.COLLECT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.YELLOW));
        leds.setBehavior(LED_STATES.CLIMB.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.YELLOW));

        leds.setBehavior(LED_STATES.CANNED_SHOT_READY.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.GREEN));
        leds.setBehavior(LED_STATES.NEAR_HUB.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 3, Color.RED));

        new Trigger(hood::isStowed).whileTrue(leds.enableState(LED_STATES.HOOD_STOWED.id()));

        new Trigger(turret::getManual).whileTrue(leds.enableState(LED_STATES.TURRET_MANUAL.id()));

        new Trigger(() -> !turret.getZeroed() && RobotState.isDisabled()).whileTrue(leds.enableState(LED_STATES.TURRET_BAD.id()));

        new Trigger(new LEDBooleanSupplier(turret::getZeroed)).whileFalse(leds.enableState(LED_STATES.TURRET_BAD.id())); // turn off turret bad LED state once turret is zeroed

        // Turn off the "vision bad" LED state once the drivetrain has moved away from the origin, indicating we likely have a valid pose estimate.
        new Trigger(new LEDBooleanSupplier(() -> (RobotState.isDisabled() && drivetrain.getPose().getTranslation().getDistance(new Translation2d()) < 0.1))).whileTrue(leds.enableState(LED_STATES.VISION_BAD.id()));

        new Trigger(new LEDBooleanSupplier(RobotState::isDisabled)).whileTrue(leds.enableState(LED_STATES.TEST.id()));

        //Turn on the NEAR_HUB light when it is near the HUB.
        new Trigger(() -> cannon.isNearHub()).whileTrue(leds.enableState(LED_STATES.NEAR_HUB.id()));

        // Turn on the NEAR_HUB light when in no passing zone
        new Trigger(() -> cannon.isInNoPassingZone()).whileTrue(leds.enableState(LED_STATES.NEAR_HUB.id()));
    }

    private void setCopilotRumble(double value) {
        // RumbleType.kBothRumble is gone; set each motor
        copilot.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, value);
        copilot.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, value);
    }

    /**
     * It waits until everything except turret is on target to start the rumble.
     * @return The command
     */
    public Command rumble(){
        return Command.noRequirements(co -> {
            co.waitUntil(() -> (hood.isOnTarget() && shooter.isOnTarget() && !turret.isOnTarget()));
            setCopilotRumble(1);
            co.park();
        }).whenCanceled(() -> setCopilotRumble(0)).named("Rumble");
    }

}
