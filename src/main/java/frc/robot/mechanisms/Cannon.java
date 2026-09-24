// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.robot.mechanisms.Indexer.IndexerConstants;
import frc.robot.mechanisms.Shooter.ShooterConstants;
import frc.util.AllianceHelpers;
import frc.util.units.ThunderMap;

/**
 * Coordinates the shooter, turret, hood and indexer. It owns no hardware, so in v3 it is a plain
 * class rather than a Mechanism: its commands require the mechanisms they actually drive.
 */
public class Cannon {
    // ======== CANNON CONSTANTS ========

    public class CannonConstants {
        public static final Distance SMART_SHOOT_MIN_DISTANCE = Inches.of(64);
        public static final Translation2d SHOOTER_TRANSLATION = new Translation2d(Inches.of(5.75), Inches.of(-5.75));
        public static final Transform2d SHOOTER_TRANSFORM = new Transform2d(SHOOTER_TRANSLATION, new Rotation2d());
        public static final Distance SHOOTER_HEIGHT = Inches.of(18);

        public record CandShot(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity){};

        public static final ThunderMap<Distance, Time> TIME_OF_FLIGHT_MAP = new ThunderMap<Distance, Time>() {{
            put(Inches.of(248), Seconds.of(3.45-2.12));
            put(Inches.of(212), Seconds.of(1.55));
            put(Inches.of(161), Seconds.of(1.1));
            put(Inches.of(100), Seconds.of(0.95));
            put(Inches.of(80), Seconds.of(0.95));
        }};

        public static final int MAX_OTF_ITERATIONS = 10;
        public static final Distance OTF_TOLERANCE = Inches.of(1.5);

        public static final CandShot LEFT_SHOT = new CandShot(Degrees.of(0), Degrees.of(63), RotationsPerSecond.of(55)); //Temp
        public static final CandShot RIGHT_SHOT = new CandShot(Degrees.of(0), Degrees.of(63), RotationsPerSecond.of(55)); //Temp
        public static final CandShot MIDDLE_SHOT = new CandShot(Degrees.of(0), Degrees.of(80), RotationsPerSecond.of(53)); //Temp

        public static final Distance SHOOT_DISTANCE_BIAS = Inches.of(6);
    }


    // Mechanisms
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Swerve drivetrain;
    private Indexer indexer;

    // Target storage
    private Target storedTarget;

    private final TelemetryTable telemetry = Telemetry.getTable("Cannon");

    // Replaces LightningShuffleboard.getDouble(): a value editable from the dashboard
    private final TunableDouble shooterDistanceBiasInches = Tunables.addDouble("Cannon/Shooter Distance Bias", 8);

    /**
     * Creates a new cannon
     * @param shooter shooter
     * @param turret turret
     * @param hood hood
     * @param drivetrain serve
     * @param indexer indexer
     */
    public Cannon(Shooter shooter, Turret turret, Hood hood, Swerve drivetrain, Indexer indexer) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.drivetrain = drivetrain;

        this.indexer = indexer;

        this.storedTarget = FieldConstants.GOAL_POSITION;

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    private void periodic() {
        // Code to calculate target position
        Pose2d robotPose = drivetrain.getPose();

        boolean isTop = robotPose.getY() > FieldConstants.FIELD_MIDDLE_Y;

        if (drivetrain.isInZone()) {
            storedTarget =  FieldConstants.GOAL_POSITION;
        } else if (isTop) {
            storedTarget =  new Target(FieldConstants.ZONE_POSITION_BLUE_TOP, FieldConstants.ZONE_POSITION_RED_TOP);
        } else {
            storedTarget =  new Target(FieldConstants.ZONE_POSITION_BLUE_BOTTOM, FieldConstants.ZONE_POSITION_RED_BOTTOM);
        }

        telemetry.log("Target Position", getTargetTranslation());
        telemetry.log("Distance To Target", getTargetDistance());
        telemetry.log("Turret Position", new Pose2d(getShooterTranslation(), new Rotation2d()));
        telemetry.log("In No Passing Zone", isInNoPassingZone());
    }



    /**
     * Gets the translation of the shooter relative to the field
     * @param pose custom pose for the robot
     * @return Returns its position
     */
    public Translation2d getShooterTranslation(Pose2d pose) {
        return pose.transformBy(CannonConstants.SHOOTER_TRANSFORM).getTranslation();
    }

    /**
     * Gets the translation of the shooter relative to the field
     * @return Returns its position
     */
    public Translation2d getShooterTranslation() {
        return getShooterTranslation(drivetrain.getPose());
    }

    /**
     * Gets the translation of the target that we want to aim at (all components)
     * @return The transltion
     */
    public Target getTarget() {
        return storedTarget;
    }

    /**
     * Gets the current target's translati on according to the alliance
     * @return The translation
    */
    public Translation2d getTargetTranslation() {
        return FieldConstants.getTargetData(getTarget());
    }

    public Angle getTargetAngle(Pose2d pose) {
        return Radians.of(getTargetTranslation().minus(getShooterTranslation(pose)).getAngle().getRadians());
    }

    public Angle getTargetAngle() {
        return getTargetAngle(drivetrain.getPose());
    }

    /**
     * Gets the distance from the robot to the target
     * @return The distance
     */
    public Distance getTargetDistance() {
        return Meters.of(getTargetTranslation().minus(getShooterTranslation()).getNorm());
    }

    /**
     * Gets the distance from the robot to the target
     * @param pose custom pose to pass in
     * @return The distance
     */
    public Distance getTargetDistance(Pose2d pose) {
        return Meters.of(getTargetTranslation().minus(getShooterTranslation(pose)).getNorm());
    }

    /**
     * Returns a command to set the hood and shooter values
     * @param hoodAngle The angle of the hood to set at
     * @param shooterVelocity The velocity to set the shooter to
     * @return The command
     */
    public Command createCannonCommand(Angle hoodAngle, AngularVelocity shooterVelocity) {
        // Composition builders need a name before they become a Command
        return Command.parallel(
            shooter.shootCommand(shooterVelocity),
            hood.setPositionCommand(hoodAngle)
        ).named("Cannon Set");
    }

    /**
     * Returns a command to set the hood and shooter values
     * @param turretAngle the angle to set the turret to
     * @param hoodAngle the angle to set the hood to
     * @param shooterVelocity the velocity to set the shooter to
     * @return The command
     */
    public Command createCannonCommand(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity) {
        return Command.parallel(
            shooter.shootCommand(shooterVelocity),
            turret.setAngleCommand(turretAngle),
            hood.setPositionCommand(hoodAngle)
        ).named("Cannon Set With Turret");
    }

    /**
     * Creates a cand shot command that uses only hood and shooter.
     * Holds the shooter and hood at the canned values and feeds once on target.
     * @param value The cand shot value
     * @return The command
     */
    public Command createCandShotCommand(CannonConstants.CandShot value) {
        return Command.requiring(shooter, hood).executing(co -> {
            shooter.setVelocity(value.shooterVelocity());
            hood.setPosition(value.hoodAngle());
            // The indexer command runs as a child; it can't outlive this command
            co.await(indexWhenOnTarget());
        }).whenCanceled(() -> {
            shooter.stop();
            turret.stop();
            hood.stop();
            indexer.stop();
        }).named("Cand Shot");
    }

    /**
     * Creates a cand shot command that also uses turret
     * @param value The cand shot to use
     * @return The command
     */
    public Command createTurretCandShotCommand(CannonConstants.CandShot value) {
        return Command.requiring(shooter, hood, turret).executing(co -> {
            shooter.setVelocity(value.shooterVelocity());
            hood.setPosition(value.hoodAngle());
            turret.setAngle(value.turretAngle());
            co.await(indexWhenOnTarget());
        }).named("Turret Cand Shot");
    }

    /**
     * Sets the hood angle at a predetermined target by cannon
     * @return The command
     */
    public Command hoodAim() {
        return hood.hoodAim(this);
    }

    /**
     * Sets the hoods angle based on a passed in target
     * @param target The target to use
     * @return the command
     */
    public Command hoodAim(Target target) {
        return hood.hoodAim(this, target);
    }

    /**
     * Aims the turret at a predetermined target
     * @return The command
     */
    public Command turretAim() {
        return turret.turretAimCommand(this);
    }

    /**
     * Remodel of shooter aim-- automatically decides when to shoot
     * @return The command to run
     */
    public Command smartShoot() {
        return Command.noRequirements(co -> {
            // fork starts the shooter command alongside this one without waiting for it
            co.fork(shooter.runShootCommand(() -> ShooterConstants.VELOCITY_MAP.get(getTargetDistance())));
            co.waitUntil(() -> turret.isOnTarget() && hood.isOnTarget() && shooter.isOnTarget() && !isNearHub());
            co.await(indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER));
        }).whenCanceled(() -> {
            shooter.setPower(ShooterConstants.COAST_DC);
            indexer.stop();
        }).named("Smart Shoot");
    }

    /**
     * YAY
     * @return DA OTFFF
     */
    public Command shootOTF() {
        return Command.requiring(turret, shooter, hood).executing(co -> {
            while (true) { // hi david (from Bea)
                Time tof;

                Pose2d previousPose;
                Pose2d futurePose = drivetrain.getPose();

                Distance futureDist = getTargetDistance();

                for (int i = 0; i < CannonConstants.MAX_OTF_ITERATIONS; i++) {
                    tof = CannonConstants.TIME_OF_FLIGHT_MAP.get(futureDist);

                    previousPose = futurePose;
                    futurePose = getFuturePoseFromTime(tof);

                    futureDist = getTargetDistance(futurePose);

                    if (Math.abs(futurePose.minus(previousPose).getTranslation().getNorm()) < CannonConstants.OTF_TOLERANCE.in(Meters)) {
                        break;
                    }
                }

                Distance mapDist = futureDist;

                if (RobotState.isTeleop()) {
                    mapDist = futureDist.minus(Inches.of(shooterDistanceBiasInches.get()));
                }

                Angle hoodAngle = Hood.HoodConstants.HOOD_MAP.get(mapDist);
                AngularVelocity shooterVelocity = Shooter.ShooterConstants.VELOCITY_MAP.get(mapDist);

                hood.setPosition(hoodAngle);
                shooter.setVelocity(shooterVelocity);

                telemetry.log("Future Pose", futurePose);

                turret.turretAim(new Pose2d(getShooterTranslation(futurePose), futurePose.getRotation()), getTarget(), getRobotAngularVelocity(), getHubAngularVelocity(futurePose));

                co.yield();
            }
        }).named("Shoot OTF");
    }

    /**
     * finds the distance between shooter and hub and calculates if it is nearby.
     * @return if the distance of shooter on the field and the hub is less than 1
     */
    public boolean isNearHub() {
          Distance distance = Meters.of(this.getShooterTranslation().getDistance(FieldConstants.getTargetData(FieldConstants.GOAL_POSITION)));
          return distance.lt(CannonConstants.SMART_SHOOT_MIN_DISTANCE);
    }

    /**
     * starts the indexer when the hood, turret, and shooter is on target.
     * @return The command
     */
    public Command indexWhenOnTarget(){
        return Command.noRequirements(co -> {
            co.waitUntil(this::isOnTarget);
            co.await(indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER));
        }).named("Index When On Target");
    }

    /**
     * Checks if the cannon is in between the tower and hub on the opposite alliance zone
     * @return if the cannon is in the no passing zone
     */
    public boolean isInNoPassingZone() {
        return AllianceHelpers.isBlueAlliance() ? FieldConstants.RED_NO_PASSING_ZONE.contains(getShooterTranslation())
            : FieldConstants.BLUE_NO_PASSING_ZONE.contains(getShooterTranslation());
    }

    /**
     *  Checks if the hood, turret, and shooter is on target.
     * @return if they are on target.
     */
    public boolean isOnTarget(){
        return (hood.isOnTarget() && turret.isOnTarget() && shooter.isOnTarget());
    }

    /**
     * get the angular velocity of the hub based on the robot's velocity and position relative to the hub.
     * @return the angular velocity of the hub based on the robot's velocity and position relative to the hub
     * @param pose the pose to use for the calculation
     */
    public AngularVelocity getHubAngularVelocity(Pose2d pose) {
        double robotTargetAngle = getTargetAngle(pose).in(Radians);

        ChassisVelocities velocities = drivetrain.getWallCorrectedChassisVelocities();
        Translation2d fieldRelativeVelocity = new Translation2d(velocities.vx, velocities.vy).rotateBy(drivetrain.getPose().getRotation());

        double hubRotation = (-fieldRelativeVelocity.getX() * Math.sin(robotTargetAngle) + fieldRelativeVelocity.getY() * Math.cos(robotTargetAngle)) / getTargetDistance(pose).in(Meters);
        return RadiansPerSecond.of(hubRotation);
    }

    public AngularVelocity getRobotAngularVelocity() {
        return RadiansPerSecond.of(drivetrain.getWallCorrectedChassisVelocities().omega);
    }

    public Pose2d getFuturePoseFromTime(Time time) {
        ChassisVelocities speeds = drivetrain.getWallCorrectedChassisVelocities();
        double dt = time.in(Seconds);

        double driveMultiplier = 1;

        Pose2d pose = drivetrain.getPose();

        double filteredOmegaRadPerSec = speeds.omega;
        double filteredXVel = speeds.vx - 0.22;
        double filteredYVel = speeds.vy;

        double rrXVel = (-filteredOmegaRadPerSec * Cannon.CannonConstants.SHOOTER_TRANSLATION.getY());
        double rrYVel = (filteredOmegaRadPerSec * Cannon.CannonConstants.SHOOTER_TRANSLATION.getX());

        Twist2d twist = new Twist2d(
            (filteredXVel+ rrXVel) * dt * driveMultiplier,
            (filteredYVel + rrYVel) * dt * driveMultiplier,
            0
        );

        return pose.exp(twist);
    }
}
