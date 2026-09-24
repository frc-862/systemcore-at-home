// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;


import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.units.ThunderMap;

public class Shooter implements Mechanism {

    public class ShooterConstants {
        public static final boolean INVERTED = false; // temp
        public static final Current STATOR_LIMIT = Amps.of(160.0); // temp
        public static final boolean BRAKE = false; // temp
        public static final double COAST_DC = 0.3; // Shooter power when coasting

        public static final double kP = 0.5d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kV = 0.1211d;
        public static final double kS = 0.31;
        public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
        public static final AngularVelocity BIAS_DELTA = RotationsPerSecond.of(1);
        public static final Frequency UPDATE_FREQUENCY = Hertz.of(1000);

        public static final double GEAR_RATIO = 1d; // temp
        public static final Distance FLYWHEEL_CIRCUMFERENCE = Inches.of(4).times(Math.PI);

        // for sim to account for movement between shooter, fuel, and hood
        public static final double SHOOTER_EFFICIENCY = 0.53;
        // Input is distance to target in meters, output is shooter speed in rotations per second
        public static final ThunderMap<Distance, AngularVelocity> VELOCITY_MAP = new ThunderMap<>() {
            {
                put(Inches.of(248), RotationsPerSecond.of(64));
                put(Inches.of(212), RotationsPerSecond.of(62));
                put(Inches.of(161), RotationsPerSecond.of(54));
                put(Inches.of(100), RotationsPerSecond.of(45));
                put(Inches.of(80), RotationsPerSecond.of(43));
            }
        };

        // Sim
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.006); // temp
        public static final Time MAX_SHOOTING_PERIOD = Seconds.of(0.5); // 10 balls per second
    }

    private final ThunderBird motorLeft;
    private final ThunderBird motorRight;

    private final DutyCycleOut dutyCycle;
    private final VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    // Mutable measures (MutAngularVelocity) no longer exist; measures are immutable
    private AngularVelocity shooterBias;

    private TalonFXSimState motorSim;
    private FlywheelSim shooterSim;

    private final TelemetryTable telemetry = Telemetry.getTable("Shooter");

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        this(new ThunderBird(RobotMap.SHOOTER_LEFT, RobotMap.CAN_BUS,
            ShooterConstants.INVERTED, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE),
            new ThunderBird(RobotMap.SHOOTER_RIGHT, RobotMap.CAN_BUS,
            ShooterConstants.INVERTED, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE));
    }

    /**
     * Creates a new Shooter Subsystem.
     * @param motorLeft the ThunderBird motor to use for the shooter
     * @param motorRight the ThunderBird motor that follows for the shooter
     */
    public Shooter(ThunderBird motorLeft, ThunderBird motorRight) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);
        shooterBias = RotationsPerSecond.zero();
        targetVelocity = RotationsPerSecond.zero();


        //creates a config for the shooter motor
        TalonFXConfiguration config = motorLeft.getConfig();

        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        motorLeft.getMotorVoltage().setUpdateFrequency(ShooterConstants.UPDATE_FREQUENCY);

        motorLeft.applyConfig(config);
        motorRight.setControl(new Follower(RobotMap.SHOOTER_LEFT, MotorAlignmentValue.Opposed));

        if (RobotBase.isSimulation()){
            motorSim = motorLeft.getSimState();
            motorSim.setMotorType(MotorType.KrakenX60);

            shooterSim = new FlywheelSim(Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX60Foc(2), ShooterConstants.MOI.in(KilogramSquareMeters),
                ShooterConstants.GEAR_RATIO), DCMotor.getKrakenX60Foc(2));
        }

        // Mechanisms have no periodic() hook; register callbacks with the scheduler instead
        Scheduler.getDefault().addPeriodic(this::periodic);
        if (RobotBase.isSimulation()) {
            Scheduler.getDefault().addPeriodic(this::simulationPeriodic);
        }
    }

    private void periodic() {
        telemetry.log("TargetVelocity", getTargetVelocity());
        telemetry.log("Bias", getBias());
        telemetry.log("OnTarget", isOnTarget());
        telemetry.log("VelocityLeft", getLeftVelocity());
        telemetry.log("VelocityRight", getRightVelocity());
    }

    private void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        shooterSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        shooterSim.update(TimedRobot.DEFAULT_PERIOD);

        motorSim.setRotorVelocity(shooterSim.getAngularVelocity());
    }

     /**
     * Set the power of the shooter motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        motorLeft.setControl(dutyCycle.withOutput(power));
        targetVelocity = RotationsPerSecond.zero();
    }

    /**
     * stops all movement to the shooter motor
     */
    public void stop() {
        motorLeft.stopMotor();
        targetVelocity = RotationsPerSecond.zero();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        applyChange();
    }

    /**
     * Change the bias of the shooter velocity. If the shooter is currently running it will update the velocity with the bias.
     * @param bias in rotations per second
     */
    public void changeBias(AngularVelocity bias) {
        shooterBias = shooterBias.plus(bias);
        if (targetVelocity.gt(RotationsPerSecond.zero())) {
            applyChange();
        }
    }

    public void clearBias() {
        shooterBias = RotationsPerSecond.zero();
    }

    // Instant commands without requirements: the body runs once and the command ends
    public Command changeBiasCommand(AngularVelocity bias) {
        return Command.noRequirements(co -> changeBias(bias)).named("Shooter Change Bias");
    }

    public Command clearBiasCommand() {
        return Command.noRequirements(co -> clearBias()).named("Shooter Clear Bias");
    }

    public void setBias(AngularVelocity bias) {
        shooterBias = bias;
        if (targetVelocity.gt(RotationsPerSecond.zero())) {
            applyChange();
        }
    }

    private void applyChange() {
        motorLeft.setControl(velocityPID.withVelocity(getTargetVelocityWithBias().in(RotationsPerSecond)));
    }

    /**
     * Get the current bias of the shooter velocity.
     * @return the current bias in rotations per second
     */
    public AngularVelocity getBias() {
        return shooterBias;
    }

    /**
     * @return the velocity of the shooter motor
     */
    public AngularVelocity getLeftVelocity(){
        return motorLeft.getVelocity().getValue();
    }

    public AngularVelocity getRightVelocity(){
        return motorRight.getVelocity().getValue();
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public AngularVelocity getTargetVelocityWithBias() {
        return targetVelocity.plus(shooterBias);
    }
    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean isOnTarget(){
        return getLeftVelocity().isNear(getTargetVelocityWithBias(), ShooterConstants.TOLERANCE);
    }

    /**
     * velocity control command for shooter
     * @param velocity the velociyt to set the shooter as
     * @return the command for running the shooter
     */
    public Command shootCommand(AngularVelocity velocity) {
        return shootCommand(() -> velocity);
    }

    /**
     * velocity control command for shooter. Ends once the shooter is up to speed.
     * @param velocitySupplier the supplier for velocity
     * @return the command for running the shooter
     */
    public Command shootCommand(Supplier<AngularVelocity> velocitySupplier) {
        return run(co -> {
            setVelocity(velocitySupplier.get());
            co.waitUntil(this::isOnTarget);
        }).named("Shooter Shoot");
    }

    /**
     * velocity control command for shooter.
     * @param velocitySupplier the supplier for velocity
     * @return command for constantly running the shooter
     */
    public Command runShootCommand(Supplier<AngularVelocity> velocitySupplier) {
        return runRepeatedly(() -> setVelocity(velocitySupplier.get())).named("Shooter Run Shoot");
    }

    /**
     * Sets shooter motor into an idle power
     * @return the command for running the shooter at coast power
     */
    public Command coast() {
        return run(co -> setPower(ShooterConstants.COAST_DC)).named("Shooter Coast");
    }
}
