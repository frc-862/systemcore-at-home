// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.system.DCMotor;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.units.ThunderMap;
import frc.util.units.ThunderUnits;

public class Hood implements Mechanism {

    public class HoodConstants {
        public static final boolean INVERTED = false; // temp
        public static final Current STATOR_LIMIT = Amps.of(10); // temp
        public static final Current SUPPLY_LIMIT = Amps.of(4); // temp
        public static final boolean SUPPLY_LIMIT_ENABLE = true; // temp
        public static final boolean BRAKE = true; // temp

        public static final Angle MIN_ANGLE = Degrees.of(50);
        public static final Angle MAX_ANGLE = Degrees.of(80);
        public static final Angle ZERO_ANGLE = Degrees.of(80);

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.004); // Temp
        public static final Distance MECHANISM_LENGTH = Inches.of(6);

        public static final ThunderMap<Distance, Angle> HOOD_MAP = new ThunderMap<>() {
            {
                put(Inches.of(248), Degrees.of(70));
                put(Inches.of(212), Degrees.of(75));
                put(Inches.of(161), Degrees.of(77));
                put(Inches.of(100), Degrees.of(80));
                put(Inches.of(80), Degrees.of(80));
            }
        };

        public static final double kS = 0.33d;
        public static final double kG = 0;
        public static final double kP = 500d;
        public static final double kI = 0.0;
        public static final double kD = 12d;

        public static final Angle POSITION_TOLERANCE = Degrees.of(0.5); // temp
        public static final Angle BIAS_DELTA = Degrees.of(0.5); // temp

        // Conversion ratios
        public static final double ROTOR_TO_ENCODER_RATIO = !hasEncoder() ? 1 : 9*42/18d;
        public static final double ENCODER_TO_MECHANISM_RATIO = 9d * 156/15d;
        public static final double ROTOR_TO_MECHANISM_RATIO = ROTOR_TO_ENCODER_RATIO * ENCODER_TO_MECHANISM_RATIO; // only used in sim

        public static final Angle OFFSET_TO_MAX = Rotations.of(0d); // temp
        public static final Angle ENCODER_OFFSET = OFFSET_TO_MAX.plus(MAX_ANGLE);
        public static final DutyCycleOut HOOD_ZEROING_DC = new DutyCycleOut(0.2);
    }

    private ThunderBird motor;
    private CANcoder encoder;

    final PositionVoltage request;
    private Angle targetAngle;
    private Angle hoodBias;

    private boolean hoodZeroed = false;
    private final Timer zeroingTimer = new Timer();

    private SingleJointedArmSim hoodSim;
    private TalonFXSimState motorSim;
    private DCMotor gearbox;
    private MechanismLigament2d ligament;
    private MechanismRoot2d root2d;
    private Mechanism2d mech2d;
    private CANcoderSimState encoderSim;
    public boolean isHoodRetracted = false;
    public boolean ignoreHoodRetract = false;

    private final TelemetryTable telemetry = Telemetry.getTable("Hood");

    /** Creates a new Hood Mechanism. */
    public Hood() {
        motor = new ThunderBird(RobotMap.HOOD, RobotMap.CAN_BUS, HoodConstants.INVERTED, HoodConstants.STATOR_LIMIT,
            HoodConstants.BRAKE);

        if (hasEncoder()) {
            encoder = new CANcoder(RobotMap.HOOD_ENCODER, RobotMap.CAN_BUS);
        }

        TalonFXConfiguration motorConfig = motor.getConfig();

        request = new PositionVoltage(0d);

        targetAngle = Degrees.zero();

        hoodBias = Degrees.zero();

        if (hasEncoder()) {
            CANcoderConfiguration angleConfig = new CANcoderConfiguration();
            angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1d;
            angleConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            encoder.getConfigurator().apply(angleConfig);
        }

        motorConfig.Slot0.kP = HoodConstants.kP;
        motorConfig.Slot0.kI = HoodConstants.kI;
        motorConfig.Slot0.kD = HoodConstants.kD;
        motorConfig.Slot0.kS = HoodConstants.kS;
        motorConfig.Slot0.kG = HoodConstants.kG;

        if (hasEncoder()) {
            motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
            motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        }

        motorConfig.Feedback.SensorToMechanismRatio = HoodConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = HoodConstants.ROTOR_TO_ENCODER_RATIO;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.SUPPLY_LIMIT_ENABLE;
        motorConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_LIMIT.in(Amps);

        motor.applyConfig(motorConfig);


        if (RobotBase.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new SingleJointedArmSim(
                gearbox,
                HoodConstants.ENCODER_TO_MECHANISM_RATIO,
                HoodConstants.MOI.magnitude(),
                HoodConstants.MECHANISM_LENGTH.in(Meters),
                HoodConstants.MIN_ANGLE.in(Radians),
                HoodConstants.MAX_ANGLE.in(Radians),
                false,
                HoodConstants.MAX_ANGLE.in(Radians)
            );

            motorSim = motor.getSimState();
            motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
            encoderSim = encoder.getSimState();

            encoderSim.Orientation = ChassisReference.Clockwise_Positive;

            motorSim.setRawRotorPosition(HoodConstants.MAX_ANGLE.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));
            encoderSim.setRawPosition(HoodConstants.MAX_ANGLE.times(HoodConstants.ENCODER_TO_MECHANISM_RATIO));

            mech2d = new Mechanism2d(2,  2);
            root2d =  mech2d.getRoot("Hood", 0.2, 0.2);

            ligament = root2d.append(new MechanismLigament2d("Hood", 1.5, HoodConstants.MAX_ANGLE.in(Degrees)));
        }

        if (!hasEncoder()){
            motor.setPosition(HoodConstants.ZERO_ANGLE); // needs to be after config and sim
        }

        Scheduler.getDefault().addPeriodic(this::periodic);
        if (RobotBase.isSimulation()) {
            Scheduler.getDefault().addPeriodic(this::simulationPeriodic);
        }
    }

    private void periodic() {
        if (!hoodZeroed && RobotState.isEnabled()) {
            if (!zeroingTimer.isRunning()) {
                zeroingTimer.restart();
                motor.setControl(HoodConstants.HOOD_ZEROING_DC);
            } else if (!motor.getVelocity().getValue().isNear(RotationsPerSecond.zero(), RotationsPerSecond.of(0.1))) {
                zeroingTimer.restart();
            } else if (zeroingTimer.hasElapsed(0.5)) {
                motor.setPosition(HoodConstants.MAX_ANGLE);
                hoodZeroed = true;
                motor.stopMotor();
                setPosition(targetAngle);
                zeroingTimer.stop();
            }
        }
        if (isHoodRetracted && !ignoreHoodRetract && hoodZeroed) {
            motor.setControl(request.withPosition(HoodConstants.MAX_ANGLE));
        }

        telemetry.log("Angle", getAngle());
        if (hasEncoder()) {
            telemetry.log("CANcoder angle", encoder.getAbsolutePosition().getValue());
        }
        telemetry.log("TargetAngle", getTargetAngle());
        telemetry.log("Bias", getBias());
        telemetry.log("OnTarget", isOnTarget());
        telemetry.log("Zeroed", hoodZeroed);
    }

    private void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        hoodSim.setInputVoltage(motorSim.getMotorVoltage());
        hoodSim.update(TimedRobot.DEFAULT_PERIOD);

        Angle simAngle = Radians.of(hoodSim.getAngleRads());

        motorSim.setRawRotorPosition(simAngle.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));

        ligament.setAngle(simAngle.in(Degrees));
        encoderSim.setRawPosition(simAngle.times(HoodConstants.ENCODER_TO_MECHANISM_RATIO));

        telemetry.log("Sim Angle", simAngle);
        // Mechanism2d is a TelemetryLoggable instead of a Sendable
        telemetry.log("Mech2d", mech2d);
    }

    private static boolean hasEncoder(){
        return RobotBase.isSimulation();
    }

    /**
     * Sets position of the hood
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetAngle = ThunderUnits.clamp(position, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        applyControl();
    }

    /**
     * Changes the bias and adds hoodBias to bias. Adds a certain amount of degrees to the hood's target position.
     * @param bias the amount of degrees to add to the hood target position going forward.
     */
    public void changeBias(Angle bias) {
        hoodBias = hoodBias.plus(bias);
        applyControl();
    }

    public Command changeBiasCommand(Angle bias) {
        return Command.noRequirements(co -> changeBias(bias)).named("Hood Change Bias");
    }

    public void setBias(Angle bias) {
        hoodBias = bias;
        applyControl();
    }

    private void applyControl() {
        if ((!isHoodRetracted || ignoreHoodRetract) && hoodZeroed) {
            motor.setControl(request.withPosition(getTargetAngleWithBias()));
        }
    }


    /**
     * Gets the current angle of the hood
     * @return current angle
     */
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    /**
     * Gets the target angle of the hood
     * @return target angle without the bias.
     */
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /**
     * Gets the target angle with bias.
     * @return target angle with the bias added.
     */
    public Angle getTargetAngleWithBias() {
        return ThunderUnits.clamp(targetAngle.plus(hoodBias), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
    }

    /**
     * Gets the bias of the hood
     * @return the bias of hood.
     */
    public Angle getBias() {
        return hoodBias;
    }

    /**
     * Returns true if the hood is on target
     * @return
     * True if on target, false otherwise
     */
    public boolean isOnTarget() {
        return getAngle().isNear(getTargetAngleWithBias(), HoodConstants.POSITION_TOLERANCE);
    }

    public boolean isStowed() {
        return getAngle().isNear(HoodConstants.MAX_ANGLE, HoodConstants.POSITION_TOLERANCE);
    }

    /**
     * Stops all movement to the hood motor
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * angle control command for hood
     * @param hoodAngle
     * @return the command for running the hood
     */
    public Command hoodCommand(Angle hoodAngle) {
        return hoodCommand(() -> hoodAngle);
    }

    /**
     * angle control command for hood. Ends once the hood reaches the angle.
     * @param hoodAngleSupplier
     * @return the command for running the hood
     */
    public Command hoodCommand(Supplier<Angle> hoodAngleSupplier) {
        return run(co -> {
            setPosition(hoodAngleSupplier.get());
            co.waitUntil(this::isOnTarget);
        }).named("Hood Set Angle");
    }

    /**
     * Retracts the hood to its maximum angle ignoring the bias.
     * @return the command for retracting the hood
     */
    public Command retractCommand() {
        // do not require hood because it should run while allowing the hood to do something else (smart shoot)
        return Command.noRequirements(co -> {
            isHoodRetracted = true;
            co.park();
        }).whenCanceled(() -> isHoodRetracted = false).named("Hood Retract");
    }


    /**
     * While this command is running, the hood will not be forced to retract, used for smart shoot in auton
     * @return the command
     */
    public Command ignoreRetractCommand() {
        // do not require hood because it should run while allowing the hood to do something else (smart shoot)
        return Command.noRequirements(co -> {
            ignoreHoodRetract = true;
            co.park();
        }).whenCanceled(() -> ignoreHoodRetract = false).named("Hood Ignore Retract");
    }

    /**
     * keeps the hood pointed at the target of the Robot.
     * @param cannon
     * @return Command for repositioning the hood.
     */
    public Command hoodAim(Cannon cannon){
        return runRepeatedly(() -> {
            Distance distance = Meters.of(cannon.getShooterTranslation().getDistance(cannon.getTargetTranslation()));
            Angle targetAngle = HoodConstants.HOOD_MAP.get(distance);
            setPosition(targetAngle);
        }).named("Hood Aim");
    }

    /**
     * Aims the hood at the target
     * @param cannon The cannon
     * @param target The target
     * @return the command
     */
    public Command hoodAim(Cannon cannon, Target target){
        return runRepeatedly(() -> {
            Distance distance = Meters.of(cannon.getShooterTranslation().getDistance(FieldConstants.getTargetData(target)));
            Angle targetAngle = HoodConstants.HOOD_MAP.get(distance);
            setPosition(targetAngle);
        }).named("Hood Aim At Target");
    }

    /**
     * A command to set the posiiton of the hood
     * @param angle the angle to set it at
     * @return the command
     */
    public Command setPositionCommand(Angle angle) {
        return Command.noRequirements(co -> setPosition(angle)).named("Hood Set Position");
    }
}
