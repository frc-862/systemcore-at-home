package frc.robot.mechanisms;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.units.ThunderUnits;
public class Collector implements Mechanism {


    public class CollectorConstants {
        // Collector Rollers
        public static final boolean INVERTED = true; // temp
        public static final Current STATOR_LIMIT = Amps.of(80); // temp
        public static final Current CURRENT_THRESHOLD = Amps.of(20); // temp
        public static final boolean BRAKE = true; // temp
        public static final double COLLECT_POWER = 1d;
        public static final double COLLECT_MULT = 1d;

        public static final MomentOfInertia COLLECTOR_MOI = KilogramSquareMeters.of(0.001); //temp
        public static final double COLLECTOR_GEAR_RATIO = 1d; //temp

        // pivot motor config
        public static final double PIVOT_KP = 15d; // temp
        public static final double PIVOT_KI = 0d; // temp
        public static final double PIVOT_KD = 0d; // temp
        public static final double PIVOT_KS = 0; // temp
        public static final double PIVOT_KG = 0d; // temp


        // pivot
        public static final boolean PIVOT_INVERTED = true;
        public static final Current PIVOT_STATOR_LIMIT = Amps.of(100); // temp
        public static final Current PIVOT_SUPPLY_LIMIT = Amps.of(20); // temp
        public static final boolean PIVOT_SUPPLY_LIMIT_ENABLE = true; // temp
        public static final boolean PIVOT_BRAKE_MODE = true; // temp
        public static final double PIVOT_ZERO_TIMER_THRESHOLD = 1;
        public static final Current COLLECTOR_SUPPLY_LIMIT = Amps.of(40); // temp
        public static final boolean COLLECTOR_SUPPLY_LIMIT_ENABLE = true; // temp
        public static final double ROTOR_TO_ENCODER_RATIO = 1d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 9d * 24d/10d; // temp
        public static final Angle MIN_ANGLE = Rotations.of(0);
        public static final Angle MAX_ANGLE = Rotations.of(0.35);
        public static final Angle NEUTRAL_ANGLE = Rotations.of(0.25);
        public static final Angle DEPLOY_ANGLE = MAX_ANGLE;
        public static final Angle STOW_ANGLE = MIN_ANGLE;
        public static final Angle TOLERANCE = Rotations.of(0.05);
        public static final DutyCycleOut PIVOT_ZEROING_DC_STOW = new DutyCycleOut(-0.1);
        public static final DutyCycleOut PIVOT_ZEROING_DC_DEPLOY = new DutyCycleOut(0.1);

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01); // temp
        public static final Distance LENGTH = Inches.of(6);

        // Sim
        public static final Distance WIDTH = Inches.of(27); // temp
        public static final Distance LENGTH_EXTENDED = Inches.of(7); // temp
        public static final int ROBOT_FUEL_CAPACITY = 50; // temp
        public static final AngularVelocity SIM_COLLECTING_THRESHOLD = RotationsPerSecond.of(1); // temp
    }

    // Collector Rollers
    private ThunderBird collectorMotor;
    private TalonFXSimState collectorMotorSim;
    private DCMotorSim collectorSim;
    private final DutyCycleOut collectorDutyCycle;
    private DCMotor collectorGearbox;

    // Pivot
    private ThunderBird pivotMotor;
    private TalonFXSimState pivotMotorSim;
    private SingleJointedArmSim collectorPivotSim;
    private Mechanism2d mech2d;
    private MechanismRoot2d root2d;
    private MechanismLigament2d ligament;
    private Angle targetPivotPosition;
    private PositionVoltage positionPID;
    private boolean pivotZeroed = false;
    private final Timer zeroingTimer = new Timer();
    private boolean pivotActive = false;
    private boolean stowZero = true;

    private DCMotor gearbox;

    private final TelemetryTable telemetry = Telemetry.getTable("Collector");


    /**
     * Creates a new Collector Mechanism.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.INVERTED, CollectorConstants.STATOR_LIMIT, CollectorConstants.BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        targetPivotPosition = CollectorConstants.STOW_ANGLE;

        collectorDutyCycle = new DutyCycleOut(0.0);

        TalonFXConfiguration pivotConfig = pivotMotor.getConfig();
        TalonFXConfiguration collectorConfig = collectorMotor.getConfig();
        positionPID = new PositionVoltage(0);

        pivotConfig.Slot0.kP = CollectorConstants.PIVOT_KP;
        pivotConfig.Slot0.kI = CollectorConstants.PIVOT_KI;
        pivotConfig.Slot0.kD = CollectorConstants.PIVOT_KD;
        pivotConfig.Slot0.kS = CollectorConstants.PIVOT_KS;
        pivotConfig.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = CollectorConstants.PIVOT_SUPPLY_LIMIT_ENABLE;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = CollectorConstants.PIVOT_SUPPLY_LIMIT.in(Amps);

        collectorConfig.CurrentLimits.SupplyCurrentLimitEnable = CollectorConstants.COLLECTOR_SUPPLY_LIMIT_ENABLE;
        collectorConfig.CurrentLimits.SupplyCurrentLimit = CollectorConstants.COLLECTOR_SUPPLY_LIMIT.in(Amps);

        pivotMotor.applyConfig(pivotConfig);
        collectorMotor.applyConfig(collectorConfig);


        if(RobotBase.isSimulation()){
            // pivot sim stuff
            gearbox = DCMotor.getKrakenX60Foc(1);

            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ENCODER_TO_MECHANISM_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.in(Meters), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), true,
            CollectorConstants.STOW_ANGLE.in(Radians));

            pivotMotorSim = pivotMotor.getSimState();
            pivotMotorSim.Orientation = ChassisReference.Clockwise_Positive;
            pivotMotorSim.setRawRotorPosition(CollectorConstants.STOW_ANGLE);

            // collector sim stuff
            // LinearSystemId.createDCMotorSystem is gone; a gravity-free arm model is the same system
            collectorGearbox = DCMotor.getKrakenX60(1);
            collectorSim = new DCMotorSim(
                Models.singleJointedArmFromPhysicalConstants(collectorGearbox, CollectorConstants.COLLECTOR_MOI.magnitude(), CollectorConstants.COLLECTOR_GEAR_RATIO),
                collectorGearbox
            );

            collectorMotorSim = collectorMotor.getSimState();
            collectorMotorSim.setMotorType(MotorType.KrakenX60);

            mech2d = new Mechanism2d(3, 3);
            root2d = mech2d.getRoot("Collector", 0.5, 0.5);
            ligament = root2d.append(new MechanismLigament2d("Collector", 2, 90));
        }

        Scheduler.getDefault().addPeriodic(this::periodic);
        if (RobotBase.isSimulation()) {
            Scheduler.getDefault().addPeriodic(this::simulationPeriodic);
        }
    }

    private void periodic() {
        if (!pivotZeroed && RobotState.isEnabled()) {
            if (!zeroingTimer.isRunning()) {
                zeroingTimer.restart();
                if (stowZero) {
                    pivotMotor.setControl(CollectorConstants.PIVOT_ZEROING_DC_STOW);
                } else {
                    pivotMotor.setControl(CollectorConstants.PIVOT_ZEROING_DC_DEPLOY);
                }
            } else if (!pivotMotor.getVelocity().getValue().isNear(RotationsPerSecond.zero(), RotationsPerSecond.of(0.1))) {
                zeroingTimer.reset();
            } else if (zeroingTimer.hasElapsed(CollectorConstants.PIVOT_ZERO_TIMER_THRESHOLD)) {
                pivotZeroed = true;
                if (stowZero) {
                    pivotMotor.setPosition(CollectorConstants.STOW_ANGLE);
                } else {
                    pivotMotor.setPosition(CollectorConstants.DEPLOY_ANGLE);
                }
                zeroingTimer.stop();
                setPivotAngle(targetPivotPosition);
            }
        }
        if (RobotState.isDisabled()) {
            pivotActive = false;
        }

        telemetry.log("Pivot Angle", getPivotAngle());
        telemetry.log("Pivot Target Angle", getTargetPivotAngle());
        telemetry.log("Pivot On Target", pivotOnTarget());
        telemetry.log("Collector Velocity", getCollectorVelocity());
        telemetry.log("Pivot Zeroed", pivotZeroed);
    }


    private void simulationPeriodic() {
        // collector sim stuff
        collectorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorSim.setInputVoltage(collectorMotorSim.getMotorVoltage());
        collectorSim.update(TimedRobot.DEFAULT_PERIOD);

        Angle collectorSimAngle = Radians.of(collectorSim.getAngularPosition());
        AngularVelocity collectorSimVelocity = RadiansPerSecond.of(collectorSim.getAngularVelocity());

        collectorMotorSim.setRawRotorPosition(collectorSimAngle.times(CollectorConstants.COLLECTOR_GEAR_RATIO));
        collectorMotorSim.setRotorVelocity(collectorSimVelocity.times(CollectorConstants.COLLECTOR_GEAR_RATIO));

        // pivot sim stuff
        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorPivotSim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        collectorPivotSim.update(TimedRobot.DEFAULT_PERIOD);

        Angle pivotSimAngle = Radians.of(collectorPivotSim.getAngle());
        AngularVelocity pivotSimVelocity = RadiansPerSecond.of(collectorPivotSim.getVelocity());

        pivotMotorSim.setRawRotorPosition(pivotSimAngle.times(CollectorConstants.ENCODER_TO_MECHANISM_RATIO));
        pivotMotorSim.setRotorVelocity(pivotSimVelocity.times(CollectorConstants.ENCODER_TO_MECHANISM_RATIO));
        ligament.setAngle(90 - pivotSimAngle.in(Degrees));

        telemetry.log("mech 2d", mech2d);
    }

    /**
     * Set the power of the collector motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setCollectorPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
    }

    /**
     * Starts deploying the pivot motor
     */
    public void deployPivot() {
        setPivotAngle(CollectorConstants.DEPLOY_ANGLE);
    }

    /**
     * Starts stowing the pivot motor
     */
    public void stowPivot() {
        setPivotAngle(CollectorConstants.STOW_ANGLE);
    }

    public void neutralPivot() {
        setPivotAngle(CollectorConstants.NEUTRAL_ANGLE);
    }

    /**
     * Stops all movement to the collector motor
     */
    public void stopCollector() {
        collectorMotor.stopMotor();
    }

    /**
     * Gets the current velocity of the collector motor.
     *
     * @return the collector motor velocity as an {@link AngularVelocity}
     */
    public AngularVelocity getCollectorVelocity() {
        return collectorMotor.getVelocity().getValue();
    }

    /**
     * Set the pivot position in degrees
     * @param position in degrees
     */
    public void setPivotAngle(Angle position) {
        targetPivotPosition = ThunderUnits.clamp(position, CollectorConstants.MIN_ANGLE, CollectorConstants.MAX_ANGLE);
        if (pivotZeroed) {
            pivotMotor.setControl(positionPID.withPosition(targetPivotPosition));
        }
        pivotActive = !targetPivotPosition.isEquivalent(CollectorConstants.STOW_ANGLE);
    }


    /**
     * gets the target position of the pivot as an {@link Angle}
     * @return the target position of the pivot
     */
    public Angle getTargetPivotAngle() {
        return targetPivotPosition;
    }

     /**
     * Get the angle of the pivot
     * @return angle of the pivot
     */
    public Angle getPivotAngle(){
        return pivotMotor.getPosition().getValue();
    }


    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean pivotOnTarget() {
        return targetPivotPosition.isNear(getPivotAngle(), CollectorConstants.TOLERANCE);
    }

    public boolean isDeployed() {
        return pivotOnTarget() && targetPivotPosition.isEquivalent(CollectorConstants.DEPLOY_ANGLE);
    }

    public boolean isStowed() {
        return pivotOnTarget() && targetPivotPosition.isEquivalent(CollectorConstants.STOW_ANGLE);
    }

    public Command deployPivotCommand() {
        return run(co -> deployPivot()).named("Collector Deploy");
    }

    public Command stowPivotCommand() {
        return run(co -> stowPivot()).named("Collector Stow");
    }

    // This command must not require the collector mechanism so it doesn't interrupt smart shoot
    public Command driverStowPivotCommand() {
        return Command.noRequirements(co -> {
            while (true) {
                stowPivot();
                co.yield();
            }
        }).whenCanceled(() -> deployPivot()).named("Collector Driver Stow");
    }

    public Command neutralPivotCommand() {
        return run(co -> {
            if (pivotActive) {
                neutralPivot();
            }
            co.park();
        }).named("Collector Neutral");
    }

    /**
     * Deploys the pivot then runs the rollers until canceled. The raised priority replaces v2's
     * InterruptionBehavior.kCancelIncoming: lower priority collector commands can't interrupt it.
     */
    public Command collectCommand(DoubleSupplier power) {
        return run(co -> {
            deployPivot();
            while (true) {
                setCollectorPower(power.getAsDouble());
                co.yield();
            }
        }).whenCanceled(() -> stopCollector()).withPriority(Command.DEFAULT_PRIORITY + 1).named("Collect");
    }

    public Command runCollectorWheels(DoubleSupplier power){
        return Command.noRequirements(co -> setCollectorPower(power.getAsDouble())).named("Collector Wheels");
    }
}

