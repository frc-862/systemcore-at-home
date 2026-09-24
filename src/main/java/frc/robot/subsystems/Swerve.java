package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MirageTunerConstants.TunerSwerveDrivetrain;
import frc.util.AllianceHelpers;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 *
 * <p>Dropped from the 2026 version: PathPlanner (no 2027 release), PhotonVision measurements
 * and maple-sim simulation.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.ZERO;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with Telemetry (was SignalLogger)
            state -> Telemetry.log("SysId/Translation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with Telemetry (was SignalLogger)
            state -> Telemetry.log("SysId/Steer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with Telemetry (was SignalLogger)
            state -> Telemetry.log("SysId/Rotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                Telemetry.log("SysId/Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            MatchState.getAlliance().ifPresent(allianceColor -> {
                setOperatorForwardDirection(
                    allianceColor == Alliance.RED
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        Pose2d pose = getPose();
        if (!FieldConstants.ROBOT_COMPENSATED_FIELD.contains(pose.getTranslation())) {
            resetPose(new Pose2d(FieldConstants.ROBOT_COMPENSATED_FIELD.nearest(pose.getTranslation()), pose.getRotation()));
        }
    }

    private void startSimThread() {
        m_lastSimTime = RobotController.getTime() / 1e6;

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = RobotController.getTime() / 1e6;
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(DriveConstants.kSimLoopPeriod.in(Seconds));
    }

    /**
     * Use for autonomous driving ex. auto  align, auton
     * Use whenever you want a specific velocity with closed loop control
     * Always uses blue alliace perspective
     * @param xInput input for the x velocity
     * @param yInput input for the x velocity
     * @param rInput input for the x velocity
     * @return command to run
     */
    public Command autoDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput) {
        return run(() -> setControl(DriveConstants.fieldCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xInput.getAsDouble()))
            .withVelocityY(DriveConstants.MaxSpeed.times(yInput.getAsDouble()))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)));
    }

    public Command driveCommand(Supplier<Vector<N2>> xyInput, DoubleSupplier rInput) {
        return run(() -> {
            var xy = xyInput.get();
            setControl(DriveConstants.fieldCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xy.get(0)))
            .withVelocityY(DriveConstants.MaxSpeed.times(xy.get(1)))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        });
    }

    public Command robotCentricDrive(Supplier<Vector<N2>> xyInput, DoubleSupplier rInput){
        return run(() -> {
            var xy = xyInput.get();
            setControl(DriveConstants.robotCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xy.get(0)))
            .withVelocityY(DriveConstants.MaxSpeed.times(xy.get(1)))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        });
    }

    public Command brakeCommand() {
        return applyRequest(() -> DriveConstants.brakeRequest);
    }

    public Command resetFieldCentricCommand() {
        return new InstantCommand(() -> seedFieldCentric());
    }

    public Pose2d getPose(){
        return getState().Pose;
    }

    public boolean isNearTrench() {
        Translation2d shooterTranslation2d = getPose().getTranslation();
        boolean isNearTrench = FieldConstants.LEFT_BLUE_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.RIGHT_BLUE_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.LEFT_RED_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.RIGHT_RED_TRENCH.contains(shooterTranslation2d);
        return isNearTrench;
    }

    public ChassisVelocities getCurrentRobotChassisVelocities(){
        return getState().Velocity;
    }

    private static final double WALL_MARGIN = 0.5; // meters (~8 in, bumper width + tolerance)


    public ChassisVelocities getWallCorrectedChassisVelocities() {
        ChassisVelocities speeds = getCurrentRobotChassisVelocities();
        Pose2d pose = getPose();

        double sin = pose.getRotation().getSin();
        double cos = pose.getRotation().getCos();

        // Robot-relative to field-relative
        double fieldVx = speeds.vx * cos - speeds.vy * sin;
        double fieldVy = speeds.vx * sin + speeds.vy * cos;

        Translation2d pos = pose.getTranslation();

        // Clamp velocity component pointing out of the field near each wall
        if (pos.getX() < WALL_MARGIN && fieldVx < 0) {
            fieldVx = 0;
        }
        if (pos.getX() > FieldConstants.FIELD_LENGTH - WALL_MARGIN && fieldVx > 0) {
            fieldVx = 0;
        }
        if (pos.getY() < WALL_MARGIN && fieldVy < 0) {
            fieldVy = 0;
        }
        if (pos.getY() > FieldConstants.FIELD_WIDTH - WALL_MARGIN && fieldVy > 0) {
            fieldVy = 0;
        }

        // Field-relative back to robot-relative
        double rrVx =  fieldVx * cos + fieldVy * sin;
        double rrVy = -fieldVx * sin + fieldVy * cos;

        return new ChassisVelocities(rrVx, rrVy, speeds.omega);
    }

    /* Checks if the robot's pose is within the current alliance's zone
     *
     * @return true if the robot is in the zone, false otherwise
     */
    public boolean isInZone() {
        Pose2d robotPose = getPose();
        if (AllianceHelpers.isRedAlliance()) {
            return (FieldConstants.RED_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        } else {
            return (FieldConstants.BLUE_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        }
    }

    @SuppressWarnings("resource")
    public Command autoAlign(Pose2d targetPose) {
        PIDController pidX = new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D);
        PIDController pidY = new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D);
        PIDController pidR = new PIDController(DriveConstants.ROT_P, DriveConstants.ROT_I, DriveConstants.ROT_D);

        pidR.enableContinuousInput(-180, 180);

        pidX.setTolerance(DriveConstants.DRIVE_TOLERANCE.in(Meters));
        pidY.setTolerance(DriveConstants.DRIVE_TOLERANCE.in(Meters));
        pidR.setTolerance(DriveConstants.ROT_TOLERANCE.in(Degrees));

        return autoDrive(() -> Math.clamp(pidX.calculate(getPose().getX(), targetPose.getX()), -1, 1),
        () -> Math.clamp(pidY.calculate(getPose().getY(), targetPose.getY()), -1, 1),
        () -> pidR.calculate(getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees()));
    }

    public Command lowerSupplyLimits() {
        var modules = getModules();
        var savedConfigs = new CurrentLimitsConfigs[modules.length];

        return new StartEndCommand(() -> {
            for (int i = 0; i < modules.length; i++) {
                var config = new CurrentLimitsConfigs();

                var configurator = modules[i].getDriveMotor().getConfigurator();
                configurator.refresh(config);
                savedConfigs[i] = config.clone();

                configurator.apply(config
                    .withSupplyCurrentLimit(DriveConstants.SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(DriveConstants.ENABLE_SUPPLY_CURRENT_LIMIT));
            }
        }, () -> {
            for (int i = 0; i < modules.length; i++) {
                modules[i].getDriveMotor().getConfigurator().apply(savedConfigs[i]);
            }
        });
    }

    public Command increaseRampRates() {
        var modules = getModules();
        var savedConfigs = new OpenLoopRampsConfigs[modules.length];

        return new StartEndCommand(() -> {
            for (int i = 0; i < modules.length; i++) {
                var config = new OpenLoopRampsConfigs();

                var configurator = modules[i].getDriveMotor().getConfigurator();
                configurator.refresh(config);
                savedConfigs[i] = config.clone();

                configurator.apply(config
                        .withVoltageOpenLoopRampPeriod(0.5)
                        .withDutyCycleOpenLoopRampPeriod(0.5)
                        .withTorqueOpenLoopRampPeriod(0.5));
            }
        }, () -> {
            for (int i = 0; i < modules.length; i++) {
                modules[i].getDriveMotor().getConfigurator().apply(savedConfigs[i]);
            }
        });
    }
}
