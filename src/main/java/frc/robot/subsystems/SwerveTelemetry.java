package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * Logs the swerve drive state. Previously split across NetworkTables publishers, SmartDashboard
 * and CTRE SignalLogger; now everything goes through one TelemetryTable.
 */
public class SwerveTelemetry {
    private final double MaxSpeed;

    private final TelemetryTable table = Telemetry.getTable("DriveState");

    /* Robot pose for field positioning */
    private final Field2d field = new Field2d();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
    };

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public SwerveTelemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
    }

    /** Accept the swerve drive state and log it.
     *
     * <p>This is called from CTRE's odometry thread, not the main robot loop.
     *
     * @param state The swerve drive state to log
    */
    public void telemeterize(SwerveDriveState state) {
        /* Struct-serializable types are logged directly */
        table.log("Pose", state.Pose);
        table.log("Velocity", state.Velocity);
        table.log("ModuleVelocities", state.ModuleVelocities, SwerveModuleVelocity.struct);
        table.log("ModuleTargets", state.ModuleTargets, SwerveModuleVelocity.struct);
        table.log("OdometryPeriod", state.OdometryPeriod);

        /* Field2d and Mechanism2d are TelemetryLoggables */
        field.setRobotPose(state.Pose);
        table.log("Field", field);

        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleVelocities[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleVelocities[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleVelocities[i].velocity / (2 * MaxSpeed));
            table.log("Module " + i, m_moduleMechanisms[i]);
        }
    }
}
