package frc.robot.constants;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    // SystemCore has built-in CAN ports instead of a CANivore. Oasis support was dropped in the port.
    public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

    public static final int COLLECTOR = 9;
    public static final int COLLECTOR_PIVOT = 10;

    public static final int SPINDEXER = 11;
    public static final int TRANSFER = 12;
    public static final int UPDEXER = 20;

    public static final int TURRET = 13;
    public static final int TURRET_ZERO_SWITCH = 0;
    public static final int TURRET_MAX_SWITCH = 1;
    public static final int HOOD = 14;
    public static final int HOOD_ENCODER = 36;
    public static final int SHOOTER_LEFT = 15;
    public static final int SHOOTER_RIGHT = 16;

    public static final int CLIMBER = 17;
    public static final int CLIMBER_ENCODER = 37;

    public static final int DRIVER_PORT = 0;
    public static final int COPILOT_PORT = 1;

    public static final int CLIMBER_FORWARD_LIMIT_SWITCH = 2;
    public static final int CLIMBER_REVERSE_LIMIT_SWITCH = 3;
}
