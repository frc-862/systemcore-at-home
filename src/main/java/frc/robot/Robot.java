// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.DataLogManager;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private static RobotContainer robotContainer;

    public Robot() {
        // Record NetworkTables (which carries Telemetry by default) to a log file.
        // The LiveWindow, the NetworkTables enable switch and the DriverStation joystick log are gone.
        DataLogManager.start();

        getContainer();
    }

    public static RobotContainer getContainer() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }

        return robotContainer;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void utilityInit() {
        // Test mode is now called utility mode
        CommandScheduler.getInstance().cancelAll();
    }
}
