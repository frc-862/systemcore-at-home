// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.*;
import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TestSubsystem extends SubsystemBase {

  // setup an motor
  private final TalonFX motor = new TalonFX(1, new CANBus(CANPort.CAN_S0));
  private DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(false);
  private final double CLOCKWISE = 0.3;
  private final double COUNTERCLOCKWISE = -0.3;

  public TestSubsystem() {
    configureMotors();
  }

  private void configureMotors() {
    var configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configuration.CurrentLimits.StatorCurrentLimit = 40;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = 40;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(configuration);
  }

  public Command runClockwise() {
    return Commands.startEnd(() -> {
      motor.setControl(request.withOutput(CLOCKWISE));
    }, () -> {
      stop();
    });
  }

  public Command runCounterClockwise() {
    return Commands.run(() -> {
      motor.setControl(request.withOutput(COUNTERCLOCKWISE));
    });
  }

  public Command stopMotors() {
    return Commands.run(() -> {
      stop();
    });
  }

  public void stop() {
    motor.stopMotor();
  }

}
