// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuatorSubsystem extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(16);
  private final TalonFX rightMotor = new TalonFX(17);

  /** Creates a new LinearActuatorSubsystem. */
  public LinearActuatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("linActuatorLeftMotorRPM", (leftMotor.getSelectedSensorVelocity() / 2048) * 10 * 60);
    SmartDashboard.putNumber("linActuatorRightMotorRPM", (rightMotor.getSelectedSensorVelocity() / 2048) * 10 * 60);

    SmartDashboard.putNumber("linActuatorMotorPositionDiff", getDiff());
  }

  private double getDiff() {
    double leftPos = leftMotor.getSelectedSensorPosition();
    double rightPos = rightMotor.getSelectedSensorPosition();

    return (leftPos - rightPos)/2048;
  }

  private Boolean checkAlignment() {
    if(Math.abs(getDiff()) > 10) {
      return false;
    }

    return true;
  }

  public void setSpeed(double percentSpeed) {
    if(!checkAlignment()) {
      //leftMotor.set(ControlMode.PercentOutput, 0);
      //rightMotor.set(ControlMode.PercentOutput, 0);
      return;
    }

    leftMotor.set(ControlMode.PercentOutput, percentSpeed);
    rightMotor.set(ControlMode.PercentOutput, percentSpeed + getDiff()/SmartDashboard.getNumber("diffDivisor", 5));
  }
}
