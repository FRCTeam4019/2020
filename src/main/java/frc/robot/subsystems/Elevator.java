/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonSRX leftMotor;
  private final TalonSRX rightMotor;
  /**
   * Creates a new Elevator.
   */
  public Elevator() {

    leftMotor = new TalonSRX(Constants.Talons.IDs.leftElevator);
    rightMotor = new TalonSRX(Constants.Talons.IDs.rightElevator);
  }

  public void setPower(double power) {
    leftMotor.set(ControlMode.PercentOutput, power);
    rightMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Raises the elevator;
   */
  public void up() {
    setPower(Constants.Evelator.upSpeed);
  }

  /**
  * Lowers the elevator;
  */
  public void down() {
    setPower(-Constants.Evelator.downSpeed);
  }

  /**
   * Stops the elevator;
   */
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
