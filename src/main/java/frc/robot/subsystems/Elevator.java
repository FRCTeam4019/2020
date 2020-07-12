/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem controls the elevation system for the hangar
 */

public class Elevator extends SubsystemBase {
  private final TalonSRX leftMotor;
  private final TalonSRX rightMotor;

  private Servo rightLockServo = new Servo(Constants.Motors.IDs.rightElevatorLock);
  private Servo leftLockServo = new Servo(Constants.Motors.IDs.leftElevatorLock);

  private final DigitalInput climberTopSwitch = new DigitalInput(Constants.Switches.climberTopSwitch);
  /**
   * Creates a new Elevator.
   */
  public Elevator() {

    leftMotor = new TalonSRX(Constants.Motors.IDs.leftElevator);
    leftMotor.setInverted(Constants.Motors.Inversions.leftElevator);
    rightMotor = new TalonSRX(Constants.Motors.IDs.rightElevator);
    rightMotor.setInverted(Constants.Motors.Inversions.rightElevator);

    engageLock();
  }

  public void setPower(double power) {
    leftMotor.set(ControlMode.PercentOutput, power);
    rightMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Raises the elevator;
   */
  public void up() {
    if (climberTopSwitch.get())
      setPower(Constants.Evelator.upSpeed);
    else
      stop();
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

  public void engageLock() {
      // rightLockServo.set(Constants.Evelator.rightLockLockedAngle);
      // leftLockServo.set(Constants.Evelator.leftLockLockedAngle);
      leftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void disengageLock() {
    // rightLockServo.set(Constants.Evelator.rightLockUnlockedAngle);
    // leftLockServo.set(Constants.Evelator.leftLockUnlockedAngle);
    // leftMotor.setNeutralMode(NeutralMode.);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Limit", climberTopSwitch.get());
  }
}
