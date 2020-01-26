/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

// Subsystem containing all methods relating to driving
public class Drivetrain extends Subsystem {
  /**
   * Creates a new Drivetrain.
   */

  // Talon motor controllers
  TalonSRX tLF;
  TalonSRX tLB;
  TalonSRX tRF;
  TalonSRX tRB;

  //Constructor
  public Drivetrain() {

    tLF = new TalonSRX(RobotMap.TALON_LEFT_FRONT_ID);
    tLB = new TalonSRX(RobotMap.TALON_LEFT_BACK_ID);
    tRF = new TalonSRX(RobotMap.TALON_RIGHT_FRONT_ID);
    tRB = new TalonSRX(RobotMap.TALON_RIGHT_BACK_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets the speed of the motors
  public void setPower(double leftPower, double rightPower) {
    tLF.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * leftPower);
    tLB.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * leftPower);

    tRF.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * rightPower);
    tRB.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * rightPower);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
    setDefaultCommand(new Drive());
  }
}
