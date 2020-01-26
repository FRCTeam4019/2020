/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends Command {
  /**
   * Creates a new Drive.
   */

  public Drive() {
    requires(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets input from Joystick
    double forward = Robot.m_leftStick.getRawAxis(RobotMap.DRIVE_AXIS_FORWARD);
    double rotate = Robot.m_leftStick.getRawAxis(RobotMap.DRIVE_AXIS_ROTATION);
    double throttle = Robot.m_leftStick.getRawAxis(RobotMap.DRIVE_AXIS_THROTTLE) / -2 + 0.5;

    // Sends appropiate power to drivetrain who in turn sets the speed of the motor.
    // System.out.println("Drive execute");
    Robot.m_drivetrain.setPower(throttle * (forward - rotate), throttle * (forward + rotate));
  }

  // Called once the command ends or is interrupted.
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
