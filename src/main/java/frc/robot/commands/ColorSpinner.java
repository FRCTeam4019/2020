/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ColorSensor;

public class ColorSpinner extends Command {
  ColorSensor color_sensor;
  int target_color = 0;
  int selected_color = 0;
  boolean finished = false;
  TalonSRX spinner;

  

  public ColorSpinner(int target_color1) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    target_color = getActualColor(target_color1);
    color_sensor = new ColorSensor();
    spinner = new TalonSRX(RobotMap.TALON_SPINNER_ID);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    spinner.set(ControlMode.PercentOutput, RobotMap.SPINNER_SPEED);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(target_color == 0 && color_sensor.isBlue()) {
      finished = true;
    }

    if(target_color == 1 && color_sensor.isGreen()) {
      finished = true;
    }

    if(target_color == 2 && color_sensor.isRed()) {
      finished = true;
    }

    if(target_color == 3 && color_sensor.isYellow()) {
      finished = true;
    }

    if(Robot.m_leftStick.getRawButton(RobotMap.NEXT_COLOR_BUTTON_ID)) {
      selected_color++;
      System.out.print("Selection changed: " + selected_color);

      if(selected_color == 0) {
        SmartDashboard.putString("Selected Color", "Blue");
      }

      if(selected_color == 1) {
        SmartDashboard.putString("Selected Color", "Green");
      }

      if(selected_color == 2) {
        SmartDashboard.putString("Selected Color", "Red");
      }

      if(selected_color == 3) {
        SmartDashboard.putString("Selected Color", "Yellow");
      }

      target_color = getActualColor(selected_color);
    }
    
  }

  private int getActualColor(int color) {
    return color += 2;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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