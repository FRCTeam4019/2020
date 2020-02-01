/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

// Command class corralated to the NewColorSensor subsystem. 
public class ColorSense extends Command {

  String desiredColor;
  int desiredColorNum = 0;
  int selectedColor = 0;
  boolean rotationTime;
  boolean colorTime;

  public ColorSense() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_colorsensor);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rotationTime = false;
    colorTime = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Get the color string and put it on the Shuffleboard.
    String currentColor = Robot.m_colorsensor.getColorString();
    SmartDashboard.putString("Color", currentColor);

    // Checks to see if button was pressed to activate rotation control
    if (Robot.m_leftStick.getRawButton(RobotMap.ROTATION_BUTTON_ID) && !colorTime)
      rotationTime = true;
    // Runs rotation control until it returns true
    if (rotationTime)
      rotationTime = !Robot.m_colorsensor.rotationControl();

    // Checks to see if button was pressed to activate color control
    if (Robot.m_leftStick.getRawButtonPressed(RobotMap.COLOR_BUTTON_BLUE_ID) && !rotationTime) {
      colorTime = true;
      desiredColor = "Blue";
    } else if (Robot.m_leftStick.getRawButtonPressed(RobotMap.COLOR_BUTTON_GREEN_ID) && !rotationTime) {
      colorTime = true;
      desiredColor = "Green";
    } else if (Robot.m_leftStick.getRawButtonPressed(RobotMap.COLOR_BUTTON_RED_ID) && !rotationTime) {
      colorTime = true;
      desiredColor = "Red";
    } else if (Robot.m_leftStick.getRawButtonPressed(RobotMap.COLOR_BUTTON_YELLOW_ID) && !rotationTime) {
      colorTime = true;
      desiredColor = "Yellow";
    }

    // Runs color control until it returns true
    if (colorTime)
      colorTime = !Robot.m_colorsensor.colorControl(desiredColor);
    if (Robot.m_leftStick.getRawButtonPressed(RobotMap.NEXT_COLOR_BUTTON_ID)) {
      selectedColor = (selectedColor + 1) % 4;
      System.out.print("Selection changed: " + selectedColor);

      if (selectedColor == 0) {
        desiredColor = "Blue";
      }

      if (selectedColor == 1) {
        desiredColor = "Green";
      }

      if (selectedColor == 2) {
        desiredColor = "Red";
      }

      if (selectedColor == 3) {
        desiredColor = "Yellow";
      }

      SmartDashboard.putString("Selected Color", desiredColor);

      // desiredColorNum = Robot.m_colorsensor.getActualColor(selectedColor);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
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
