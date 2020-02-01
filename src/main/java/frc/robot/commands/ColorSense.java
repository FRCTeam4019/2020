/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorSense extends CommandBase {
  String desiredColor;
  int desiredColorNum = 0;
  int selectedColor = 0;
  boolean rotationTime;
  boolean colorTime;

  /**
   * Creates a new ColorSense.
   */
  public ColorSense() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationTime = false;
    colorTime = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //checkSwitchColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void checkStartCommand() {

  }

  /* private void checkSwitchColor() {
    if (Robot.m_leftStick.getRawButton(RobotMap.NEXT_COLOR_BUTTON_ID)) {
      selectedColor++;
      System.out.print("Selection changed: " + selectedColor);

      if (selectedColor == 0) {
        SmartDashboard.putString("Selected Color", "Blue");
      }

      if (selectedColor == 1) {
        SmartDashboard.putString("Selected Color", "Green");
      }

      if (selectedColor == 2) {
        SmartDashboard.putString("Selected Color", "Red");
      }

      if (selectedColor == 3) {
        SmartDashboard.putString("Selected Color", "Yellow");
      }

      // desiredColorNum = Robot.m_colorsensor.getActualColor(selectedColor);
    } 
  } */
}
