/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Spinner;

public class ColorSense extends CommandBase {

  private final ColorSensor m_colorSensor;
  private final Spinner m_spinner;
  private final BooleanSupplier startRotationControl;
  private final BooleanSupplier startColorControl;
  private final BooleanSupplier selectNextColor;

  String desiredColor;
  int desiredColorNum = 0;
  int selectedColor = 0;
  int desiredRotations = 0;
  boolean rotationTime;
  boolean colorTime;

  /**
   * Creates a new ColorSense.
   */
  public ColorSense(ColorSensor colorSensor, Spinner spinner, BooleanSupplier startRotationControl, BooleanSupplier startColorControl,
      BooleanSupplier selectNextColor) {
    this.m_colorSensor = colorSensor;
    this.m_spinner = spinner;
    this.startRotationControl = startRotationControl;
    this.startColorControl = startColorControl;
    this.selectNextColor = selectNextColor;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements();
    addRequirements(colorSensor);
    addRequirements(spinner);

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
    rotationTime = startRotationControl.getAsBoolean() && !colorTime;
    colorTime = !(startColorControl.getAsBoolean() && !rotationTime) ? false : true;

    if (rotationTime) {
      m_colorSensor.setRotationControl(true);
      m_spinner.setPower(50);

      if(m_colorSensor.getTotalRotations() >= desiredRotations) {
        rotationTime = false;
      }
    } else if (colorTime) {
      m_spinner.setPower(50);
      colorTime = !m_colorSensor.checkColorMatch(desiredColor);
    } else {
      m_colorSensor.setRotationControl(false);
      m_spinner.setPower(0);
    }

    if (selectNextColor.getAsBoolean()) {
      desiredColorNum = (desiredColorNum + 1) % 4;
      if (desiredColorNum == 0)
        desiredColor = "Red";
      else if (desiredColorNum == 1)
        desiredColor = "Yellow";
      else if (desiredColorNum == 2)
        desiredColor = "Green";
      else if (desiredColorNum == 3)
        desiredColor = "Blue";
      SmartDashboard.putString("Selected Color", desiredColor);
    }
    // checkSwitchColor();
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

  /*
   * private void checkSwitchColor() { if
   * (Robot.m_leftStick.getRawButton(RobotMap.NEXT_COLOR_BUTTON_ID)) {
   * selectedColor++; System.out.print("Selection changed: " + selectedColor);
   * 
   * if (selectedColor == 0) { SmartDashboard.putString("Selected Color", "Blue");
   * }
   * 
   * if (selectedColor == 1) { SmartDashboard.putString("Selected Color",
   * "Green"); }
   * 
   * if (selectedColor == 2) { SmartDashboard.putString("Selected Color", "Red");
   * }
   * 
   * if (selectedColor == 3) { SmartDashboard.putString("Selected Color",
   * "Yellow"); }
   * 
   * // desiredColorNum = Robot.m_colorsensor.getActualColor(selectedColor); } }
   */
}
