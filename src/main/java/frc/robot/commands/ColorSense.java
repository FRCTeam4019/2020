/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  int desiredRotations = 4;
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

    desiredColor = getSelectedColorString(desiredColorNum);


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
    // rotationTime = startRotationControl.getAsBoolean() && !colorTime;
    // // colorTime = !(startColorControl.getAsBoolean() && !rotationTime) ? !true ? false:true : !false ? true:false;
    // colorTime = startColorControl.getAsBoolean() && !rotationTime;

    if(startColorControl.getAsBoolean() && !rotationTime) {
      colorTime = !colorTime;
    }

    if(startRotationControl.getAsBoolean() && !colorTime) {
      rotationTime = !rotationTime;
      m_colorSensor.resetIndex();
    }

    if (rotationTime) {
      
      m_colorSensor.setRotationControl(true);
      m_spinner.setPower(Constants.spinnerSpeed);

      if(m_colorSensor.getTotalRotations() >= desiredRotations) {
        rotationTime = false;
      }
    } else if (colorTime) {
      m_spinner.setPower(Constants.spinnerSpeed);
      colorTime = !m_colorSensor.checkColorMatch(getSelectedColorString((desiredColorNum + 3) % 4));
    } else {
      m_colorSensor.setRotationControl(false);
      m_spinner.setPower(0);
    }

    SmartDashboard.putNumber("Number of Rotations", m_colorSensor.getTotalRotations());

    updateSelectedColor();

    // checkSwitchColor();
  }

  void updateSelectedColor() {
    if (selectNextColor.getAsBoolean()) {
      desiredColorNum = (desiredColorNum + 1) % 4;
      desiredColor = getSelectedColorString(desiredColorNum);
    }

    SmartDashboard.putString("Selected Color", desiredColor);
  }

  private String getSelectedColorString(int num) {
    if (num == 0)
      return "Red";
    else if (num == 1)
      return "Yellow";
    else if (num == 2)
      return "Blue";
    else if (num == 3)
      return "Green";
    return "None";
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
