/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
/**
 * This subsystem is used to control the color wheel with rotation control and
 * color control.
 */

public class ColorSense extends CommandBase {

  private final ColorSensor m_colorSensor;
  private final BooleanSupplier startRotationControl;
  private final BooleanSupplier startColorControl;
  private final BooleanSupplier selectNextColor;

  String desiredColor;
  int desiredColorNum = 0;
  int selectedColor = 0;
  int desiredRotations = 4;
  boolean rotationTime;
  boolean colorTime;

  SendableChooser<Integer> colorSelector;

  /**
   * Creates a new ColorSense.
   */
  public ColorSense(ColorSensor colorSensor, BooleanSupplier startRotationControl, BooleanSupplier startColorControl,
      BooleanSupplier selectNextColor) {
    this.m_colorSensor = colorSensor;
    this.startRotationControl = startRotationControl;
    this.startColorControl = startColorControl;
    this.selectNextColor = selectNextColor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorSensor);

    desiredColor = m_colorSensor.intToColorString(desiredColorNum);

    colorSelector = new SendableChooser<Integer>();

    colorSelector.setDefaultOption("Red", 0);
    colorSelector.addOption("Yeetlow", 1);
    colorSelector.addOption("Blue", 2);
    colorSelector.addOption("Green", 3);
    
    

    SmartDashboard.putData("Selected Color 1", colorSelector);
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
    // The almighty ternerary
    // // colorTime = !(startColorControl.getAsBoolean() && !rotationTime) ? !true ?
    // false:true : !false ? true:false;

    // Only starts the color control command if the rotation control command is not
    // currently running
    if (startColorControl.getAsBoolean() && !rotationTime) {
      colorTime = !colorTime;
    }

    // Only starts the rotation control command if the color control command is not
    // currently running
    if (startRotationControl.getAsBoolean() && !colorTime) {
      rotationTime = !rotationTime;
    }

    if (rotationTime) {
      rotationTime = !m_colorSensor.rotate(desiredRotations);
    } else if (colorTime) {
      colorTime = !m_colorSensor.goToColor(desiredColorNum);
    } else {
      // Turns everything off
      m_colorSensor.stopAll();
    }

    updateSelectedColor();
  }

  /**
   * Checks if the the next color button has been pressed, and if so updates it
   */
  private void updateSelectedColor() {
    // if (selectNextColor.getAsBoolean()) {
    //   desiredColorNum = (desiredColorNum + 1) % 4;
    //   desiredColor = m_colorSensor.intToColorString(desiredColorNum);
    // }

    desiredColorNum = colorSelector.getSelected();
    // desiredColorNum = colorSelector.getSelected() != null ? colorSelector.getSelected() : 0;
    desiredColor = m_colorSensor.intToColorString(desiredColorNum);

    // SmartDashboard.putString("Selected Color", desiredColor);
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
}
