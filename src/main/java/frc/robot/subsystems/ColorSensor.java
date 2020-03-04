/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is the color sensor, used for the color wheel
 */

public class ColorSensor extends SubsystemBase {

  // Port for the color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // Color sensor object
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // Color match object used to process colors
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private int rotationControlIndex;

  private Boolean prevRed = false;

  private Boolean rotation_control = false;

  private String currentColor = "";

  private final TalonSRX spinner;

  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(Constants.Colors.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kYellowTarget);

    spinner = new TalonSRX(Constants.Motors.IDs.spinner);
    spinner.setInverted(Constants.Motors.Inversions.spinner);



    updateColorString();
  }

  /**
   * Rotates the specified number of rotations
   * 
   * @param desiredRotations The desired number of rotations
   * @return Whether or not the number of rotations has been met
   */
  public boolean rotate(int desiredRotations) {
    rotation_control = true;
    if (rotationControlIndex <= desiredRotations) {
      startSpinner();
      return false;
    }

    stopSpinner();
    rotationControlIndex = 0;
    rotation_control = false;

    return true;
  }

  /**
   * Goes to the specified color
   * 
   * @param desiredColorNum The desired color number
   * @return Whether or not the desired color has been reached
   */
  public Boolean goToColor(int desiredColorNum) {
    // Checks if the color number matches what the field color sensor is seeing
    String color = intToColorString((desiredColorNum + Constants.Spinner.spinnerColorOffset) % 4);
    if (!checkColorMatch(color)) {
      startSpinner();
      return false;
    }
    
    stopSpinner();

    return true;
  }

  /**
   * Stops the spinner, turns off rotation control, and resets the rotation
   * control index
   */
  public void stopAll() {
    stopSpinner();
    rotation_control = false;
    rotationControlIndex = 0;
  }

  /**
   * Gets the current color that the sensor is reading
   * 
   * @return The current color; either red, green, blue, or yellow
   */
  public String getColorString() {
    return currentColor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (rotation_control) {
      checkRotation();
    }
    updateColorString();

    SmartDashboard.putNumber("Num Rot", rotationControlIndex / 2);
  }

  /**
   * Updates the current color string
   */

  private void updateColorString() {
    Color detectedColor = m_colorSensor.getColor();

    // Processes the color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // Checks if the processed color matches one of the preset
    if (match.color.equals(Constants.Colors.kYellowTarget))
      currentColor = "Yellow";
    else if (match.color.equals(Constants.Colors.kRedTarget))
      currentColor = "Red";
    else if (match.color.equals(Constants.Colors.kGreenTarget))
      currentColor = "Green";
    else if (match.color.equals(Constants.Colors.kBlueTarget))
      currentColor = "Blue";
    else
      currentColor = "None";

    SmartDashboard.putString("Current Color", currentColor);
  }

  /**
   * This function tracks how many times the sensor has passed red
   */
  private void checkRotation() {
    // The prevRed variable is enabled when the red is first seen, and then disabled
    // when it is not. this prevents it from seeing the same red marker multiple
    // times.
    if (!prevRed && checkColorMatch("Red")) {
      prevRed = true;
      rotationControlIndex++;

    } else if (!checkColorMatch("Red")) {
      prevRed = false;
    }
  }

  /**
   * Sets the spinner to the spinner speed constant
   */
  private void startSpinner() {
    spinner.set(ControlMode.PercentOutput, Constants.Spinner.spinnerSpeed);
  }

  /**
   * Stops the spinner
   */
  private void stopSpinner() {
    spinner.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Checks whether the color matches what the sensor sees
   * 
   * @param desiredColor The color to check for; red, greed, blue, yellow (not
   *                     case sensitive)
   * @return Whether or not the colors match
   */
  public Boolean checkColorMatch(String desiredColor) {
    return currentColor.equalsIgnoreCase(desiredColor);
  }

  /**
   * Gives you a color string value from a color number
   * <ul>
   * <li>0 = "Red"
   * <li>1 = "Yellow"
   * <li>2 = "Blue"
   * <li>3 = "Green"
   * <li>4+ = "None"
   * </ul>
   * 
   * @param num The color number
   * @return The color string
   */
  public String intToColorString(int num) {
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

}
