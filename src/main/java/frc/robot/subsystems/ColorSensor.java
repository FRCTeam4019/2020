/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {

  // Port for the color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // Color sensor object
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // Color match object used to process colors
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private int rotationControlIndex;

  private TalonSRX spinner;

  private Boolean prevRed = false;

  private Boolean rotation_control = false;

  private String currentColor = "";


  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(Constants.Colors.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kYellowTarget);

    updateColorString();

    // spinner = new TalonSRX(Constants.Talons.IDs.spinner);
  }

  /**
   * Gets the current color that the sensor is reading
   * @return The current color; either red, green, blue, or yellow
   */
  public String getColorString() {
    return currentColor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(rotation_control) {
      checkRotation();
    }
    updateColorString();
  
    // SmartDashboard.putNumber("Number of Rotations", rotationControlIndex);
  }

  private void updateColorString() {
    Color detectedColor = m_colorSensor.getColor();
    //  System.out.println(String.format("Detected Color: %s %s %s", detectedColor.red, detectedColor.green, detectedColor.blue));
    // Processes the color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // System.out.println(String.format("Matched Color: %s %s %s", match.color.red, match.color.green, match.color.blue));
    // Checks if the processed color matches one of the preset
    if (match.color.equals(Constants.Colors.kYellowTarget))
      currentColor = "Yellow";
    else if (match.color.equals(Constants.Colors.kRedTarget))
      currentColor = "Red";
    else if (match.color.equals(Constants.Colors.kGreenTarget))
      currentColor = "Green";
    else if (match.color.equals(Constants.Colors.kBlueTarget))
      currentColor = "Blue";
    else currentColor = "None";

    SmartDashboard.putString("Current Color", currentColor);
  }
 
  
  /**
   * Resets the rotation control index
   */
  public void resetIndex() {
    rotationControlIndex = 0;
  }

  /**
   * @return The total number of rotations that have been seen by the color sensor
   */
  public int getTotalRotations() {
    return rotationControlIndex/2;
  }

  public void setRotationControl(Boolean value) {
    rotation_control = value;
  }

  private void checkRotation() {
    if(!prevRed && checkColorMatch("Red")) {
      prevRed = true;
      rotationControlIndex++;

    } else if(!checkColorMatch("Red")) {
      prevRed = false;
    }
  }

    /**
   * Checks whether the color matches what the sensor sees
   * @param desiredColor The color to check for; red, greed, blue, yellow (not case sensitive)
   * @return Whether or not the colors match 
   */
  public Boolean checkColorMatch(String desiredColor){
    return currentColor.equalsIgnoreCase(desiredColor);
  }

}
