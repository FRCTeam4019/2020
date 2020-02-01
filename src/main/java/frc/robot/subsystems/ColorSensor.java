/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
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

  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(Constants.Colors.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.Colors.kYellowTarget);
  }

  public String getColorString() {
    // Gets raw color
    Color detectedColor = m_colorSensor.getColor();
    // Processes the color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // Checks if the processed color matches one of the preset
    if (match.color == Constants.Colors.kYellowTarget)
      return "Yellow";
    if (match.color == Constants.Colors.kRedTarget)
      return "Red";
    if (match.color == Constants.Colors.kGreenTarget)
      return "Green";
    if (match.color == Constants.Colors.kBlueTarget)
      return "Blue";
    return "None";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
