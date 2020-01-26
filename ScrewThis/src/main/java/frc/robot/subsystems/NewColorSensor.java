/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import frc.robot.commands.ColorSense;

// Color Sensor subsystem used to detect colors on the color wheel
public class NewColorSensor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Port for the color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // Color sensor object
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // Color match object used to process colors
  private final ColorMatch m_colorMatcher = new ColorMatch();

  // Different color IDs
  private static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
  private static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
  private static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
  private static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

  // Motor used to spin the wheel
  TalonSRX spinner;

  // Int to track how many times we've seen the same color
  int rotationControlIndex;

  // Constructor for ColorSensor
  public NewColorSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    spinner = new TalonSRX(RobotMap.TALON_SPINNER_ID);

    rotationControlIndex = 0;
  }

  // Processes color and compares it to preset color IDs
  public String getColorString() {
    // Gets raw color
    Color detectedColor = m_colorSensor.getColor();
    // Processes the color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // Checks if the processed color matches one of the preset
    if (match.color == kYellowTarget)
      return "Yellow";
    if (match.color == kRedTarget)
      return "Red";
    if (match.color == kGreenTarget)
      return "Green";
    if (match.color == kBlueTarget)
      return "Blue";
    return "None";
  }

  // TODO: Create method that rotates properly for rotation control
  public boolean rotationControl() {
    spinner.set(ControlMode.PercentOutput, 1); // TODO: Pick a better motor power value
    if (getColorString() == "Yellow")
      rotationControlIndex++;
    return rotationControlIndex > 4;
  }

  // TODO: Create method that rotates wheel and returns true when proper color is
  // found
  public boolean colorControl(String desiredColor) {
    spinner.set(ControlMode.PercentOutput, 1); // TODO: Pick a better motor power value
    return getColorString() == desiredColor;

  }

  // Sets the command that will call these methods
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ColorSense());
  }
}
