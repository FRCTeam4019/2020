package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.command.Command;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatchResult;

public class ColorSensor {
    public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
    public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    public ColorSensor() {
        m_colorMatcher.addColorMatch(kBlueTarget);

        m_colorMatcher.addColorMatch(kGreenTarget);

        m_colorMatcher.addColorMatch(kRedTarget);

        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public boolean isBlue() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            return true;
        }

        return false;

    }

    public boolean isGreen() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kGreenTarget) {
            return true;
        }

        return false;

    }

    public boolean isRed() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kRedTarget) {
            return true;
        }

        return false;

    }

    public boolean isYellow() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kYellowTarget) {
            return true;
        }

        return false;

    }

    public Color getColor() {
        return m_colorSensor.getColor();
    }

    //gives you the actual color the sensor needs to look for so that the field sensor sees the right color
    public int getActualColor(int color) {
        return color += 2;
    }

    public String getColoString() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kYellowTarget) {
            return "Yellow";
        }
        if (match.color == kRedTarget) {
            return "Red";
        }
        if (match.color == kGreenTarget) {
            return "Green";
        }
        if (match.color == kBlueTarget) {
            return "Blue";
        }

        return "None";
    }

}
