/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static abstract class Autonomous {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final int[] kRightEncoderPorts = { 0, 1 };
        public static final int[] kLeftEncoderPorts = { 2, 3 };

        public static final boolean kRightEncoderReversed = false;
        public static final boolean kLeftEncoderReversed = false;

        public static final double kEncoderDistancePerPulse = 100;
        public static final boolean kGyroReversed = false;

        public static final double autoDriveSpeed = 0.0;
        public static final double autoTurnRate = 0.002;

        public static final String trajectoryJSON = "paths/YourPath.wpilib.json";
    }

    public static abstract class Talons {
        public static abstract class IDs {
            public static final int leftFront = 0;
            public static final int leftBack = 1;
            public static final int rightFront = 2;
            public static final int rightBack = 3;
            public static final int spinner = 4;
            public static final int topConveyor = 5;
            public static final int bottomConveyor = 6;
            public static final int topIntake = 7;
            public static final int bottomIntake = 8;
            public static final int shooter = 9;
        }
        
        public static abstract class Inversions{
            public static final boolean leftFront = true;
            public static final boolean leftBack = true;
            public static final boolean rightFront = false;
            public static final boolean rightBack = false;
        }
    }

    public static abstract class Controls {
        public static abstract class ButtonIDs{
            public static final int intake = 5;
            public static final int shoot = 6;
            public static final int rotate = 7;
            public static final int color = 8;
            public static final int nextColor = 12;
            public static final int autoAlign = 2;
        }

        public static final double driveThrottle = 0.5;

        public static final int driveAxisForward = 1;
        public static final int driveAxisRotate = 0;
        public static final int driveAxisThrottle = 3;
    }

    public static abstract class Colors {
        // Different color IDs
        public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
        public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);
    }

    public static abstract class Vision {
        public static final int[] camSize = { 320, 240 };
        public static final int alignmentOffset = 30;
    }

    public static abstract class Ultrasonics {
        public static final int ultrasonic1Port = 0;
        public static final int ultrasonic2Port = 1;

        // public static final double distanceMultiplier = 5.0/(5.0/1024.0)*6;
        public static final double distanceMultiplier = 500;
        public static final double distanceOffset = 0;

        public static final double maxRange = 500;
        public static final double minRange = 30;


        //5 cm: 0.058
        //30 cm: 0.06
        // 30 min distance
    }

    public static final int spinnerSpeed = 100;

}
