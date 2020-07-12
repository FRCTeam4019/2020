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

    /**
     * Data about the robot, along with other information
     */
    public static abstract class Stats {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.6;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.05;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 0.72;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.36;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kEncoderDistancePerPulse = 1.25/8.00;
        public static final boolean kGyroReversed = false;

    }

    /**
     * Values for use while in Autonomous mode
     */
    public static abstract class Autonomous {

        public static final double autoDriveSpeed = 0.5;
        public static final double autoTurnRateAddition = 0.2;
        public static final double autoTurnRate = 0.002;
        public static final double driveForwardDistance = 0.69 * 2;
        public static final double eStopDistance = 0;
        public static final double encoderDiffMultiplier = 0.3;

        public static abstract class TrajectoryPaths {
            public static final String driveAndShoot1JSON = "paths/output/DriveAndShoot1.wpilib.json";
            public static final String driveAndShoot2JSON = "paths/output/DriveAndShoot2.wpilib.json";
            public static final String driveAndShoot3JSON = "paths/output/DriveAndShoot3.wpilib.json";
            public static final String driveFowardJSON = "paths/output/DriveFoward.wpilib.json";
        }        
    }

    /**
     * Information on encodors and motors
     */
    public static abstract class Motors {

        /**
         * The Talon IDs for the motors
         */
        public static abstract class IDs {
            // public static final int leftFront = 0;
            // public static final int leftBack = 1;
            // public static final int rightFront = 2;
            // public static final int rightBack = 3;
            // public static final int spinner = 4;
            // public static final int frontConveyor = 5;
            // public static final int rearConveyor = 6;
            // public static final int intake = 7;
            // public static final int bottomIntake = 8;
            // public static final int shooter = 9;
            // public static final int rightElevator = 10;
            // public static final int leftElevator = 11;

            public static final int leftElevator = 0;
            public static final int rightElevator = 10;

            public static final int rightFront = 9;
            public static final int leftFront = 1;

            public static final int intake = 4;
            // public static final int frontConveyor = 4;
            public static final int backConveyor = 8;
            public static final int topShooter = 6;
            public static final int bottomShooter = 7;

            // These are unset
            public static final int spinner = 2;
            public static final int hatch = 3;

            public static final int rightElevatorLock = 0;
            public static final int leftElevatorLock = 1;
        }

        /**
         * Booleans that tell whether or not the motor or encoder is inverted
         */
        public static abstract class Inversions {
            public static final boolean leftFront = true;
            public static final boolean leftBack = true;
            public static final boolean rightFront = false;
            public static final boolean rightBack = false;

            public static final boolean leftElevator = false;
            public static final Boolean rightElevator = true;

            public static final Boolean spinner = false;
            // public static final Boolean frontConveyor = true;
            public static final Boolean backConveyor = false;
            public static final Boolean intake = true;
            // public static final int bottomIntake = 8;
            public static final Boolean topShooter = false;
            public static final Boolean bottomShooter = false;
            public static final Boolean hatch = false;

            public static final boolean kRightEncoderReversed = true;
            public static final boolean kLeftEncoderReversed = false;
        }

        /**
         * The ports for the encoders
         */
        public static abstract class Encoders {
            public static final int[] kRightEncoderPorts = { 8, 9 };
            public static final int[] kLeftEncoderPorts = { 6, 7 };
        }
    }

    /**
     * Information on limit switches
     */

    public static abstract class Switches {
        public static final int climberTopSwitch = 4;
        public static final int hatchBottomSwitch = 5;


    }

    /**
     * Joystick and button assignments
     */
    public static abstract class Controls {
        /**
         * The button IDs for specific commands. Set your button bindings here.
         */
        public static abstract class ButtonIDs {
            // public static final int intake = 2;
            // public static final int shoot = 1;
            // public static final int rotate = 7;
            // public static final int color = 8;
            // public static final int nextColor = 11;
            // public static final int autoAlign = 3;
            // public static final int elevatorUp = 6;
            // public static final int elevatorDown = 4;
            // public static final int reverse = 9;

            public static final int intake = 2;
            public static final int shoot = 1;
            public static final int rotate = 11;
            public static final int color = 9;
            public static final int nextColor = 9;
            public static final int autoAlign = 6;
            public static final int elevatorUp = 8;
            public static final int elevatorDown = 7;
            public static final int reverse = 3;
            public static final int firePowerOverride = 10;
        }

        public static final double driveThrottle = 0.5;

        public static final int driveAxisForward = 1;
        public static final int driveAxisRotate = 0;
        public static final int driveAxisThrottle = 3;
    }

    /**
     * A list of colors that the color sensor is registered to look for
     */
    public static abstract class Colors {
        // Different color IDs
        public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
        public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);
    }

    /**
     * Information for vision tracking and autoalign
     */
    public static abstract class Vision {
        public static final int[] camSize = { 320, 240 };
        public static final int alignmentOffset = 30;
        public static final double distanceOffset = 40;
    }

    public static abstract class Ultrasonics {
        public static final int ultrasonic1Port = 0;
        public static final int ultrasonic2Port = 1;

        // public static final double distanceMultiplier = 5.0/(5.0/1024.0)*6;
        public static final double distanceMultiplier = 500;
        public static final double distanceOffset = 20;

        public static final double maxRange = 500;
        public static final double minRange = 30;

        // 5 cm: 0.058
        // 30 cm: 0.06
        // 30 min distance
    }

    public static abstract class Shooter {
        /**
         * The range from which to fire in CENTIMETRES
         */
        public static final double firingRange = 105;
        public static final double topShooterPower = 0.5;
        public static final double bottomShooterPower = 0.4;

        public static final double conveyorOuttakePower = 0.5;
        public static final double frontConveyorIntakePower = 0.5;
        public static final double rearConveyorIntakePower = 0.5;
        public static final double intakeIntakePower = 0.5;
        public static final double intakeOuttakePower = 0.4;

        public static final double hatchClosedAngle = 0;
        public static final double hatchOpenAngle = 45;

        public static final double hatchOpenSpeed = 0.2;
        public static final double hatchCloseSpeed = -0.2;

        public static final double vomitSpeed = -0.3;

        /**
         * The amount of updates to delay to allow the fly wheels to speed up before
         * firing
         */
        public static final int fireDelay = 60;

        public static final int fireTime = 300;
    }

    public static abstract class Evelator {
        // public static final double upSpeed = 0.69;
        // public static final double downSpeed = 0.69;

        public static final double upSpeed = 0.95;
        public static final double downSpeed = 0.95;

        public static final double leftLockLockedAngle = 0;
        public static final double rightLockLockedAngle = 0;

        public static final double leftLockUnlockedAngle = 0;
        public static final double rightLockUnlockedAngle = 0;

    }

    /**
     * Contains information on the color spinner
     */
    public static abstract class Spinner {
        public static final double spinnerSpeed = 0.4;
        /**
         * How many colors away from the selected color to stop so that the field color
         * sensor is seeing the right color
         */
        public static final int spinnerColorOffset = 2;
    }

    public static final boolean updateShuffleboard = false;
    public static final double turnMultiplier = 0.8;
}
