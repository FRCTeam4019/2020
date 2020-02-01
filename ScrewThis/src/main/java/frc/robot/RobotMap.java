/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  public static final int TALON_LEFT_FRONT_ID = 0;
  public static final int TALON_LEFT_BACK_ID = 1;
  public static final int TALON_RIGHT_FRONT_ID = 2;
  public static final int TALON_RIGHT_BACK_ID = 3;
  public static final int TALON_SPINNER_ID = 4;
  public static final int TALON_TOP_CONVEYOR_ID = 5;
  public static final int TALON_BOTTOM_CONVEYOR_ID = 6;
  public static final int TALON_TOP_INTAKE_ID = 7;
  public static final int TALON_BOTTOM_INTAKE_ID = 8;
  public static final int TALON_SHOOTER_ID = 9;

  public static final boolean TALON_LEFT_FRONT_INVERT = false;
  public static final boolean TALON_LEFT_BACK_INVERT = false;
  public static final boolean TALON_RIGHT_FRONT_INVERT = true;
  public static final boolean TALON_RIGHT_BACK_INVERT = true;

  public static final int LEFT_STICK_ID = 0;
  public static final int RIGHT_STICK_ID = 1;

  public static final int DRIVE_AXIS_FORWARD = 0;
  public static final int DRIVE_AXIS_ROTATION = 1;
  public static final int DRIVE_AXIS_THROTTLE = 3;

  public static final int INTAKE_BUTTON_ID = 2;
  public static final int SHOOT_BUTTON_ID = 1;
  public static final int ROTATION_BUTTON_ID = 7;
  public static final int NEXT_COLOR_BUTTON_ID = 12;
  public static final int COLOR_BUTTON_BLUE_ID = 3;
  public static final int COLOR_BUTTON_GREEN_ID = 4;
  public static final int COLOR_BUTTON_YELLOW_ID = 5;
  public static final int COLOR_BUTTON_RED_ID = 6;
  

  public static final double DRIVE_THROTTLE = 0.5;

  public static final double AUTO_DRIVE_SPEED = 0.6;
  public static final double AUTO_TURN_RATE = 0.003;

  /*
   * public static final int HAND_OPEN_BUTTON_ID = 5; public static final int
   * HAND_CLOSE_BUTTON_ID = 3;
   * 
   * public static final int AUTO_FLOOR_BUTTON_ID = 10; public static final int
   * AUTO_HATCH_1_BUTTON_ID = 7; public static final int AUTO_HATCH_2_BUTTON_ID =
   * 9; public static final int AUTO_HATCH_3_BUTTON_ID = 11; public static final
   * int AUTO_CARGO_TOGGLE_BUTTON_ID = 8;
   */

  public static final int SPINNER_SPEED = 100;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;
  public static final double kTrackwidthMeters = 0.69;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final int[] kRightEncoderPorts = {0, 1};
  public static final int[] kLeftEncoderPorts = {0, 1};

  public static final boolean kRightEncoderReversed = false;
  public static final boolean kLeftEncoderReversed = false;

  public static final double kEncoderDistancePerPulse = 0;
  public static final boolean kGyroReversed = false;
}