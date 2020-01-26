/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  public static final int INTAKE_BUTTON_ID = 5;
  public static final int SHOOT_BUTTON_ID = 6;
  public static final int ROTATION_BUTTON_ID = 7;
  public static final int COLOR_BUTTON_ID = 8;
  public static final int NEXT_COLOR_BUTTON_ID = 12;

  public static final double DRIVE_THROTTLE = 0.5;

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
}
