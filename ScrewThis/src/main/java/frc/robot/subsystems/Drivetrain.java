/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

// Subsystem containing all methods relating to driving
public class Drivetrain extends Subsystem {
  /**
   * Creates a new Drivetrain.
   */

  // Talon motor controllers
  TalonSRX tLF;
  TalonSRX tLB;
  TalonSRX tRF;
  TalonSRX tRB;

    // The robot's drive
  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
    new Encoder(RobotMap.kLeftEncoderPorts[0], RobotMap.kLeftEncoderPorts[1],
      RobotMap.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
    new Encoder(RobotMap.kRightEncoderPorts[0], RobotMap.kRightEncoderPorts[1],
      RobotMap.kRightEncoderReversed);

  // The gyro sensor                                                                                                                                                                                                                                                                                                                                         
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;


  //Constructor
  public Drivetrain() {

    tLF = new TalonSRX(RobotMap.TALON_LEFT_FRONT_ID);
    tLB = new TalonSRX(RobotMap.TALON_LEFT_BACK_ID);
    tRF = new TalonSRX(RobotMap.TALON_RIGHT_FRONT_ID);
    tRB = new TalonSRX(RobotMap.TALON_RIGHT_BACK_ID);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());
  }

  // Sets the speed of the motors
  public void setPower(double leftPower, double rightPower) {
    tLF.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * leftPower);
    tLB.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * leftPower);

    tRF.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * rightPower);
    tRB.set(ControlMode.PercentOutput, RobotMap.DRIVE_THROTTLE * rightPower);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
    setDefaultCommand(new Drive());
  }

    /**              
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    setPower(RobotMap.AUTO_DRIVE_SPEED + (rot * RobotMap.AUTO_TURN_RATE),
    -RobotMap.AUTO_DRIVE_SPEED + (rot * RobotMap.AUTO_TURN_RATE));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    tLF.set(ControlMode.Current, leftVolts);
    tLB.set(ControlMode.Current, leftVolts);
    tRF.set(ControlMode.Current, rightVolts);
    tRB.set(ControlMode.Current, rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    // setPower(100, 100);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (RobotMap.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (RobotMap.kGyroReversed ? -1.0 : 1.0);
  }
}
