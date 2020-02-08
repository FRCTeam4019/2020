/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ColorSense;
import frc.robot.commands.Drive;
import frc.robot.commands.Vision;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Ultrasonics;
import frc.robot.subsystems.VisionTracking;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Drivetrain m_drivetrain;
  private VisionTracking m_visiontracking;
  private ColorSensor m_colorSensor;
  private Ultrasonics m_ultrasonics;
  private Spinner m_spinner;

  private Drive m_drive;
  private ColorSense m_colorSense;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setResolution(Constants.Vision.camSize[0], Constants.Vision.camSize[1]);
    //cam.setExposureManual(1);
    cam.setExposureAuto();
    cam.setWhiteBalanceAuto();
    
    m_robotContainer = new RobotContainer();
    m_drivetrain =  m_robotContainer.m_drivetrain;
    m_visiontracking = m_robotContainer.m_visiontracking;
    m_colorSensor = m_robotContainer.m_colorSensor;
    m_ultrasonics = m_robotContainer.m_ultrasonics;
    m_spinner = m_robotContainer.m_spinner;

    m_drive = m_robotContainer.m_drive;
    m_colorSense = m_robotContainer.m_colorSense;

    
    //cam.setBrightness(1);

    //Starts periodic updates for visionTracking
    CommandScheduler.getInstance().registerSubsystem(m_visiontracking);
    CommandScheduler.getInstance().registerSubsystem(m_drivetrain);
    CommandScheduler.getInstance().registerSubsystem(m_colorSensor);
    CommandScheduler.getInstance().registerSubsystem(m_ultrasonics);
  }

  
   /* teleoperated and test.
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //Cancels the normal Teleop drive command
    m_drive.cancel();
    m_colorSense.cancel();
  }

  /**

    * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.updateValues();
  }
  
  @Override
  public void teleopInit() {
    //Start the Teleop drive command
    m_drive.schedule();
    m_colorSense.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
