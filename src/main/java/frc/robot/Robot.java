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
import frc.robot.commands.Drive;
import frc.robot.commands.Vision;
import frc.robot.subsystems.Drivetrain;
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

  private Drive m_drive;

  private VisionThread visionThread;
  public static Object imgLock = new Object();
  private double centerX = 0.0;

  Ultrasonic ultrasonic = new Ultrasonic(5,6);
  AnalogPotentiometer pot = new AnalogPotentiometer(new AnalogInput(0), 1, 0);

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

    m_drive = m_robotContainer.m_drive;
    //cam.setBrightness(1);

    //Starts periodic updates for visionTracking
    CommandScheduler.getInstance().registerSubsystem(m_visiontracking);

    ultrasonic.setAutomaticMode(true);
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    ultrasonic.ping();
    SmartDashboard.putNumber("Ultrasonic", pot.get());
    SmartDashboard.updateValues();
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
