/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.channels.Pipe;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner.Listener;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.vision.VisionThread;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewColorSensor;
import frc.robot.subsystems.Shooter;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

// REMEMBER THIS
// https://phoenix-documentation.readthedocs.io/en/latest/ch05a_CppJava.html#frc-c-java-add-phoenix

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;

  // Joysticks
  public static Joystick m_leftStick;
  public static Joystick m_rightStick;

  // Subsystems
  public static Drivetrain m_drivetrain;
  public static NewColorSensor m_colorsensor;
  public static Shooter m_shooter;

  private double centerX = 0.0;

  // public static Ultrasonic;
  private VisionThread visionThread;
  private final Object imgLock = new Object();

  public SimpleWidget color_tab;
  public Supplier<String> color_supplier = () -> new String("");

  private static final int[] IMG_SIZE = { 320, 240 };

  private static AnalogPotentiometer ultrasonic = new AnalogPotentiometer(1, 645, 0);

  Trajectory trajectory1;

  @Override
  public void robotInit() {
    // Ititalizing Joysticks
    m_leftStick = new Joystick(RobotMap.LEFT_STICK_ID);
    m_rightStick = new Joystick(RobotMap.RIGHT_STICK_ID);

    // Ititalizing subsystems
    m_drivetrain = new Drivetrain();
    m_colorsensor = new NewColorSensor();
    m_shooter = new Shooter();

    // Camera Code
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    // UsbCamera camera = CameraServer.getInstance().;
    camera.setResolution(IMG_SIZE[0], IMG_SIZE[1]);
    CvSource output = CameraServer.getInstance().putVideo("Processed: ", IMG_SIZE[0], IMG_SIZE[1]);

    visionThread = new VisionThread(CameraServer.getInstance().getVideo().getSource(), new Pipeline(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }
        // System.out.println(r.width + " " + r.height);
      }
      output.putFrame(pipeline.hsvThresholdOutput());

    });
    visionThread.start();

    // Initializes an AnalogInput on port 0, and enables 2-bit averaging
    // AnalogInput input = new AnalogInput(0);
    SmartDashboard.updateValues();

    String trajectoryJSON = "paths/YourPath.wpilib.json";

  }

  @Override
  public void autonomousPeriodic() {
    double centerX;
    synchronized (imgLock) {
      centerX = this.centerX;
    }
    // System.out.println(SmartDashboard.getNumber("Luminance", 180));
    double turn = centerX - (IMG_SIZE[0] / 2);
    // System.out.println("Turn: " + turn);
    m_drivetrain.setPower(RobotMap.AUTO_DRIVE_SPEED + (turn * RobotMap.AUTO_TURN_RATE),
        -RobotMap.AUTO_DRIVE_SPEED + (turn * RobotMap.AUTO_TURN_RATE));
    // m_drivetrain.setPower(-1, -1);
    // m_drivetrain.setPower(0.5 * (forward - rotate), throttle * (forward +
    // rotate));
    SmartDashboard.updateValues();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void teleopInit() {
    robotInit();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Ultrasonic", ultrasonic.get());

    Scheduler.getInstance().run();

    Shuffleboard.update();

  }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(RobotMap.ksVolts,
                                       RobotMap.kvVoltSecondsPerMeter,
                                       RobotMap.kaVoltSecondsSquaredPerMeter),
            RobotMap.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(RobotMap.kMaxSpeedMetersPerSecond,
                             RobotMap.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RobotMap.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(RobotMap.kRamseteB, RobotMap.kRamseteZeta),
        new SimpleMotorFeedforward(RobotMap.ksVolts,
                                   RobotMap.kvVoltSecondsPerMeter,
                                   RobotMap.kaVoltSecondsSquaredPerMeter),
        RobotMap.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(RobotMap.kPDriveVel, 0, 0),
        new PIDController(RobotMap.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

}
