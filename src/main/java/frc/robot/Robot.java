/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.channels.Pipe;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

  private static final int[] IMG_SIZE = {320, 240};

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
    /*// Pipeline pipeline = new Pipeline();

    // Listener listener = new Listener<Pipeline>() {
    //   @Override
    //   public void copyPipelineOutputs(Pipeline pipeline) {
    //     // TODO Auto-generated method stub
    //   }
    // };

    // VisionRunner runner = new
    // VisionRunner<edu.wpi.first.vision.VisionPipeline>(camera, pipeline,
    // listener);

    // VisionThread thread = new VisionThread(CameraServer.getInstance().getVideo().getSource(), pipeline, listener);
    // thread.setName("Processed");
    // CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 1280, 720);
    // thread.start();

    new Thread(() -> {
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 1280, 720);
      Pipeline pipeline = new Pipeline();

      Mat source = new Mat();
      Mat output = new Mat();

      while (!Thread.interrupted()) {
        cvSink.grabFrame(source); 
        pipeline.process(source);
        //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(pipeline.hslThresholdOutput());
      }
    }).start();*/
    CvSource output = CameraServer.getInstance().putVideo("Processed: ", IMG_SIZE[0], IMG_SIZE[1]);

    visionThread = new VisionThread(CameraServer.getInstance().getVideo().getSource(), new Pipeline(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
              centerX = r.x + (r.width / 2);
          }
          System.out.println(r.width + " " + r.height);
      }
      output.putFrame(pipeline.hslThresholdOutput());

  });
  visionThread.start();
  }

@Override
public void autonomousPeriodic() {
    double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }
    double turn = centerX - (IMG_SIZE[0] / 2);
    System.out.println("Turn: " + turn);
    m_drivetrain.setPower(RobotMap.AUTO_DRIVE_SPEED + (turn * RobotMap.AUTO_TURN_RATE), -RobotMap.AUTO_DRIVE_SPEED + (RobotMap.AUTO_TURN_RATE * 0.003));
    // m_drivetrain.setPower(-1, -1);
    // m_drivetrain.setPower(0.5 * (forward - rotate), throttle * (forward + rotate));
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
    Scheduler.getInstance().run();
    // vision.testPixy1();

    /*
     * String colorString; Color detectedColor = m_colorSensor.getColor();
     * 
     * ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
     * 
     * 
     * 
     * if (match.color == kBlueTarget) {
     * 
     * colorString = "Blue";
     * 
     * } else if (match.color == kRedTarget) {
     * 
     * colorString = "Red";
     * 
     * } else if (match.color == kGreenTarget) {
     * 
     * colorString = "Green";
     * 
     * } else if (match.color == kYellowTarget) {
     * 
     * colorString = "Yellow";
     * 
     * } else {
     * 
     * colorString = "Unknown";
     * 
     * }
     * 
     * color_supplier = () -> colorString;
     */
    Shuffleboard.update();

  }

}
