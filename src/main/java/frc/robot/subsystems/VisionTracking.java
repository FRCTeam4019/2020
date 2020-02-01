/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pipeline;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionTracking extends SubsystemBase {

  private VisionThread visionThread;
  public static Object imgLock = new Object();
  private double centerX = 0.0;

  /**
   * Creates a new VisionTracking.
   */
  public VisionTracking() {

    CvSource output = CameraServer.getInstance().putVideo("Processed: ", Constants.Vision.camSize[0],
        Constants.Vision.camSize[1]);

    visionThread = new VisionThread(CameraServer.getInstance().getVideo().getSource(), new Pipeline(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }
        System.out.println(r.width + " " + r.height);
      }
      output.putFrame(pipeline.hsvThresholdOutput());

    });
    visionThread.start();

  }

  private void autoAlign() {
    double centerX;
    synchronized (imgLock) {
      centerX = this.centerX;
    }
    double turn = centerX - (Constants.Vision.camSize[0] / 2);
    RobotContainer.m_drivetrain.setPower(Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate),
        -Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate));
    System.out.println(turn);
  }

  @Override
  public void periodic() {
    autoAlign();
  }
}
