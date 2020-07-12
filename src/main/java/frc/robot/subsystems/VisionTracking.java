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

public class VisionTracking extends SubsystemBase {
  // This subsystem is used for vision tracking

  private VisionThread visionThread;
  public static Object imgLock = new Object();
  private double centerX = 0.0;
  double turn = 0;

  /**
   * Creates a new VisionTracking.
   */
  public VisionTracking() {

    CvSource output = CameraServer.getInstance().putVideo("Processed: ", Constants.Vision.camSize[0],
        Constants.Vision.camSize[1]);

    // Creates the new visionthread which updates the camera and contours
    visionThread = new VisionThread(CameraServer.getInstance().getVideo().getSource(), new Pipeline(), pipeline -> {

      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }
      }
      output.putFrame(pipeline.hsvThresholdOutput());
    });
    // visionThread.set

    // visionThread.start();
  }

  /**
   * @return The amount of pixels from the center that the target is
   */
  public double getTurn() {
    double centerX;

    synchronized (imgLock) {
      centerX = this.centerX;
    }

    turn = centerX - (Constants.Vision.camSize[0] / 2);
    return turn;
  }

  @Override
  public void periodic() {

  }
}
