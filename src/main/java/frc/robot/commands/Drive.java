/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ultrasonics;
import frc.robot.subsystems.VisionTracking;

/**
 * Command class in charge of calling methods related to driving.
 */
public class Drive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final VisionTracking m_visionTracking;
  private final Ultrasonics m_ultrasonics;

  private final DoubleSupplier forward;
  private final DoubleSupplier rotate;
  private final DoubleSupplier throttle;
  private final BooleanSupplier alignTrigger;

  /**
   * Command class in charge of calling methods related to driving.
   * @param drivetrain Subsystem with drive methods
   * @param visionTracking Subsystem for vision processing
   * @param ultrasonics Subsystem for ultrasonic value
   * @param forward Magnitude value from the joystick
   * @param rotate Direction value from the joystick
   * @param throttle Throttle value from the joystick (acts as multiplier)
   * @param alignTrigger Button to activate auto align for shooting
   */
  public Drive(Drivetrain drivetrain, VisionTracking visionTracking, Ultrasonics ultrasonics, DoubleSupplier forward,
      DoubleSupplier rotate, DoubleSupplier throttle, BooleanSupplier alignTrigger) {

    this.m_drivetrain = drivetrain;
    this.m_visionTracking = visionTracking;
    this.m_ultrasonics = ultrasonics;

    this.forward = forward;
    this.rotate = rotate;
    this.throttle = throttle;
    this.alignTrigger = alignTrigger;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain, visionTracking);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Gets values from lambdas
    double dForward = forward.getAsDouble();
    double dRotate = rotate.getAsDouble();
    double dThrottle = throttle.getAsDouble() / -2 + 0.5;

    // Sends values to drive method.
    // If the auto align button is not pressed, do normal drive, otherwise, do auto
    // align
    if (!alignTrigger.getAsBoolean()) {
      m_drivetrain.setPower(dThrottle * (dForward - dRotate), dThrottle * (dForward + dRotate));
    } else {
      autoAlign();
    }

  }

  /**
   * Align the robot with the center of the target and move to the proper range
   */
  private void autoAlign() {
    double rawTurn = m_visionTracking.getTurn();
    double distFromTarget = m_ultrasonics.getUltrasonic1() - Constants.Shooter.firingRange;

    if (Math.abs(rawTurn) >= Constants.Vision.alignmentOffset
        || Math.abs(distFromTarget) >= Constants.Vision.distanceOffset) {
      double turn = rawTurn * Constants.Autonomous.autoTurnRate;

      // double power = Math.abs(distFromTarget) >= Constants.Vision.distanceOffset
      // ? Constants.Autonomous.autoDriveSpeed * Math.copySign(1, distFromTarget)
      // : 0;
      double power = Constants.Autonomous.autoDriveSpeed * throttle.getAsDouble()
          + Constants.Autonomous.autoTurnRateAddition;
      
      // TODO: Find out why power is negative
      m_drivetrain.setPower(power - (turn), -power + (turn));

      SmartDashboard.putBoolean("Aligned", false);
    } else {
      SmartDashboard.putBoolean("Aligned", true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
