/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionTracking;

import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
  /**
   * Creates a new Drive.
   */
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotate;
  private final DoubleSupplier throttle;

  public Drive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier rotate, DoubleSupplier throttle) {

    this.m_drivetrain = drivetrain;

    this.forward = forward;
    this.rotate = rotate;
    this.throttle = throttle;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dForward = forward.getAsDouble();
    double dRotate = rotate.getAsDouble();
    double dThrottle = throttle.getAsDouble() / -2 + 0.5;

    m_drivetrain.setPower(dThrottle * (dForward - dRotate), dThrottle * (dForward + dRotate));

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
