/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionTracking;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// Command class in charge of calling methods related to driving.
public class Drive extends CommandBase {
  /**
   * Creates a new Drive.
   */
  private final Drivetrain m_drivetrain;
  private final VisionTracking m_visionTracking;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotate;
  private final DoubleSupplier throttle;
  private final BooleanSupplier alignTrigger;

  // Constructor
  public Drive(Drivetrain drivetrain, VisionTracking visionTracking, DoubleSupplier forward, DoubleSupplier rotate, DoubleSupplier throttle, BooleanSupplier alignTrigger) {

    this.m_drivetrain = drivetrain;
    this.m_visionTracking = visionTracking;

    this.forward = forward;
    this.rotate = rotate;
    this.throttle = throttle;
    this.alignTrigger = alignTrigger;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
    addRequirements(m_visionTracking);
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
    //If the auto align button is not pressed, do normal drive, otherwise, do auto align
    if(!alignTrigger.getAsBoolean()) {
      m_drivetrain.setPower(dThrottle * (dForward - dRotate), dThrottle * (dForward + dRotate));
    } else {
      autoAlign();
    }

  }

  public void autoAlign() {
    double turn = m_visionTracking.getTurn();
    m_drivetrain.setPower(Constants.Autonomous.autoDriveSpeed - (turn * Constants.Autonomous.autoTurnRate),
      -Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate));
    System.out.println(turn);
    
    if(Math.abs(turn) <= 20) {
      SmartDashboard.putBoolean("Aligned", true);
    } else {
      SmartDashboard.putBoolean("Aligned", false);
    }
    // m_drivetrain.arcadeDrive(0, turn);
    // System.out.println(String.format("Left: %s Right: %s", 
    //   Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate),
    //   -Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate)));
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
