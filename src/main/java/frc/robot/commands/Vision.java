/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionTracking;

public class Vision extends CommandBase {
  VisionTracking m_visionTracking;
  Drivetrain m_drivetrain;
  private double centerX = Constants.Vision.camSize[0]/2;
  private VisionThread visionThread;
  public static Object imgLock = new Object();

  private final BooleanSupplier triggered;

  /**
   * Creates a new Vision.
   */
  public Vision(VisionTracking visionTracking, Drivetrain driveTrain, BooleanSupplier trigger) {
    m_visionTracking = visionTracking;
    m_drivetrain = driveTrain;

    CommandScheduler.getInstance().registerSubsystem(m_visionTracking);

    this.triggered = trigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionTracking);
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  public void autoAlign() {
    double turn = m_visionTracking.getTurn();
    m_drivetrain.setPower(Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate),
      -Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate));

    System.out.println(String.format("Left: %s Right: %s", 
      Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate),
      -Constants.Autonomous.autoDriveSpeed + (turn * Constants.Autonomous.autoTurnRate)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(triggered.getAsBoolean()){
      autoAlign();
      System.out.println("Triggered");
    } else {
      System.out.println("Not Triggered");
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
