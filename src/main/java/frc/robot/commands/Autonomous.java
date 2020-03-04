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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Ultrasonics;
import frc.robot.subsystems.VisionTracking;

/**
 * This command is used to align the robot with the vision target, get it to the
 * proper distance, and then fire. This should be in a sequential command group
 * after the PathWeaver command.
 */

public class Autonomous extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final VisionTracking m_visionTracking;
  private final Ultrasonics m_ultrasonics;
  private final Shooter m_shooter;

  private boolean isFinished = false;

  /**
   * Creates a new Autonomous.
   */
  public Autonomous(Drivetrain drivetrain, Shooter shooter, VisionTracking visionTracking, Ultrasonics ultrasonics) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = drivetrain;
    this.m_visionTracking = visionTracking;
    this.m_ultrasonics = ultrasonics;
    this.m_shooter = shooter;

    addRequirements(drivetrain, visionTracking, ultrasonics, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoAlign();
  }

  private void autoAlign() {
    double rawTurn = m_visionTracking.getTurn();
    double distFromTarget = m_ultrasonics.getUltrasonic1() - Constants.Shooter.firingRange;

    if (Math.abs(rawTurn) >= Constants.Vision.alignmentOffset
        || Math.abs(distFromTarget) >= Constants.Vision.distanceOffset) {
      double turn = rawTurn * Constants.Autonomous.autoTurnRate;
      double power = Math.abs(distFromTarget) >= Constants.Vision.distanceOffset
          ? Constants.Autonomous.autoDriveSpeed * Math.signum(distFromTarget)
          : 0;
      m_drivetrain.setPower(power - (turn), -power + (turn));
      m_shooter.ceaseFire();

      SmartDashboard.putBoolean("Aligned", false);
    } else {
      SmartDashboard.putBoolean("Aligned", true);
      m_shooter.openFire();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
