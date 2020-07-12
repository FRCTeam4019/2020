/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ultrasonics;

public class AutoDriveFoward extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Ultrasonics m_ultrasonics;
  private double distanceDriven = 0;
  private boolean isFinished = false;

  /**
   * Creates a new AutoDriveFoward.
   */
  public AutoDriveFoward(Drivetrain drivetrain, Ultrasonics ultrasonics) {
    m_drivetrain = drivetrain;
    m_ultrasonics = ultrasonics;

    addRequirements(m_drivetrain, m_ultrasonics);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_ultrasonics.getUltrasonic1() / 100.00 > Constants.Autonomous.eStopDistance
    if (distanceDriven < Constants.Autonomous.driveForwardDistance) {
      if (m_ultrasonics.getUltrasonic2() / 100.00 > Constants.Autonomous.eStopDistance) {
        double encoderDiff = (m_drivetrain.getLeftEncoder().getDistance()
            - m_drivetrain.getRightEncoder().getDistance()) * Constants.Autonomous.encoderDiffMultiplier;

        // m_drivetrain.setPower(Constants.Autonomous.autoDriveSpeed - encoderDiff,
        //     Constants.Autonomous.autoDriveSpeed + encoderDiff);

        m_drivetrain.setPower(-Constants.Autonomous.autoDriveSpeed,
            -Constants.Autonomous.autoDriveSpeed);

        // distanceDriven = m_drivetrain.getLeftEncoder().getDistance() - encoderDiff;
        
      } else {
        m_drivetrain.setPower(0, 0);
      }
    } else {
      m_drivetrain.setPower(0, 0);
      isFinished = true;
    }

    distanceDriven = m_drivetrain.getLeftEncoder().getDistance();
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
