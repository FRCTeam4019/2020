/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  private final Shooter m_shooter;
  private final BooleanSupplier m_fireButton;
  private final BooleanSupplier m_intakeButton;

  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, BooleanSupplier fireButton, BooleanSupplier intakeButton) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;

    m_fireButton = fireButton;
    m_intakeButton = intakeButton;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_fireButton.getAsBoolean()) 
      m_shooter.fireAllWeapons();
    else m_shooter.ceaseFire();
    
    if(m_intakeButton.getAsBoolean())
      m_shooter.setIntakePower(Constants.Shooter.intakeIntakePower);
    else m_shooter.setIntakePower(0);
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
