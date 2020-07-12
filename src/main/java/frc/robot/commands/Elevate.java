/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/**
 * This command controls the elevator mechanism
 */
public class Elevate extends CommandBase {
  private final Elevator m_elevator;
  private final BooleanSupplier m_upButton;
  private final BooleanSupplier m_downButton;
  // private final BooleanSupplier m_engageLockButton;
  private boolean lockElevator = true;
  /**
   * Creates a new Elevate.
   */
  public Elevate(Elevator elevator, BooleanSupplier upButton, BooleanSupplier downButton) {
    m_elevator = elevator;

    m_upButton = upButton;
    m_downButton = downButton;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_upButton.getAsBoolean())
      m_elevator.up();
    else if(m_downButton.getAsBoolean()) 
      m_elevator.down();
    else
      m_elevator.stop();

    // if(m_engageLockButton.getAsBoolean())
    //   lockElevator = !lockElevator;
    
    // if(lockElevator) 
    //   m_elevator.engageLock();
    // else m_elevator.disengageLock();
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
