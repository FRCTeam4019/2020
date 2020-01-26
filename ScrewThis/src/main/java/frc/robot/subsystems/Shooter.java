/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Shoot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX conveyor_top;
  TalonSRX conveyor_bottom;
  TalonSRX intake_top;
  TalonSRX intake_bottom;
  TalonSRX outtake;

  public Shooter() {
    conveyor_top = new TalonSRX(RobotMap.TALON_TOP_CONVEYOR_ID);
    conveyor_bottom = new TalonSRX(RobotMap.TALON_BOTTOM_CONVEYOR_ID);
    intake_top = new TalonSRX(RobotMap.TALON_TOP_INTAKE_ID);
    intake_bottom = new TalonSRX(RobotMap.TALON_BOTTOM_INTAKE_ID);
    outtake = new TalonSRX(RobotMap.TALON_SHOOTER_ID);
  }

  public void intake(){
    intake_bottom.set(ControlMode.PercentOutput, 1); //TODO: Find power number
    intake_top.set(ControlMode.PercentOutput, 1);
  }

  public void shoot(){
    intake_bottom.set(ControlMode.PercentOutput, 1); //TODO: Find power number
    intake_top.set(ControlMode.PercentOutput, 1);
    conveyor_bottom.set(ControlMode.PercentOutput, 1); //TODO: Find power number
    conveyor_top.set(ControlMode.PercentOutput, 1);
    outtake.set(ControlMode.PercentOutput, 1);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Shoot());
  }
}
