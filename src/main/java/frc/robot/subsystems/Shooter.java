/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonSRX topConveyor;
  private final TalonSRX bottomConveyor;
  private final TalonSRX topIntake;
  private final TalonSRX bottomIntake;
  private final TalonSRX shooter;

  private final Servo hatch;

  private int delayInt = 0;
  
  /**
   * Creates a new Shooter
   */
  public Shooter() {
    topConveyor = new TalonSRX(Constants.Talons.IDs.topConveyor);
    bottomConveyor = new TalonSRX(Constants.Talons.IDs.bottomConveyor);
    topIntake = new TalonSRX(Constants.Talons.IDs.topIntake);
    bottomIntake = new TalonSRX(Constants.Talons.IDs.bottomIntake);
    shooter = new TalonSRX(Constants.Talons.IDs.shooter);
    hatch = new Servo(Constants.Servos.Channels.shooterHatch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setConveyorPower(double power) {
    topConveyor.set(ControlMode.PercentOutput, power);
    bottomConveyor.set(ControlMode.PercentOutput, power);
  }

  public void setIntakePower(double power) {
    topIntake.set(ControlMode.PercentOutput, power);
    bottomIntake.set(ControlMode.PercentOutput, power);
  }

  public void setShooterPower(double power) {
    shooter.set(ControlMode.PercentOutput, power);
  }

  public void startIntake() {

  }

  public void fireAllWeapons() {
    setShooterPower(Constants.Shooter.shooterPower);

    if(delayInt < Constants.Shooter.fireDelay) {
      hatch.set(Constants.Shooter.hatchOpenAngle);
      setIntakePower(Constants.Shooter.intakeOuttakePower);
      setConveyorPower(Constants.Shooter.conveyorOuttakePower);
    } else {
      delayInt++;
    }
  }

  public void ceaseFire() {
    hatch.set(Constants.Shooter.hatchClosedAngle);
    setShooterPower(0);
    setIntakePower(0);
    setConveyorPower(0);
    delayInt = 0;
  }
}
