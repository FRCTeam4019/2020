/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem controls the intake, conveyor, and firing mechanisms.
 */
public class Shooter extends SubsystemBase {
  // private final TalonSRX frontConveyor;
  private final TalonSRX backConveyor;
  private final TalonSRX intake;
  // private final TalonSRX bottomIntake;
  private final TalonSRX topShooter;
  private final TalonSRX bottomShooter;
  private final TalonSRX hatchMotor;

  private final DigitalInput hatchTopSwitch = new DigitalInput(Constants.Switches.hatchTopSwitch);
  private final DigitalInput hatchBottomSwitch = new DigitalInput(Constants.Switches.hatchBottomSwitch);

  private int delayInt = 0;

  NetworkTableEntry topShooterSlider;
  NetworkTableEntry bottomShooterSlider;
  

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // frontConveyor = new TalonSRX(Constants.Motors.IDs.frontConveyor);
    // frontConveyor.setInverted(Constants.Motors.Inversions.frontConveyor);

    backConveyor = new TalonSRX(Constants.Motors.IDs.backConveyor);
    backConveyor.setInverted(Constants.Motors.Inversions.backConveyor);

    intake = new TalonSRX(Constants.Motors.IDs.intake);
    intake.setInverted(Constants.Motors.Inversions.intake);
    // bottomIntake = new TalonSRX(Constants.Motors.IDs.bottomIntake);
    topShooter = new TalonSRX(Constants.Motors.IDs.topShooter);
    topShooter.setInverted(Constants.Motors.Inversions.topShooter);

    bottomShooter = new TalonSRX(Constants.Motors.IDs.bottomShooter);
    bottomShooter.setInverted(Constants.Motors.Inversions.bottomShooter);

    hatchMotor = new TalonSRX(Constants.Motors.IDs.hatch);

    topShooterSlider = Shuffleboard.getTab("Motors")
    .add("Top Spinner 1", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    .getEntry();

    bottomShooterSlider = Shuffleboard.getTab("Motors")
    .add("Bottom Spinner 1", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putData("Hatch Top Switch", hatchTopSwitch);
  }

  public void setConveyorPower(double power) {
    // frontConveyor.set(ControlMode.PercentOutput, power);
    backConveyor.set(ControlMode.PercentOutput, power);
  }

  public void setIntakePower(double power) {
    intake.set(ControlMode.PercentOutput, power);
    // bottomIntake.set(ControlMode.PercentOutput, power);
  }

  public void setShooterPower(double power) {
    topShooter.set(ControlMode.PercentOutput, power);
    bottomShooter.set(ControlMode.PercentOutput, power);
  }

  /**
   * Activates the intake and conveyor system
   */
  public void startIntake() {
    // setConveyorPower(Constants.Shooter.conveyorIntakePower);
    // frontConveyor.set(ControlMode.PercentOutput, Constants.Shooter.frontConveyorIntakePower);
    setConveyorPower(Constants.Shooter.rearConveyorIntakePower);
    setIntakePower(Constants.Shooter.intakeIntakePower);
  }

  /**
   * Vomit: eject matter from the stomach through the mouth
   */
  public void vomit() {
    setConveyorPower(Constants.Shooter.vomitSpeed);
    setIntakePower(Constants.Shooter.vomitSpeed);
  }

  /**
   * Runs the hatch motor until the top limit switch has been hit
   * 
   * @return Whether or not the hatch is fully open
   */
  public Boolean openHatch() {
    // The switch returns true when it is open
    if (hatchTopSwitch.get()) {
      hatchMotor.set(ControlMode.PercentOutput, Constants.Shooter.hatchOpenSpeed);
      return false;
    }

    hatchMotor.set(ControlMode.PercentOutput, 0);

    return true;
  }

  /**
   * Runs the hatch motor until the bottom limit switch has been hit
   * 
   * @return Whether or not the hatch is fully closed
   */
  public Boolean closeHatch() {
    // The switch returns true when it is open
    if (hatchBottomSwitch.get()) {
      hatchMotor.set(ControlMode.PercentOutput, Constants.Shooter.hatchCloseSpeed);
      return false;
    }

    hatchMotor.set(ControlMode.PercentOutput, 0);

    return true;
  }

  /**
   * <b>Does the following:</b>
   * <ol>
   * <li>Starts up the flywheels
   * <li>Waits a certain amount of time to allow the flywheels to start
   * <li>After the timer is up, opens the hatch and starts the conveyor
   * </ol>
   */
  public void openFire() {
    // Start up the fly wheels
    // setShooterPower(Constants.Shooter.shooterPower);
    // SmartDashboard.add();
    topShooter.set(ControlMode.PercentOutput, topShooterSlider.getDouble(0));
    bottomShooter.set(ControlMode.PercentOutput, bottomShooterSlider.getDouble(0));
    // test();

    // Give the fly wheels a chance to pick up speed before firing
    if (delayInt >= Constants.Shooter.fireDelay) {
      // openHatch() || 
      if (true) {
        setIntakePower(Constants.Shooter.intakeOuttakePower);
        setConveyorPower(Constants.Shooter.conveyorOuttakePower);
        
      }
    } else {
      delayInt++;
    }
  }

  public void test() {
    hatchMotor.set(ControlMode.PercentOutput, 0.05);
  }

  /**
   * Stops the intake system, the flywheels, the conveyor, and resets the firing
   * timer
   */
  public void ceaseFire() {
    // hatch.set(Constants.Shooter.hatchClosedAngle);
    setShooterPower(0);
    setIntakePower(0);
    setConveyorPower(0);
    hatchMotor.set(ControlMode.PercentOutput, 0);
    delayInt = 0;
  }
}
