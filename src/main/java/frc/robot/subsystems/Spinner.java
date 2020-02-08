/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spinner extends SubsystemBase {
  private final TalonSRX spinner;
  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    spinner = new TalonSRX(Constants.Talons.IDs.spinner);
  }

  /**
   * Set the power for the spinner
   * @param power The pecentage of power to send to the spinner
   */
  public void setPower(double power) {
    spinner.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
