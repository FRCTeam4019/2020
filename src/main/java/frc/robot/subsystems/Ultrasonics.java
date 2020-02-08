/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ultrasonics extends SubsystemBase {
  //.05 offset
  //.09 for 5mm
  //
  private AnalogPotentiometer pot1 = new AnalogPotentiometer(Constants.Ultrasonics.ultrasonic1Port, 
    Constants.Ultrasonics.distanceMultiplier, Constants.Ultrasonics.distanceOffset);
  private AnalogPotentiometer pot2 = new AnalogPotentiometer(Constants.Ultrasonics.ultrasonic2Port, 
    Constants.Ultrasonics.distanceMultiplier, Constants.Ultrasonics.distanceOffset);
  /**
   * Creates a new Ultrasonics.
   */
  public Ultrasonics() {
    
  }

  /**
   * @return The current distance that the Ultrasonic 1 is reading
   */
  public double getUltrasonic1() {
    return pot1.get();
  }

  /**
   * @return The current distance that Ultrasonic 2 is reading
   */
  public double getUltrasonic2() {
    return pot2.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putString("Ultrasonic 1", formatPot(pot1.get()));
    SmartDashboard.putString("Ultrasonic 2", formatPot(pot2.get()));
  }

  //Format the distance output to be a readable string
  private String formatPot(double pot) {
    String nicePot = String.format("%.2f", pot);

    if(pot <= Constants.Ultrasonics.minRange) {
      nicePot = "<= " + Constants.Ultrasonics.minRange;
    }

    if(pot >= Constants.Ultrasonics.maxRange) {
      nicePot = ">= " + Constants.Ultrasonics.maxRange;
    }

    return nicePot + " cm";
  }
}
