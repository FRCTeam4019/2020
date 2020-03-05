/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous;
import frc.robot.commands.ColorSense;
import frc.robot.commands.Drive;
import frc.robot.commands.Elevate;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Ultrasonics;
import frc.robot.subsystems.VisionTracking;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Drivetrain m_drivetrain = new Drivetrain();
  public final VisionTracking m_visiontracking = new VisionTracking();
  public final ColorSensor m_colorSensor = new ColorSensor();
  public final Ultrasonics m_ultrasonics = new Ultrasonics();
  public final Shooter m_shooter = new Shooter();
  public final Elevator m_elevator = new Elevator();

  public Drive m_drive;
  public ColorSense m_colorSense;
  public Autonomous m_autoCommand;
  public Shoot m_shoot;
  public Elevate m_elevate;
  public Command m_autoCommandGroup;

  public final Joystick m_drivestick = new Joystick(0);
  public final Joystick m_operatestick = new Joystick(1);

  // NetworkTableEntry autonomousSelector;
  SendableChooser<Integer> autonomousSelector;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoCommand = new Autonomous(m_drivetrain, m_shooter, m_visiontracking, m_ultrasonics);

    String[] options = { "Drive foward", "Drive and Shoot" };

    // autonomousSelector = Shuffleboard.getTab("SmartDashboard")
    // .add("Autonomous Selector", 0)
    // .withWidget(BuiltInWidgets.kComboBoxChooser)
    // .getEntry();
    // autonomousSelector.forceSetStringArray(options);

    autonomousSelector = new SendableChooser<Integer>();

    autonomousSelector.addOption("Drive Foward", 0);
    autonomousSelector.addOption("Drive and Shoot Pos 1", 1);
    autonomousSelector.addOption("Drive and Shoot Pos 2", 2);
    autonomousSelector.addOption("Drive and Shoot Pos 3", 3);
    SmartDashboard.putData("Autonomous Selector 1", autonomousSelector);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drive = new Drive(m_drivetrain, m_visiontracking, m_ultrasonics,
        () -> m_drivestick.getRawAxis(Constants.Controls.driveAxisForward),
        () -> m_drivestick.getRawAxis(Constants.Controls.driveAxisRotate),
        () -> m_drivestick.getRawAxis(Constants.Controls.driveAxisThrottle),
        () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.autoAlign));

    // m_colorSense = new ColorSense(m_colorSensor,
    //     () -> m_drivestick.getRawButtonPressed(Constants.Controls.ButtonIDs.rotate),
    //     () -> m_drivestick.getRawButtonPressed(Constants.Controls.ButtonIDs.color),
    //     () -> m_drivestick.getRawButtonPressed(Constants.Controls.ButtonIDs.nextColor));

    m_colorSense = new ColorSense(m_colorSensor,
        () -> m_operatestick.getRawButtonPressed(Constants.Controls.ButtonIDs.rotate),
        () -> m_operatestick.getRawButtonPressed(Constants.Controls.ButtonIDs.color),
        () -> m_operatestick.getRawButtonPressed(Constants.Controls.ButtonIDs.nextColor));

    // m_shoot = new Shoot(m_shooter, () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.shoot),
    //     () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.intake),
    //     () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.reverse));

    m_shoot = new Shoot(m_shooter,
        () -> m_operatestick.getRawButton(Constants.Controls.ButtonIDs.shoot),
        () -> m_operatestick.getRawButton(Constants.Controls.ButtonIDs.intake),
        () -> m_operatestick.getRawButton(Constants.Controls.ButtonIDs.reverse));

    // m_elevate = new Elevate(m_elevator, () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.elevatorUp),
    //     () -> m_drivestick.getRawButton(Constants.Controls.ButtonIDs.elevatorDown));

    m_elevate = new Elevate(m_elevator, 
        () -> m_operatestick.getRawButton(Constants.Controls.ButtonIDs.elevatorUp),
        () -> m_operatestick.getRawButton(Constants.Controls.ButtonIDs.elevatorDown));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autonomousSelector.getSelected() == 1) {
      Trajectory trajectory = null;

      // Get the pathweaver JSON file from the file system and convert it to a path
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
            .resolve(Constants.Autonomous.TrajectoryPaths.driveAndShoot1JSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + Constants.Autonomous.TrajectoryPaths.driveAndShoot1JSON,
            ex.getStackTrace());
      }

      RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
          new RamseteController(Constants.Stats.kRamseteB, Constants.Stats.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.Stats.ksVolts, Constants.Stats.kvVoltSecondsPerMeter,
              Constants.Stats.kaVoltSecondsSquaredPerMeter),
          Constants.Stats.kDriveKinematics, m_drivetrain::getWheelSpeeds,
          new PIDController(Constants.Stats.kPDriveVel, 0, 0), new PIDController(Constants.Stats.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrain::tankDriveVolts, m_drivetrain);

      // Run path following command, then stop at the end.
      // return ramseteCommand;

      return new SequentialCommandGroup(ramseteCommand, m_autoCommand);

    }
    if (autonomousSelector.getSelected() == 2) {
      Trajectory trajectory = null;

      // Get the pathweaver JSON file from the file system and convert it to a path
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
            .resolve(Constants.Autonomous.TrajectoryPaths.driveAndShoot2JSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + Constants.Autonomous.TrajectoryPaths.driveAndShoot2JSON,
            ex.getStackTrace());
      }

      RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
          new RamseteController(Constants.Stats.kRamseteB, Constants.Stats.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.Stats.ksVolts, Constants.Stats.kvVoltSecondsPerMeter,
              Constants.Stats.kaVoltSecondsSquaredPerMeter),
          Constants.Stats.kDriveKinematics, m_drivetrain::getWheelSpeeds,
          new PIDController(Constants.Stats.kPDriveVel, 0, 0), new PIDController(Constants.Stats.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrain::tankDriveVolts, m_drivetrain);

      // Run path following command, then stop at the end.
      // return ramseteCommand;

      return new SequentialCommandGroup(ramseteCommand, m_autoCommand);
    } 
    if (autonomousSelector.getSelected() == 3) {
      Trajectory trajectory = null;

      // Get the pathweaver JSON file from the file system and convert it to a path
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
            .resolve(Constants.Autonomous.TrajectoryPaths.driveAndShoot2JSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError(
            "Unable to open trajectory: " + Constants.Autonomous.TrajectoryPaths.driveAndShoot3JSON,
            ex.getStackTrace());
      }

      RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
          new RamseteController(Constants.Stats.kRamseteB, Constants.Stats.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.Stats.ksVolts, Constants.Stats.kvVoltSecondsPerMeter,
              Constants.Stats.kaVoltSecondsSquaredPerMeter),
          Constants.Stats.kDriveKinematics, m_drivetrain::getWheelSpeeds,
          new PIDController(Constants.Stats.kPDriveVel, 0, 0), new PIDController(Constants.Stats.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrain::tankDriveVolts, m_drivetrain);

      // Run path following command, then stop at the end.
      // return ramseteCommand;

      return new SequentialCommandGroup(ramseteCommand, m_autoCommand);

    }
    
    Trajectory trajectory = null;

    // Get the pathweaver JSON file from the file system and convert it to a path

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
          .resolve(Constants.Autonomous.TrajectoryPaths.driveFowardJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Constants.Autonomous.TrajectoryPaths.driveFowardJSON,
          ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
        new RamseteController(Constants.Stats.kRamseteB, Constants.Stats.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.Stats.ksVolts, Constants.Stats.kvVoltSecondsPerMeter,
            Constants.Stats.kaVoltSecondsSquaredPerMeter),
        Constants.Stats.kDriveKinematics, m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.Stats.kPDriveVel, 0, 0), new PIDController(Constants.Stats.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts, m_drivetrain);

    // Run path following command, then stop at the end.
    // return ramseteCommand;

    return ramseteCommand;
  }
}
