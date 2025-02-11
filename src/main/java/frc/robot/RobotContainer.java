// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  Joystick m_Joystick0 = new Joystick(OIConstants.kDriverControllerPort0);
  Joystick m_Joystick1 = new Joystick(OIConstants.kDriverControllerPort1);
  //XboxController m_XBoxController2 = new XboxController(OIConstants.kDriverControllerPort2);

  SendableChooser<Command> m_chooser = new SendableChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    SmartDashboard.putData("Auto Chooser",m_chooser);
    m_chooser.setDefaultOption("MainTest", AutoBuilder.buildAuto("MainTest"));
    m_chooser.addOption("Turn", AutoBuilder.buildAuto("Turn"));
    m_chooser.addOption("No Auto", NoAuto);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_Joystick0.getY() * 0.25, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_Joystick0.getX() * 0.25, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_Joystick1.getX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_Joystick0, 1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    final JoystickButton ElevatorUp = new JoystickButton(m_Joystick0, 5);
    ElevatorUp.whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem)).whileFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    final JoystickButton ElevatorDown = new JoystickButton(m_Joystick0, 3);
    ElevatorDown.whileTrue(new ElevatorCommand(-1 * ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem)).whileFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    final JoystickButton ClimbUp = new JoystickButton(m_Joystick1, 6);
    ClimbUp.whileTrue(new ClimberSubsystem().climberCommand(ClimberConstants.ClimbSpeed)).whileFalse(new ClimberSubsystem().climberCommand(0));

    final JoystickButton ClimbDown = new JoystickButton(m_Joystick1, 4);
    ClimbDown.whileTrue(new ClimberSubsystem().climberCommand(ClimberConstants.ClimbSpeed * -1)).whileFalse(new ClimberSubsystem().climberCommand(0));
  }

  private Command NoAuto;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
