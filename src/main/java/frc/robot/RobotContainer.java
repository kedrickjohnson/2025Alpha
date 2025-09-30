// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.ClimberConstants;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
// import frc.robot.commands.Autos;

//import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorHoldCommand;
//import frc.robot.commands.ElevatorPIDCommand;
// import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeAutoCommand;
// import frc.robot.commands.PivotAutoCommand;
import frc.robot.commands.PivotPIDCommand;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorHoldSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

 // ********Commited out elevator subsystem for elevator hold subsystem testing********
  //private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  private final ElevatorHoldSubsystem m_ElevatorHoldSubsystem = new ElevatorHoldSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // The driver's controller
  Joystick m_Joystick0 = new Joystick(OIConstants.kDriverControllerPort0);
  Joystick m_Joystick1 = new Joystick(OIConstants.kDriverControllerPort1);

  //XboxController m_XBoxController2 = new XboxController(OIConstants.kDriverControllerPort2);

  @SuppressWarnings({ "rawtypes", "unchecked" })
  SendableChooser<Command> m_chooser = new SendableChooser();
@SuppressWarnings("rawtypes")
SendableChooser m_ControllerChooser = new SendableChooser();
  @SuppressWarnings("rawtypes")
  SendableChooser m_DriveSpeed = new SendableChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("unchecked")
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("PivotL1", new PivotPIDCommand(m_PivotSubsystem, 80));
    NamedCommands.registerCommand("PivotStart", new PivotPIDCommand(m_PivotSubsystem, 180));






     NamedCommands.registerCommand("L1Full", new SequentialCommandGroup(
     new PivotPIDCommand(m_PivotSubsystem, 80),
     new WaitCommand(1),
   new PivotPIDCommand(m_PivotSubsystem, 180)
     ));
    // NamedCommands.registerCommand("L2Up", new ParallelCommandGroup(
    //   new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.ElevatorL2Setpoint),
    //   new PivotPIDCommand(m_PivotSubsystem, 80)
    // ));
   //NamedCommands.registerCommand("ScoreL2/3", new IntakeAutoCommand(m_IntakeSubsystem, 1, true));
    //NamedCommands.registerCommand("PivotLoad", new ParallelCommandGroup(
    //   new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.ElevatorL2Setpoint),
    //   new PivotPIDCommand(m_PivotSubsystem, 135)
    // ));
   // NamedCommands.registerCommand("ElevatorHold", new ElevatorPIDCommand(m_ElevatorSubsystem, 0));


  
    
    SmartDashboard.putData("Auto Chooser", m_chooser);
    m_chooser.setDefaultOption("CenterAutoOpp", AutoBuilder.buildAuto("CenterAutoOpp"));
    m_chooser.addOption("CenterAutoAll", AutoBuilder.buildAuto("CenterAutoAll"));
    m_chooser.addOption("MidOpp", AutoBuilder.buildAuto("MidOpp"));
    m_chooser.addOption("Leave", AutoBuilder.buildAuto("Leave"));
    m_chooser.addOption("MainTest", AutoBuilder.buildAuto("MainTest"));
    m_chooser.addOption("Turn", AutoBuilder.buildAuto("Turn"));
    m_chooser.addOption("PivotTest", AutoBuilder.buildAuto("PivotTest"));
    m_chooser.addOption("No Auto", NoAuto);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_Joystick0.getY() * .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_Joystick0.getX() * .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_Joystick1.getX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    SmartDashboard.putData("Controller", m_ControllerChooser);
    m_ControllerChooser.addOption("Drive", DriveControls);
    m_ControllerChooser.addOption("Showcase", null);
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
    new JoystickButton(m_Joystick1, 2)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    SmartDashboard.putData("Start", new ElevatorHoldCommand(m_ElevatorHoldSubsystem, ElevatorHoldCommand.Preset.START));
    SmartDashboard.putData("L2", new ElevatorHoldCommand(m_ElevatorHoldSubsystem, ElevatorHoldCommand.Preset.L2));
    SmartDashboard.putData("L3", new ElevatorHoldCommand(m_ElevatorHoldSubsystem, ElevatorHoldCommand.Preset.L3));

    // final JoystickButton ElevatorUp = new JoystickButton(m_Joystick1, 6);
    // ElevatorUp.whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    // final JoystickButton ElevatorDown = new JoystickButton(m_Joystick0, 5);
    // ElevatorDown.whileTrue(new ElevatorCommand(-1 * ElevatorConstants.ElevatorSpeed / 2, m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    // final JoystickButton Climb = new JoystickButton(m_Joystick0, 11); //lowers arms
    // Climb.whileTrue(m_ClimberSubsystem.climberCommand(ClimberConstants.ClimbSpeed * -1)).whileFalse(new ClimberSubsystem().climberCommand(0));

    // final JoystickButton ClimbReverse = new JoystickButton(m_Joystick1, 11); //Raises arm
    // ClimbReverse.whileTrue(m_ClimberSubsystem.climberCommand(ClimberConstants.ClimbSpeed)).whileFalse(m_ClimberSubsystem.climberCommand(0));

    // final JoystickButton PivotUp = new JoystickButton(m_Joystick0, 4);
    // PivotUp.whileTrue(m_PivotSubsystem.pivotCommand(PivotConstants.PivotSpeed)).whileFalse(new PivotSubsystem().pivotCommand(0));

    // final JoystickButton PivotDown = new JoystickButton(m_Joystick1, 3);
    // PivotDown.whileTrue(m_PivotSubsystem.pivotCommand(PivotConstants.PivotSpeed * -1)).whileFalse(new PivotSubsystem().pivotCommand(0));

    // final JoystickButton ElevatorStart = new JoystickButton(m_Joystick0, 6);
    // ElevatorStart.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.ElevatorStartSetpoint));

    // final JoystickButton ElevatorL2 = new JoystickButton(m_Joystick1, 5);
    // ElevatorL2.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.ElevatorL2Setpoint));

    // final JoystickButton ElevatorL3 = new JoystickButton(m_Joystick0, 12);
    // ElevatorL3.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.ElevatorL3Setpoint));

    // final JoystickButton ResetElevatorEncoder = new JoystickButton(m_Joystick0, 7);
    // ResetElevatorEncoder.onTrue(m_ElevatorSubsystem.ResetEncoder());

    final JoystickButton PivotLoad = new JoystickButton(m_Joystick1, 8);
    PivotLoad.onTrue(new PivotPIDCommand(m_PivotSubsystem, Constants.PivotConstants.PivotLoad));

    final JoystickButton PivotL1 = new JoystickButton(m_Joystick1, 7);
    PivotL1.onTrue(new PivotPIDCommand(m_PivotSubsystem, Constants.PivotConstants.PivotL1));;

    /*final JoystickButton PivotL2 = new JoystickButton(m_Joystick1, 1);
    PivotL1.onTrue(new PivotPIDCommand(m_PivotSubsystem, Constants.PivotConstants.PivotL1));*/

    final JoystickButton Intake = new JoystickButton(m_Joystick0, 1);
    Intake.whileTrue(m_IntakeSubsystem.Intake(-IntakeConstants.IntakeSpeed)).onFalse(m_IntakeSubsystem.Intake(0));

    final JoystickButton Outtake = new JoystickButton(m_Joystick1, 1);
    Outtake.whileTrue(m_IntakeSubsystem.Intake(IntakeConstants.IntakeSpeed)).onFalse(m_IntakeSubsystem.Intake(0));

  }

  private Command NoAuto;

  private Object DriveControls = new Object() {

  };

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
