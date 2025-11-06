package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class ElevatorHoldSubsystem extends SubsystemBase {

  private static SparkMax sparkMax4 = new SparkMax(ElevatorConstants.ElevatorCanID1, MotorType.kBrushless);
  private static SparkMax sparkMax12 = new SparkMax(ElevatorConstants.ElevatorCanID2, MotorType.kBrushless);
  private static SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private static SparkMaxConfig followerConfig = new SparkMaxConfig();

  private static final DigitalInput encA = new DigitalInput(ElevatorConstants.encA);
  private static final DigitalInput encB = new DigitalInput(ElevatorConstants.encB);
  // (limit switch removed)

  private static Encoder encoder = new Encoder(encA, encB, true, EncodingType.k4X);
  private static ProfiledPIDController PidElevator = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      ElevatorConstants.kElevatorConstraints);
  private static ElevatorFeedforward feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV);
  // Limit Switche (if needed in future)
  // private static DigitalInput BottomLimit = new DigitalInput(5);

  private double targetPosition; // Target position

  public ElevatorHoldSubsystem() {
    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
    sparkMax4.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig.apply(elevatorConfig);
    followerConfig.follow(sparkMax4, false);
    sparkMax12.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    PidElevator.setTolerance(ElevatorConstants.kTolerance);
    PidElevator.setIZone(ElevatorConstants.kIZone);
    encoder.setDistancePerPulse((Math.PI * ElevatorConstants.PitchDiameter / (2048*4)));
  }

  @Override
  public void periodic() {
    // Calculate the motor output based on the current position
    // ProfiledPIDController automatically handles the motion profile
    double pidOutput = PidElevator.calculate(encoder.getDistance());
    
    // Calculate feedforward to compensate for gravity and velocity
    double feedforwardOutput = feedforward.calculate(PidElevator.getSetpoint().velocity);
    
    // Combine PID and feedforward outputs
    double voltage = pidOutput + feedforwardOutput;

    // Set the motor voltage to control the elevator
    sparkMax4.setVoltage(voltage);

    // putting values on the SmartDashboard for tuning and logging
    SmartDashboard.putNumber("Elevator Voltage", voltage);
    SmartDashboard.putNumber("Elevator PID Output", pidOutput);
    SmartDashboard.putNumber("Elevator Feedforward Output", feedforwardOutput);
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Target Position", targetPosition);
    SmartDashboard.putNumber("Elevator Goal Position", PidElevator.getGoal().position);
    SmartDashboard.putNumber("Elevator Setpoint Position", PidElevator.getSetpoint().position);
    SmartDashboard.putNumber("Elevator Setpoint Velocity", PidElevator.getSetpoint().velocity);
    SmartDashboard.putBoolean("At Target Position", atTargetPosition());

  }

  // Methods to set target positions

  public void setStart() {
    targetPosition = ElevatorConstants.ElevatorStartSetpoint;
    PidElevator.setGoal(targetPosition);
  }

  public void setL2() {
    targetPosition = ElevatorConstants.ElevatorL2Setpoint;
    PidElevator.setGoal(targetPosition);
  }

  public void setL3() {
    targetPosition = ElevatorConstants.ElevatorL3Setpoint;
    PidElevator.setGoal(targetPosition);
  }

  // Method to stop the elevator motor
  public void stop() {
    sparkMax4.setVoltage(0);
  }

  // Method to check if the elevator is at the target position
  public boolean atTargetPosition() {
    return PidElevator.atSetpoint();

  }

  public double getPosition() {
    return encoder.getDistance();
  
  }
  public static void Extend(double speed) {
    sparkMax4.set(speed);
}

  public void ResetEncoder() {
    encoder.reset();
  }
}