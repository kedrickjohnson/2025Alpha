package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

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

  private static Encoder encoder = new Encoder(encA, encB, true, EncodingType.k4X);
  private static PIDController PidElevator = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,
      ElevatorConstants.kD);
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
     encoder.setDistancePerPulse((Math.PI * 1.432) / 2048);
  }

  @Override
  public void periodic() {
    // Set the PID controller's setpoint to the target position
    PidElevator.setSetpoint(targetPosition);

    // Calculate the motor output based on the current position
    double speed = PidElevator.calculate(encoder.get());

    // Set the motor speed to hold the elevator at the target position
    sparkMax4.set(speed);

    // putting values on the SmartDashboard for tuning and logging
    SmartDashboard.putNumber("Elevator Speed", speed);
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putBoolean("At Target Position", atTargetPosition());

  }

  // Methods to set target positions

  public void setStart() {
    targetPosition = ElevatorConstants.ElevatorStartSetpoint;
  }

  public void setL2() {
    targetPosition = ElevatorConstants.ElevatorL2Setpoint;
  }

  public void setL3() {
    targetPosition = ElevatorConstants.ElevatorL3Setpoint;
  }

  // Method to stop the elevator motor
  public void stop() {
    sparkMax4.set(0);
  }

  // Method to check if the elevator is at the target position
  public boolean atTargetPosition() {
    return PidElevator.atSetpoint();

  }

  public double getPosition() {
    return encoder.get();
  
  }
  public void resetEncoder() {
    encoder.reset();
  }
}