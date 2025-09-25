package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorHoldSubsystem extends SubsystemBase {

  private static final int elevatorMotorCANId = ElevatorConstants.ElevatorCanID1;
  private final SparkMax elevatorMotor;
  private final AbsoluteEncoder absoluteEncoder;
  private final PIDController pidController;

  private double targetPosition = 0.0;

  public ElevatorHoldSubsystem() {
    elevatorMotor = new SparkMax(elevatorMotorCANId, MotorType.kBrushless);
    absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

    // Initialize PID controller with proportional, integral, and derivative gains
    pidController = new PIDController(  ElevatorConstants.kP, 
                                        ElevatorConstants.kI, 
                                        ElevatorConstants.kD
                                        );
                               
    pidController.setTolerance(ElevatorConstants.kTolerance); // Set tolerance for acceptable error
  }

  @Override
  public void periodic() {
    // Set the PID controller's setpoint to the target position
    pidController.setSetpoint(targetPosition);

    // Calculate the motor output based on the current position
    double speed = pidController.calculate(absoluteEncoder.getPosition());

    // Set the motor speed to hold the elevator at the target position
    elevatorMotor.set(speed);
  }

  // Method to set the target position for the elevator
  public void holdPosition(double position) {
    targetPosition = position;
  }

  // Method to stop the elevator motor
  public void stop() {
    elevatorMotor.set(0);
  }

  // Method to check if the elevator is at the target position
  public boolean atTargetPosition() {
    return pidController.atSetpoint();
  }
}