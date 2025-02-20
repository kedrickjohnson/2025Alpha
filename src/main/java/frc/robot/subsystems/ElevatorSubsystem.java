package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private static SparkMax sparkMax4 = new SparkMax(ElevatorConstants.ElevatorCanID, MotorType.kBrushless);
    private static SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;
    PIDController PIDElevator = new PIDController(1, 0, 0);

    public ElevatorSubsystem() {
        elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.encoder.positionConversionFactor(0.1);
        sparkMax4.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkMax4.getEncoder();
        PIDElevator.enableContinuousInput(0, 5);
        PIDElevator.setTolerance(0.02);
    };

    public void goToSetpoint(double setpoint) {
        double speed = PIDElevator.calculate(getEncoderPosition(), setpoint);
        sparkMax4.set(speed);
        SmartDashboard.putNumber("Elevator Speed", speed);
    }

    public boolean atSetpoint() {
        return PIDElevator.atSetpoint();
    }

    public void stopMotor() {
        sparkMax4.set(0);
    }

    public RelativeEncoder getRelativeEncoder() {
        return encoder;
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public static void Extend(double speed) {
        sparkMax4.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Height", getEncoderPosition() + 0);
    }
}
