package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private static SparkMax sparkMax4 = new SparkMax(ElevatorConstants.ElevatorCanID1, MotorType.kBrushless);
    private static SparkMax sparkMax12 = new SparkMax(ElevatorConstants.ElevatorCanID2, MotorType.kBrushless);
    private static SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();
    private static SparkClosedLoopController elevatorController = sparkMax4.getClosedLoopController();
    
    private static final DigitalInput encA = new DigitalInput(3);
    private static final DigitalInput encB = new DigitalInput(4);
    private static final DigitalInput encIndex = new DigitalInput(5);
    private static Encoder encoder = new Encoder(encA, encB, true, EncodingType.k4X);
    private static PIDController PIDElevator = new PIDController(0.00015, 0.00002, 0);
    //private static DigitalInput bottomLimit = new DigitalIO;

    public ElevatorSubsystem() {
        elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
        sparkMax4.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.apply(elevatorConfig);
        followerConfig.follow(sparkMax4, false);
        sparkMax12.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        PIDElevator.setTolerance(100);
        PIDElevator.setIZone(500);
        encoder.setDistancePerPulse((Math.PI * 1.432) / 2048);
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

    public Encoder getEncoder() {
        return encoder;
    }

    public double getEncoderPosition() {
        return encoder.getRaw();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public void PIDExtend(double speed) {
        if (speed > ElevatorConstants.ElevatorMaxSpeed) {
            speed = ElevatorConstants.ElevatorMaxSpeed;
        } if (speed < -ElevatorConstants.ElevatorMaxSpeed) {
            speed = -ElevatorConstants.ElevatorMaxSpeed;
        }
        sparkMax4.set(speed);
    }

    public static void Extend(double speed) {
        sparkMax4.set(speed);
    }

    public Command ResetEncoder() {
        return runOnce(() -> resetEncoder());
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber("Pivot Height", getEncoderPosition()); //Add 23.25
        //SmartDashboard.putBoolean("DI Test", test.get());
    }
}
