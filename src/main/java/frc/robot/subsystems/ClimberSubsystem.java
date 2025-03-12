package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants;

public class ClimberSubsystem extends SubsystemBase {
    private static SparkMax sparkMax2 = new SparkMax(ClimberConstants.FrontClimbCanID, MotorType.kBrushless);
    private static SparkMax sparkMax3 = new SparkMax(ClimberConstants.BackClimbCanID, MotorType.kBrushless);
    private static SparkMaxConfig config = new SparkMaxConfig();
    private static AbsoluteEncoder encoder;
    private static double speed;

    public ClimberSubsystem() {
        
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder.positionConversionFactor(360);
        sparkMax2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkMax2.getAbsoluteEncoder();
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public Command climberCommand(double speedSupplied) {
        return run(() -> {
            if (getEncoderPosition() > 130) {
                speed = speedSupplied * 2;
            } else {
                speed = speedSupplied;
            }
            sparkMax2.set(speed);
            sparkMax3.set(speed);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Angle", getEncoderPosition());
    }
}
