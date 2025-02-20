package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    
    private static SparkMax sparkMax13 = new SparkMax(PivotConstants.PivotCanID, MotorType.kBrushless);
    private static SparkMaxConfig pivotConfig = new SparkMaxConfig();

    public PivotSubsystem() {
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimit550).inverted(true);
        sparkMax13.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        sparkMax13.set(speed);
    }

    public Command pivotCommand(double speed) {
        return run(() -> {
            sparkMax13.set(speed);
        });
    }
}
