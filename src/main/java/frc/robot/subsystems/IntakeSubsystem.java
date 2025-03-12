package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private static SparkMax sparkMax11 = new SparkMax(IntakeConstants.IntakeCanID, MotorType.kBrushless);

    public IntakeSubsystem() {

    }

    public void setSpeed(double speed) {
        sparkMax11.set(speed);
    }

    public Command Intake(double speed) {
        return run(() -> {
            setSpeed(speed);
        });
    }
}
