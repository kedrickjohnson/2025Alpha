package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private static SparkMax sparkMax4 = new SparkMax(ElevatorConstants.ElevatorCanID, MotorType.kBrushless);

    public static void Extend(double speed) {
        sparkMax4.set(speed);
    }
}
