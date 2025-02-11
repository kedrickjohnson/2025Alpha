package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private static SparkMax sparkMax4 = new SparkMax(ElevatorConstants.ElevatorCanID, MotorType.kBrushless);
    private static SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    SparkClosedLoopController PIDElevator = sparkMax4.getClosedLoopController();

    public ElevatorSubsystem() {
        elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
        elevatorConfig.absoluteEncoder.positionConversionFactor(1);
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1, 0, 0);
        sparkMax4.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    };

    public static void Extend(double speed) {
        sparkMax4.set(speed);
    }
}
