package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCommand extends Command {
    
    private final IntakeSubsystem m_intakeSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    private final boolean direction;
    private boolean Finished;

    public IntakeAutoCommand(IntakeSubsystem intakeSubsystem, double Time, boolean inversed) {
        m_intakeSubsystem = intakeSubsystem;
        time = Time;
        direction = inversed;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Finished = false;
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            double speed = direction ? IntakeConstants.IntakeSpeed : -IntakeConstants.IntakeSpeed;
            m_intakeSubsystem.setSpeed(speed);
        } else {
            Finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Finished;
    }
}
