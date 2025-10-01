package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorHoldSubsystem;

/**
 * Hold the elevator at a preset: START, L2, or L3.
 */
public class ElevatorHoldCommand extends Command {
  public enum Preset { START, L2, L3 }
  private final ElevatorHoldSubsystem elevator;
  private final Preset preset;

  public ElevatorHoldCommand(ElevatorHoldSubsystem elevator, Preset preset) {
    this.elevator = elevator;
    this.preset = preset;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    switch (preset) {
      case START -> elevator.setStart();
      case L2    -> elevator.setL2();
      case L3    -> elevator.setL3();
    }
  }

  @Override
  public void execute() {
    // ElevatorHoldSubsystem's periodic() will run the PID to hold the target.
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.atTargetPosition();
  }
}
