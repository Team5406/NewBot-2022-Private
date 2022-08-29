package frc.team5406.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;

public class IntakeDeployCommand extends CommandBase {
    IntakeSubsystem intake;

    public IntakeDeployCommand(IntakeSubsystem _intake) {
        intake = _intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        intake.intakeExtend();
    }

    @Override
    public boolean isFinished() {
      return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake Deploy - End");
    }
}
