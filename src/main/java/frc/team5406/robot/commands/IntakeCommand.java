package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intake;
    FeederSubsystem feeder;

    public IntakeCommand(IntakeSubsystem _intake, FeederSubsystem _feeder) {
        intake = _intake;
        feeder = _feeder;
        addRequirements(intake, feeder);
    }

    @Override
    public void execute() {
        intake.intakeExtend();
        feeder.setFeederBottomSpeed(1000); // FIXME
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake - End");
        intake.intakeRetract();
    }
}
