package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;

public class GateTopOpen extends CommandBase {
    BackGateSubsystem backGate;

    public GateTopOpen(BackGateSubsystem _backGate) {
        backGate = _backGate;
        addRequirements(backGate);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        backGate.backGateRetract();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
