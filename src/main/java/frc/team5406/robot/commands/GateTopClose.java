package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;

public class GateTopClose extends CommandBase {
    BackGateSubsystem backGate;

    public GateTopClose(BackGateSubsystem _backGate) {
        backGate = _backGate;
        addRequirements(backGate);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        backGate.backGateExtend();
    }
    
    @Override
    public boolean isFinished() {
      return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
