package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;

public class GateBottomClose extends CommandBase {
    FrontGateSubsystem frontGate;

    public GateBottomClose(FrontGateSubsystem _frontGate) {
        frontGate = _frontGate;
        addRequirements(frontGate);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        frontGate.frontGateExtend();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
