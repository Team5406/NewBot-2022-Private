package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;

public class Shoot extends CommandBase {
    private BackGateSubsystem topGate;

    public Shoot(BackGateSubsystem _topGate) {
        topGate = _topGate;
        addRequirements(topGate);
    }

    @Override
    public void initialize() {
        topGate.backGateRetract();
    }

    @Override
    public void end(boolean interrupted) {
        topGate.backGateExtend();
    }

}
