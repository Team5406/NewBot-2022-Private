package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;


public class Shoot extends CommandBase {
    private BackGateSubsystem topGate;
    private LimelightSubsystem limelight;


    public Shoot(BackGateSubsystem _topGate, LimelightSubsystem _limelight) {
        topGate = _topGate;
        limelight = _limelight;
        addRequirements(topGate);
    }

    @Override
    public void initialize() {
        topGate.backGateRetract();
        limelight.startShoot();
    }

    @Override
    public void end(boolean interrupted) {
        topGate.backGateExtend();
        limelight.endShoot();
    }

}
