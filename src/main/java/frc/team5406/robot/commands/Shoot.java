package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;

public class Shoot extends CommandBase {
    private BoosterSubsystem booster;

    public Shoot(BoosterSubsystem _booster) {
        booster = _booster;
        addRequirements(booster);
    }

    @Override
    public void execute() {
        double boosterSpeed = SmartDashboard.getNumber("Booster Target RPM", 1000);
        booster.setBoosterSpeed(boosterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        booster.stopBooster();
    }

}
