package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;

public class Shoot extends CommandBase {
    private BoosterSubsystem booster;
    private FeederSubsystem feeder;

    public Shoot(BoosterSubsystem _booster, FeederSubsystem _feeder) {
        booster = _booster;
        feeder = _feeder;
        addRequirements(booster, feeder);
    }

    @Override
    public void execute() {
        double boosterSpeed = SmartDashboard.getNumber("Booster Target RPM", 1000);
        booster.setBoosterSpeed(boosterSpeed);
        feeder.setFeederTopSpeed(Constants.FEEDER_SPEED_TOP);
    }

    @Override
    public void end(boolean interrupted) {
        booster.stopBooster();
        feeder.stopFeederTop();
    }

}
