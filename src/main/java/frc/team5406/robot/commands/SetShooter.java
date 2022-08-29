package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;


public class SetShooter extends CommandBase {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final LimelightSubsystem limelight;

    private final BoosterSubsystem booster;

    public SetShooter (FlywheelSubsystem _flywheel, HoodSubsystem _hood, BoosterSubsystem _booster, LimelightSubsystem _limelight) {
        flywheel = _flywheel;
        hood = _hood;
        limelight = _limelight;
        booster = _booster;
        addRequirements(flywheel, hood, booster);
    }

    @Override
    public void initialize() {
      limelight.turnOnLimelight();
      System.out.println("Set Shooter - Start");

      
    }

    @Override
    public void execute(){
        flywheel.setShooterSpeed(limelight.getLLShooterSpeed());
        hood.setHoodPosition(limelight.getLLHoodPosition());
        booster.setBoosterSpeed(Constants.BOOSTER_SPEED);

    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Set Shooter - End");


    }
}
