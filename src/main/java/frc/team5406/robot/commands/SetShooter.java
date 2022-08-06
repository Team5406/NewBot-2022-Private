package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class SetShooter extends CommandBase {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final LimelightSubsystem limelight;

    public SetShooter (FlywheelSubsystem _flywheel, HoodSubsystem _hood, LimelightSubsystem _limelight) {
        flywheel = _flywheel;
        hood = _hood;
        limelight = _limelight;
        addRequirements(flywheel, hood, limelight);
    }

    @Override
    public void initialize() {
      limelight.turnOnLimelight();
    }

    @Override
    public void execute(){
        flywheel.setShooterSpeed(limelight.getLLShooterSpeed());
        hood.setHoodPosition(limelight.getLLHoodPosition());
    }

    @Override
    public void end(boolean interrupted) {
      flywheel.stopShooter();     
      limelight.turnOffLimelight();
    }
}
