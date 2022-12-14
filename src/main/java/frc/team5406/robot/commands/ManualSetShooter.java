package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class ManualSetShooter extends CommandBase  {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final double flywheelSpeed, hoodPosition;
    private final BoosterSubsystem booster;

    
    public ManualSetShooter (FlywheelSubsystem _flywheel, HoodSubsystem _hood, BoosterSubsystem _booster, double _flywheelSpeed, double _hoodPosition) {
        flywheel = _flywheel;
        hood = _hood;
        flywheelSpeed = _flywheelSpeed;
        hoodPosition = _hoodPosition;
        booster = _booster;
        addRequirements(flywheel, hood, booster);
      }

      @Override
      public void execute(){
          flywheel.setShooterSpeed(flywheelSpeed);
          hood.setHoodPosition(hoodPosition);
          booster.setBoosterSpeed(Constants.BOOSTER_SPEED);
  
      }
  
      @Override
      public void end(boolean interrupted) {
        flywheel.stopShooter();
        hood.stopHood();
        booster.stopBooster();
      }

}
