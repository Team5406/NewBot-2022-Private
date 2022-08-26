package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class HighLob extends CommandBase  {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final BoosterSubsystem booster;

    
    public HighLob (FlywheelSubsystem _flywheel, HoodSubsystem _hood, BoosterSubsystem _booster) {
        flywheel = _flywheel;
        hood = _hood;
        booster = _booster;

        addRequirements(flywheel, hood, booster);
      }

      @Override
      public void initialize(){
      }

      @Override
      public void execute(){
          flywheel.setShooterSpeed(Constants.FLYWHEEL_SPEED_FENDER_HIGH);
          hood.setHoodPosition(Constants.HOOD_ANGLE_FENDER_HIGH);
          booster.setBoosterSpeed(Constants.BOOSTER_SPEED);
  
      }
  
      @Override
      public void end(boolean interrupted) {
        flywheel.stopShooter();
        hood.stopHood();
        booster.stopBooster();
      }

}
