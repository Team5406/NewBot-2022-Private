package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class RejectLowLob extends CommandBase  {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final BoosterSubsystem booster;
    private final FrontGateSubsystem frontGate;
    private final BackGateSubsystem backGate;

    
    public RejectLowLob (FlywheelSubsystem _flywheel, HoodSubsystem _hood, BoosterSubsystem _booster, BackGateSubsystem _backGate, FrontGateSubsystem _frontGate) {
        flywheel = _flywheel;
        hood = _hood;
        booster = _booster;
        frontGate = _frontGate;
        backGate = _backGate;

        addRequirements(flywheel, hood, booster, backGate, frontGate);
      }

      @Override
      public void initialize(){
        frontGate.frontGateExtend();
        backGate.backGateRetract();
      }

      @Override
      public void execute(){
          flywheel.setShooterSpeed(Constants.FLYWHEEL_SPEED_LOW_LOB);
          hood.setHoodPosition(Constants.HOOD_ANGLE_LOW_LOB);
          booster.setBoosterSpeed(Constants.BOOSTER_SPEED);
  
      }
  
      @Override
      public void end(boolean interrupted) {
        flywheel.stopShooter();
        hood.stopHood();
        booster.stopBooster();
        frontGate.frontGateRetract();
        backGate.backGateExtend();

      }

}
