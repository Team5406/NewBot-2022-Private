package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class ManualSetShooter extends CommandBase  {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final double flywheelSpeed, hoodPosition;
    
    public ManualSetShooter (FlywheelSubsystem _flywheel, HoodSubsystem _hood, double _flywheelSpeed, double _hoodPosition) {
        flywheel = _flywheel;
        hood = _hood;
        flywheelSpeed = _flywheelSpeed;
        hoodPosition = _hoodPosition;
        addRequirements(flywheel, hood);
      }

      @Override
      public void execute(){
          flywheel.setShooterSpeed(flywheelSpeed);
          hood.setHoodPosition(hoodPosition);
      }
  
      @Override
      public void end(boolean interrupted) {
    
      }

}
