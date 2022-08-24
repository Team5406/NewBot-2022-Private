package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;



public class ResetHood extends Trigger {
    HoodSubsystem hoodSubsystem;
    public ResetHood (HoodSubsystem _hoodSubsystem){  
        hoodSubsystem = _hoodSubsystem;
    }

    @Override
    public boolean get() {
      return !hoodSubsystem.hoodReset;
    }
  }