package frc.team5406.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.subsystems.climber.ClimbSubsystem;



public class ResetClimber extends Trigger {
    ClimbSubsystem climbSubsystem;
    public ResetClimber (ClimbSubsystem _climbSubsystem){  
        climbSubsystem = _climbSubsystem;
    }

    @Override
    public boolean get() {
      return !climbSubsystem.climberReset;
    }
  }