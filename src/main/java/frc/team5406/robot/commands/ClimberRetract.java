package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.climber.ClimbSubsystem;

public class ClimberRetract extends CommandBase {
    ClimbSubsystem climber;

    public ClimberRetract(ClimbSubsystem _climber) {
        climber = _climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setClimber(Constants.CLIMBER_POSITION_DOWN);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Climb - End");    
    }

}