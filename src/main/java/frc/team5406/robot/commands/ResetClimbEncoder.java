
package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.climber.ClimbSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class ResetClimbEncoder extends CommandBase {

    private final ClimbSubsystem climb;
    private int currentCounter = 0;
    public ResetClimbEncoder(ClimbSubsystem _climb) {
        climb = _climb;
   //     addRequirements(shooter);
    }

    @Override
    public void initialize() {
        climb.disableClimbLimits();
        currentCounter = 0;
    }

    @Override
    public void execute() {
        climb.setClimberSpeed(Constants.HOOD_ZEROING_SPEED); // 1.65 is magic number
        if(climb.getClimbCurrent() >= Constants.NEO550_CURRENT_SPIKE){
            currentCounter++;
        }
    }
    @Override
    public boolean isFinished() {
        //return ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE && Math.abs(ShooterSubsystem.getHoodVelocity()) <= 10;
        return currentCounter >= 3;
    }

    @Override
    public void end(boolean interrupted){
        climb.resetClimb();
        climb.stopClimber();
        climb.enableClimbLimits();
    }
}