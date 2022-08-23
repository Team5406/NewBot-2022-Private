
package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class ResetHoodEncoder extends CommandBase {

    private final HoodSubsystem hood;
    private int currentCounter = 0;
    public ResetHoodEncoder(HoodSubsystem _hood) {
        hood = _hood;
   //     addRequirements(shooter);
    }

    @Override
    public void initialize() {
        hood.disableHoodLimits();
        currentCounter = 0;
    }

    @Override
    public void execute() {
        hood.setHoodSpeed(Constants.HOOD_ZEROING_SPEED); // 1.65 is magic number
        if(hood.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE){
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
        hood.resetHood();
        hood.stopHood();
        hood.enableHoodLimits();
    }
}