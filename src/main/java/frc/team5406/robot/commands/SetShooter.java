package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;


public class SetShooter extends CommandBase {
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final LimelightSubsystem limelight;

    private final BoosterSubsystem booster;

    public SetShooter (FlywheelSubsystem _flywheel, HoodSubsystem _hood, BoosterSubsystem _booster, LimelightSubsystem _limelight) {
        flywheel = _flywheel;
        hood = _hood;
        limelight = _limelight;
        booster = _booster;
        SmartDashboard.putNumber("hoodValue", 1);
        SmartDashboard.putNumber("flywheelValue", 1000);
        addRequirements(flywheel, hood, booster);
    }

    @Override
    public void initialize() {
      limelight.turnOnLimelight();
      System.out.println("Set Shooter - Start");

      
    }

    @Override
    public void execute(){
    //  double shooter = SmartDashboard.getNumber("flywheelValue", 1000);
      //double hoodValue = SmartDashboard.getNumber("hoodValue", 1);
    double shooterValue = limelight.getLLShooterSpeed();
      double hoodValue =limelight.getLLHoodPosition();
      SmartDashboard.putNumber("shooterValue", 1000);
     // SmartDashboard.putNumber("hoodValue", 10+9);
        flywheel.setShooterSpeed(shooterValue);
        hood.setHoodPosition(hoodValue);
        booster.setBoosterSpeed(Constants.BOOSTER_SPEED);

    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Set Shooter - End");


    }
}
