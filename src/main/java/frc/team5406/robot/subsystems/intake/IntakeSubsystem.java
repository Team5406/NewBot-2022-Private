package frc.team5406.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private Solenoid intakePivot = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_INTAKE_PIVOT_ONE);

    public void intakeRetract() {
        intakePivot.set(Constants.INTAKE_RETRACT);
    }
    
    public void intakeExtend() {
        intakePivot.set(Constants.INTAKE_EXTEND);
    }
}
