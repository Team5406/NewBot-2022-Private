package frc.team5406.robot.subsystems.gates;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class BackGateSubsystem extends SubsystemBase {
    
    private Solenoid backGate;

    public void setupSolenoid() {
        backGate = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_FEEDER_GATE_BOTTOM_ONE);
    }

    
  public void backGateRetract() {
    backGate.set(Constants.BACK_GATE_RETRACT);
  }

  public void backGateExtend() {
    backGate.set(Constants.BACK_GATE_EXTEND);
  }
  
    public BackGateSubsystem(){
        setupSolenoid();
    }
}
