package frc.team5406.robot.subsystems.gates;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class FrontGateSubsystem extends SubsystemBase {

    private Solenoid frontGate;

    public void setupSolenoid() {
        frontGate = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_FEEDER_GATE_TOP_ONE);
    }

    public void frontGateRetract() {
        frontGate.set(Constants.FRONT_GATE_RETRACT);
    }

    public void frontGateExtend() {
        frontGate.set(Constants.FRONT_GATE_EXTEND);
    }

    public FrontGateSubsystem() {
        setupSolenoid();
    }

}
