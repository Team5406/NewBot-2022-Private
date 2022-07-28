package frc.team5406.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    private CANSparkMax feederTop = new CANSparkMax(Constants.MOTOR_FEEDER_TOP_ONE, MotorType.kBrushless);
    private CANSparkMax feederBottom = new CANSparkMax(Constants.MOTOR_FEEDER_BOTTOM_ONE,  MotorType.kBrushless);

    private RelativeEncoder feederTopEncoder, feederBottomEncoder;

    private SparkMaxPIDController feederTopPID, feederBottomPID;
    
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    private Solenoid frontGate, backGate;

    public void setupMotors(){
        feederTopEncoder = feederTop.getEncoder();
        feederBottomEncoder = feederBottom.getEncoder();

        feederTopPID = feederTop.getPIDController();
        feederBottomPID = feederBottom.getPIDController();

        frontGate = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_FEEDER_GATE_TOP_ONE);
        backGate = new Solenoid(PneumaticsModuleType.REVPH, Constants.CYLINDER_FEEDER_GATE_BOTTOM_ONE);

        feederTop.setSmartCurrentLimit(Constants.CURRENT_LIMIT_FEEDER_TOP);
        feederBottom.setSmartCurrentLimit(Constants.CURRENT_LIMIT_FEEDER_BOTTOM);

        resetEncoders();
        feederTop.burnFlash();
        feederBottom.burnFlash();
    }

    public void resetEncoders(){
        feederTopEncoder.setPosition(0);
        feederBottomEncoder.setPosition(0);
    }

    public void disableCompressor(){
        compressor.disable();
    }
    
    public void enableCompressor(){
        compressor.enableAnalog(Constants.MIN_PRESSURE, Constants.MAX_PRESSURE);
    }

    public FeederSubsystem(){
        setupMotors();
    }
    
}