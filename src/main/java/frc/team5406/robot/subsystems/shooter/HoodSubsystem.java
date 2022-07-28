package frc.team5406.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private CANSparkMax hood = new CANSparkMax(Constants.MOTOR_HOOD_ONE, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder;
    private SparkMaxPIDController hoodPID;

    public void setupMotors(){
        hoodEncoder = hood.getEncoder();
        hoodPID = hood.getPIDController();

        hood.setSmartCurrentLimit(Constants.CURRENT_LIMIT_HOOD);
        hood.setInverted(true);

        hood.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.GEAR_RATIO_HOOD*40/360);
        hood.setSoftLimit(SoftLimitDirection.kReverse, 0);

        resetEncoders();
        hood.burnFlash();
    }

    public void resetEncoders(){
        hoodEncoder.setPosition(0);
    }

    public HoodSubsystem(){
        setupMotors();
    }


}
