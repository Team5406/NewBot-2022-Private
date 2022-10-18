package frc.team5406.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax climber = new CANSparkMax(Constants.MOTOR_CLIMBER_ONE, MotorType.kBrushless);

    private RelativeEncoder climbEncoder;
    private SparkMaxPIDController climbPID;

    public void setupMotors(){
        climber.restoreFactoryDefaults();

        climbEncoder = climber.getEncoder();
        climbPID = climber.getPIDController();

        climbPID.setP(Constants.CLIMBER_PID0_P, 0);
        climbPID.setI(Constants.CLIMBER_PID0_I, 0);
        climbPID.setD(Constants.CLIMBER_PID0_D, 0);
        climbPID.setIZone(Constants.PID_IZ_VALUE, 0);
        climbPID.setFF(Constants.CLIMBER_PID0_F, 0);
        climbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

        climber.setIdleMode(IdleMode.kBrake);

        climber.enableSoftLimit(SoftLimitDirection.kForward, true);
        climber.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climber.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.CLIMBER_DISTANCE);
        climber.setSoftLimit(SoftLimitDirection.kReverse, 0);

        climber.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CLIMBER);

        resetEncoders();
        climber.burnFlash();
    }

    public void resetEncoders(){
        climbEncoder.setPosition(0);
    }

    public void setClimber(double value) {
        climbPID.setReference(value, ControlType.kPosition, 0);
  
    }

    public void moveClimbUp(){
    }

    public void stopClimber(){
        climber.set(0);
    }

    public ClimbSubsystem(){
        setupMotors();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current", climber.getOutputCurrent());
        SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
    }
}
