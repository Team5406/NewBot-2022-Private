package frc.team5406.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private CANSparkMax hood = new CANSparkMax(Constants.MOTOR_HOOD_ONE, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder;
    private SparkMaxPIDController hoodPID;

    public boolean hoodReset = false;


    public void setupMotors() {
        hoodEncoder = hood.getEncoder();
        hoodPID = hood.getPIDController();

        hoodPID.setP(Constants.HOOD_PID1_P, 1);
        hoodPID.setI(Constants.HOOD_PID1_I, 1);
        hoodPID.setD(Constants.HOOD_PID1_D, 1);
        hoodPID.setIZone(0, 1);
        hoodPID.setFF(Constants.HOOD_PID1_F, 1);
        hoodPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);
        hoodPID.setSmartMotionAllowedClosedLoopError(0.05, 1);

        hood.setSmartCurrentLimit(Constants.CURRENT_LIMIT_HOOD);
        hood.setInverted(true);

        hood.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.GEAR_RATIO_HOOD * 40 / 360);
        hood.setSoftLimit(SoftLimitDirection.kReverse, 0);

        resetEncoders();
        hood.burnFlash();
    }

    public void resetEncoders() {
        hoodEncoder.setPosition(0);
    }

    public void setHoodPosition(double angle) {
        hoodPID.setReference(angle / 360 * Constants.GEAR_RATIO_HOOD, ControlType.kPosition, 1);
    }

    public double getHoodCurrent() {
        return hood.getOutputCurrent();
    }

    public double getHoodVelocity() {
        return hoodEncoder.getVelocity();
    }

    // Used to reset the hood
    public void setHoodSpeed(double speed) {
        // TODO: Ensure right direction of hood
        hood.set(speed);
        // hoodPID.setReference(speed, ControlType.kVelocity, 1);

    }

    public void stopHood() {
        hood.set(0);
    }

    public void resetHood() {
        hoodEncoder.setPosition(0);
        hoodReset = true;
    }

    public void disableHoodLimits() {
        hood.enableSoftLimit(SoftLimitDirection.kForward, false);
        hood.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void enableHoodLimits() {
        hood.enableSoftLimit(SoftLimitDirection.kForward, true);
        hood.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public double getHoodAngle() {
        return 360 * hoodEncoder.getPosition() / Constants.GEAR_RATIO_HOOD;
    }

    public HoodSubsystem() {
        setupMotors();
    }

    @Override
    public void periodic() {
    }
}
