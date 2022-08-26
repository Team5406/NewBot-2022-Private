package frc.team5406.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class FlywheelSubsystem extends SubsystemBase {
    private CANSparkMax shooter = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_ONE, MotorType.kBrushless);
    private CANSparkMax shooterFollower = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_TWO,  MotorType.kBrushless);
    
    private RelativeEncoder shooterEncoder;
    private SparkMaxPIDController shooterPID;

    static SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);

    public void setupMotors() {
        shooterEncoder = shooter.getEncoder();
        shooterPID = shooter.getPIDController();

        shooterPID.setP(Constants.SHOOTER_PID0_P, 0);
        shooterPID.setI(Constants.SHOOTER_PID0_I, 0);
        shooterPID.setD(Constants.SHOOTER_PID0_D, 0);
        shooterPID.setIZone(0, 0);
        shooterPID.setFF(Constants.SHOOTER_PID0_F, 0);
        shooterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

        shooter.setSmartCurrentLimit(Constants.CURRENT_LIMIT_SHOOTER_WHEEL);
        shooterFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_SHOOTER_WHEEL);

        shooter.setInverted(false);
        shooterFollower.follow(shooter, true);

        resetEncoders();
        shooter.burnFlash();
        shooterFollower.burnFlash();
    }

    public void resetEncoders(){
        shooterEncoder.setPosition(0);
    }

    public void setShooterSpeed(double RPM) {
        //RPM*=0.98;
          if (RPM == 0) {
            stopShooter();
    
        } else {
          double arbFF = shooterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
          shooterPID.setReference(RPM *  Constants.GEAR_RATIO_SHOOTER, ControlType.kVelocity, 0, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
    
      }

    public void stopShooter(){
        shooter.set(0);
    }

    public double getShooterSpeed() {
        return shooterEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_SHOOTER;
    }

    public FlywheelSubsystem(){
        setupMotors();
    }
}
