package frc.team5406.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class FlywheelSubsystem extends SubsystemBase {
    private CANSparkMax shooter = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_ONE, MotorType.kBrushless);
    private CANSparkMax shooterFollower = new CANSparkMax(Constants.MOTOR_SHOOTER_WHEEL_TWO,  MotorType.kBrushless);
    
    private RelativeEncoder shooterEncoder;
    private SparkMaxPIDController shooterPID;

    public void setupMotors() {
        shooterEncoder = shooter.getEncoder();
        shooterPID = shooter.getPIDController();

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

    public FlywheelSubsystem(){
        setupMotors();
    }
}
