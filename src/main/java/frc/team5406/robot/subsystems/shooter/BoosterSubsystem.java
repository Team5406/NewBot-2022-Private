package frc.team5406.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class BoosterSubsystem extends SubsystemBase {
    private CANSparkMax booster = new CANSparkMax(Constants.MOTOR_BOOSTER_ONE, MotorType.kBrushless);
    private RelativeEncoder boosterEncoder;
    private SparkMaxPIDController boosterPID;

    public void setupMotors(){
        boosterEncoder = booster.getEncoder();
        boosterPID = booster.getPIDController();
        booster.setSmartCurrentLimit(Constants.CURRENT_LIMIT_BOOSTER);

        booster.setInverted(true);

        resetEncoders();
        booster.burnFlash();
    }
    
    public void resetEncoders(){
        boosterEncoder.setPosition(0);
    }

    public void setBoosterSpeed(double RPM){
        if (RPM == 0) {
            booster.set(0);
          } else {
           // double arbFF = boosterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            boosterPID.setReference(RPM *  Constants.GEAR_RATIO_BOOSTER, ControlType.kVelocity, 1);
          }      
        }

    public void stopBooster() {
        booster.set(0);
    }

    public double getBoosterSpeed() {
        return boosterEncoder.getVelocity() * 1 / Constants.GEAR_RATIO_BOOSTER;
    }

    public BoosterSubsystem(){
        setupMotors();
    }
}
