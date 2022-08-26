package frc.team5406.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class BoosterSubsystem extends SubsystemBase {
    private CANSparkMax booster = new CANSparkMax(Constants.MOTOR_BOOSTER_ONE, MotorType.kBrushless);
    private RelativeEncoder boosterEncoder;
    private SparkMaxPIDController boosterPID;

    static SimpleMotorFeedforward boosterFF = new SimpleMotorFeedforward(Constants.BOOSTER_KS, Constants.BOOSTER_KV, Constants.BOOSTER_KA);

    public void setupMotors(){
        boosterEncoder = booster.getEncoder();
        boosterPID = booster.getPIDController();
        booster.setSmartCurrentLimit(Constants.CURRENT_LIMIT_BOOSTER);

        boosterPID.setP(Constants.BOOSTER_PID0_P, 0);
        boosterPID.setI(Constants.BOOSTER_PID0_I, 0);
        boosterPID.setD(Constants.BOOSTER_PID0_D, 0);
        boosterPID.setIZone(0, 0);
        boosterPID.setFF(Constants.BOOSTER_PID0_F, 0);
        boosterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

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
            double arbFF = boosterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
            boosterPID.setReference(RPM *  Constants.GEAR_RATIO_BOOSTER, ControlType.kVelocity, 0, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
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
