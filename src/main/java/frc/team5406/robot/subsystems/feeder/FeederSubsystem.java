package frc.team5406.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax conveyorTop = new CANSparkMax(Constants.MOTOR_CONVEYOR_TOP_ONE, MotorType.kBrushless);
  private CANSparkMax conveyorBottom = new CANSparkMax(Constants.MOTOR_CONVEYOR_BOTTOM_ONE, MotorType.kBrushless);

  private RelativeEncoder conveyorTopEncoder, conveyorBottomEncoder;

  private SparkMaxPIDController conveyorTopPID, conveyorBottomPID;

  private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public void setupMotors() {
    conveyorTop.restoreFactoryDefaults();
    conveyorBottom.restoreFactoryDefaults();

    conveyorTopEncoder = conveyorTop.getEncoder();
    conveyorBottomEncoder = conveyorBottom.getEncoder();

    conveyorTopPID = conveyorTop.getPIDController();
    conveyorBottomPID = conveyorBottom.getPIDController();

    conveyorTop.setIdleMode(IdleMode.kCoast);
    conveyorTop.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CONVEYOR_TOP);
    conveyorTop.setInverted(Constants.CONVEYOR_TOP_MOTOR_INVERSION);

    conveyorTopPID.setP(Constants.CONVEYOR_TOP_PID1_P, 0);
    conveyorTopPID.setI(Constants.CONVEYOR_TOP_PID1_I, 0);
    conveyorTopPID.setD(Constants.CONVEYOR_TOP_PID1_D, 0);
    conveyorTopPID.setIZone(Constants.PID_IZ_VALUE, 0);
    conveyorTopPID.setFF(Constants.CONVEYOR_TOP_PID1_F, 0);

    conveyorBottom.setIdleMode(IdleMode.kCoast);
    conveyorBottom.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CONVEYOR_BOTTOM);
    conveyorBottom.setInverted(Constants.CONVEYOR_BOTTOM_MOTOR_INVERSION);

    conveyorBottomPID.setP(Constants.CONVEYOR_BOTTOM_PID1_P, 0);
    conveyorBottomPID.setI(Constants.CONVEYOR_BOTTOM_PID1_I, 0);
    conveyorBottomPID.setD(Constants.CONVEYOR_BOTTOM_PID1_D, 0);
    conveyorBottomPID.setIZone(Constants.PID_IZ_VALUE, 0);
    conveyorBottomPID.setFF(Constants.CONVEYOR_BOTTOM_PID1_F, 0);


    conveyorTopEncoder.setPositionConversionFactor(Constants.GEAR_RATIO_CONVEYOR_TOP);
    conveyorTopEncoder.setVelocityConversionFactor(Constants.GEAR_RATIO_CONVEYOR_TOP);

    conveyorBottomEncoder.setPositionConversionFactor(Constants.GEAR_RATIO_CONVEYOR_BOTTOM);
    conveyorBottomEncoder.setVelocityConversionFactor(Constants.GEAR_RATIO_CONVEYOR_BOTTOM);

    resetEncoders();
    conveyorTop.burnFlash();
    conveyorBottom.burnFlash();
  }



  public void resetEncoders() {
    conveyorTopEncoder.setPosition(0);
    conveyorBottomEncoder.setPosition(0);
  }

  public void disableCompressor() {
    compressor.disable();
  }

  public void enableCompressor() {
    compressor.enableDigital();
  }

  public void setConveyorTopSpeed(double RPM) {
    // RPM*=0.98;
    if (RPM == 0) {
      stopConveyorTop();

    } else {
      // double arbFF = shooterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
      conveyorTopPID.setReference(RPM * Constants.GEAR_RATIO_CONVEYOR_TOP, ControlType.kVelocity, 1);
    }

  }

  public void setConveyorBottomSpeed(double RPM) {
    // RPM*=0.98;
    if (RPM == 0) {
      stopConveyorBottom();
    } else {
      // double arbFF = shooterFF.calculate(RPM/Constants.SECONDS_PER_MINUTE);
      conveyorBottomPID.setReference(RPM * Constants.GEAR_RATIO_CONVEYOR_BOTTOM, ControlType.kVelocity, 1);
    }
  }

  public void stopConveyorTop() {
    conveyorTop.set(0);
  }

  public void stopConveyorBottom() {
    conveyorBottom.set(0);
  }

  public void stopConveyors() {
    stopConveyorTop();
    stopConveyorBottom();
  }

  public FeederSubsystem() {
    setupMotors();
  }

}
