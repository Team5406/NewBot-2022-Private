package frc.team5406.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5406.robot.Constants;

public class SwerveModule {

  private CANSparkMax driveMotor;
  private CANSparkMax rotationMotor;

  private RelativeEncoder driveRelEncoder;
  private RelativeEncoder rotationRelEncoder;

  private SparkMaxPIDController drivePIDController;
  private SparkMaxPIDController rotationPIDController;

  private CANCoder rotateAbsSensor;

  private double resetIteration = 0;



  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA);
  private final SimpleMotorFeedforward rotationFF = new SimpleMotorFeedforward(Constants.ROTATION_KS, Constants.ROTATION_KV, Constants.ROTATION_KA); 

  /**
   * Creates a new SwerveModule object
   * 
   * @param driveMotorID    The CAN ID of the SparkMAX connected to the drive
   * @param rotationMotorID The CAN ID of the SparkMax connected to the module
   *                        rotation motor
   * @param canCoderID      The CAN ID of the rotation sensor
   */
  public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID) {
    // Drive Motor:
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    driveRelEncoder = driveMotor.getEncoder();
    drivePIDController = driveMotor.getPIDController();

    driveMotor.restoreFactoryDefaults();

    driveMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERSION);
    driveMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    driveMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_MOTOR);
    driveMotor.setClosedLoopRampRate(0.05);

    drivePIDController.setP(Constants.DRIVE_PID0_P, 0);
    drivePIDController.setI(Constants.DRIVE_PID0_I, 0);
    drivePIDController.setD(Constants.DRIVE_PID0_D, 0);
    drivePIDController.setIZone(Constants.PID_IZ_VALUE, 0);
    drivePIDController.setFF(Constants.DRIVE_PID0_F, 0);
    drivePIDController.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    driveRelEncoder.setPositionConversionFactor(Constants.MK4_L2_DRIVE_REDUCTION * Math.PI * Constants.MK4_L2_WHEEL_DIAMETER);
    driveRelEncoder.setVelocityConversionFactor((Constants.MK4_L2_DRIVE_REDUCTION * Math.PI * Constants.MK4_L2_WHEEL_DIAMETER)/60);


    // Rotation Motor:
    rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
    rotationPIDController = rotationMotor.getPIDController();
    rotationRelEncoder = rotationMotor.getEncoder();

    rotationMotor.restoreFactoryDefaults();

    rotationMotor.setIdleMode(IdleMode.kCoast);
    rotationMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    rotationMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_ROTATION_MOTOR);
    rotationMotor.setInverted(Constants.ROTATION_MOTOR_INVERSION);

    rotationPIDController.setP(Constants.ROTATION_PID0_P, 0);
    rotationPIDController.setI(Constants.ROTATION_PID0_I, 0);
    rotationPIDController.setD(Constants.ROTATION_PID0_D, 0);
    rotationPIDController.setIZone(Constants.PID_IZ_VALUE, 0);
    rotationPIDController.setFF(Constants.ROTATION_PID0_F, 0);
    rotationPIDController.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
    rotationPIDController.setSmartMotionMaxAccel(Constants.ROTATION_PID0_MAX_ACCEL, 0);
    rotationPIDController.setSmartMotionMaxVelocity(Constants.ROTATION_PID0_MAX_VELOCITY, 0);

    rotateAbsSensor = new CANCoder(canCoderID);
    rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rotateAbsSensor.configSensorDirection(false);

    rotationRelEncoder.setPositionConversionFactor(2.0 * Math.PI * Constants.MK4_L2_ROTATION_REDUCTION);
    rotationRelEncoder.setVelocityConversionFactor(2.0 * Math.PI * Constants.MK4_L2_ROTATION_REDUCTION / 60.0);

    SmartDashboard.putNumber("P Gain", Constants.DRIVE_PID0_P);
    SmartDashboard.putNumber("kV", Constants.DRIVE_KV);
    SmartDashboard.putNumber("kA", Constants.DRIVE_KA);
    SmartDashboard.putNumber("kS", Constants.DRIVE_KS);


    resetEncoders();

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(rotationRelEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double cappedAngle = rotationRelEncoder.getPosition();
    if (cappedAngle > Math.PI) {
      cappedAngle -= 2.0 * Math.PI;
  } else if (cappedAngle < -Math.PI) {
      cappedAngle += 2.0 * Math.PI;
  }
  SmartDashboard.putNumber("Steer Desired Angle" + rotationMotor.getDeviceId(), desiredState.angle.getRadians());




    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(cappedAngle));

        SmartDashboard.putNumber("Steer Opt Angle" + rotationMotor.getDeviceId(), state.angle.getRadians());

    /*double rotationArbFF = rotationFF.calculate(
        ( rotationRelEncoder.getVelocity() * Constants.MK4_L2_ROTATION_REDUCTION) / Constants.SECONDS_PER_MINUTE);*/
    double driveArbFF = driveFF.calculate(state.speedMetersPerSecond);
    

    drivePIDController.setReference(state.speedMetersPerSecond, 
     ControlType.kVelocity, 0, driveArbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
        setReferenceAngle(state.angle.getRadians());
  }

  public void setReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians =rotationRelEncoder.getPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (rotationRelEncoder.getVelocity() < Constants.ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
        if (++resetIteration >= Constants.ENCODER_RESET_ITERATIONS) {
            resetIteration = 0;
            double absoluteAngle = getAbsoluteAngle();
            rotationRelEncoder.setPosition(absoluteAngle);
            currentAngleRadians = absoluteAngle;
        }
    } else {
        resetIteration = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
        currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
        adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    SmartDashboard.putNumber("Steer Target" + rotationMotor.getDeviceId(), adjustedReferenceAngleRadians);
    rotationPIDController.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition, 0);
}
/*
  public double getRotateAngleFromAbs(double absPos) {
    if (absPos > 180) {
      absPos -= 360;
    }
    return absPos / Constants.MK4_L2_ROTATION_REDUCTION;
  }*/

  public void setDriveP(){
  
     // display PID coefficients on SmartDashboard
     double p = SmartDashboard.getNumber("P Gain", 0);
     drivePIDController.setP(p, 0);

   }
 
   public void setDriveFF(){
  
    // display PID coefficients on SmartDashboard
    double kV = SmartDashboard.getNumber("kV", 0);
    double kA = SmartDashboard.getNumber("kA", 0);
    double kS = SmartDashboard.getNumber("kS", 0);
    driveFF = new SimpleMotorFeedforward(kS, kV, kA);


  }

  public double getAbsoluteAngle() {
    double angle = Math.toRadians(rotateAbsSensor.getAbsolutePosition());
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }

    return angle;
}

  /*public double getAbsRotatePosition() {
    return getRotateAngleFromAbs(rotateAbsSensor.getAbsolutePosition());
  }*/

  public void resetEncoders(){
    driveRelEncoder.setPosition(0);
    rotationRelEncoder.setPosition(getAbsoluteAngle());
  }

  public double getDriveSpeed() {
    return driveRelEncoder.getVelocity();
  }

}
