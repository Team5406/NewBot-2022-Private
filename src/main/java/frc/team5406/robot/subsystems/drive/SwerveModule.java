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

  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV); //Values from random GitHub Repo - Probably should fix this.
  private final SimpleMotorFeedforward rotationFF = new SimpleMotorFeedforward(Constants.ROTATION_KS, Constants.ROTATION_KV); 

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
    driveMotor.setInverted(Constants.DIRVE_MOTOR_INVERSION);
    driveMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    driveMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_MOTOR);

    drivePIDController.setP(Constants.DRIVE_PID0_P, 0);
    drivePIDController.setI(Constants.DRIVE_PID0_I, 0);
    drivePIDController.setD(Constants.DRIVE_PID0_D, 0);
    drivePIDController.setIZone(Constants.PID_IZ_VALUE, 0);
    drivePIDController.setFF(Constants.DRIVE_PID0_F, 0);
    drivePIDController.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);


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
    rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

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
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(rotationRelEncoder.getPosition()));

    double rotationArbFF = rotationFF.calculate(
        (rotationRelEncoder.getVelocity() * Constants.MK4_L2_ROTATION_REDUCTION) / Constants.SECONDS_PER_MINUTE);
    double driveArbFF = driveFF.calculate(state.speedMetersPerSecond);
    

    drivePIDController.setReference(state.speedMetersPerSecond * Constants.MK4_L2_DRIVE_REDUCTION,
        ControlType.kVelocity, 0, driveArbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
    rotationPIDController.setReference(
        Units.radiansToRotations(state.angle.getRadians()) * Constants.MK4_L2_ROTATION_REDUCTION,
        ControlType.kSmartMotion, 0, rotationArbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public double getRotateAngleFromAbs(double absPos) {
    if (absPos > 180) {
      absPos -= 360;
    }
    return absPos / Constants.MK4_L2_ROTATION_REDUCTION;
  }

  public double getAbsRotatePosition() {
    return getRotateAngleFromAbs(rotateAbsSensor.getAbsolutePosition());
  }

  public void resetEncoders(){
    driveRelEncoder.setPosition(0);
    rotationRelEncoder.setPosition((getAbsRotatePosition() * Constants.MK4_L2_ROTATION_REDUCTION) / 360);
  }

  public double getDriveSpeed() {
    return Units.inchesToMeters(((driveRelEncoder.getVelocity() / Constants.MK4_L2_DRIVE_REDUCTION) * Math.PI * Constants.MK4_L2_WHEEL_DIAMETER)
        / Constants.SECONDS_PER_MINUTE);
  }

}
