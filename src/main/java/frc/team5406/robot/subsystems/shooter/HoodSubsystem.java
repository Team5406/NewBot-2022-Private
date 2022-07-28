package frc.team5406.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private CANSparkMax hood = new CANSparkMax(Constants.MOTOR_HOOD_ONE, MotorType.kBrushless);
    private RelativeEncoder hoodEncoder;
    private SparkMaxPIDController hoodPID;

    public boolean hoodReset = false;

    public static double lltv = 0;
    public static double lltx = 0;
    public static double llty = 0;
    public static double llts = 0;
    public static double lldist = 0;

    public boolean llHasValidTarget = false;

    public void setupMotors() {
        hoodEncoder = hood.getEncoder();
        hoodPID = hood.getPIDController();

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

    public double getLLHoodPosition() {

        if (llHasValidTarget) {
            return calculateHoodAngle(llty);
        } else {
            return 0; // do this better
        }
    }

    public double calculateHoodAngle(double ty) {
        double hoodAngle = 2e-05 * Math.pow(ty, 4) - 0.0002 * Math.pow(ty, 3) - 0.0144 * Math.pow(ty, 2) - 0.4064 * ty
                + 14.482;

        if (hoodAngle > Constants.HOOD_MAX_LIMIT) {
            hoodAngle = Constants.HOOD_MAX_LIMIT;
        }
        if (hoodAngle < Constants.HOOD_MIN_LIMIT) {
            hoodAngle = Constants.HOOD_MIN_LIMIT;
        }
        return hoodAngle;
    }

    public void updateLimelightTracking() {
        lltv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        lltx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        llty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        llts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

        if (lltv < 1.0) {
            llHasValidTarget = false;

            return;
        }
        lldist = Units.inchesToMeters(Constants.LL_TARGET_HEIGHT - Constants.LL_MOUNT_HEIGHT)
                / Math.tan(Units.degreesToRadians(Constants.LL_MOUNT_ANGLE + llty));

        if (llts < -45) {
            llts += 90;
        }

        llHasValidTarget = true;

    }

    public HoodSubsystem() {
        setupMotors();
    }

    @Override
    public void periodic() {
      updateLimelightTracking();
    }
}
