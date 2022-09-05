package frc.team5406.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    public boolean llHasValidTarget = false;

    public double lltv = 0;
    public double lltx = 0;
    public double llty = 0;
    public double llts = 0;
    public double lldist = 0;

    private static boolean shooting = false;
    private static double shooterSpeed = 0;
    private static double hoodAngle = 0;

    public void turnOffLimelight() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void turnOnLimelight() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
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

    public void startShoot(){
        shooting = true;
    }

    public void endShoot(){
        shooting = false;
    }

    public static double calculateHoodAngle(double ty) {
        if (!shooting){
            hoodAngle =  0.0007 * Math.pow(ty, 3) +0.0153 * Math.pow(ty, 2) -0.5161 * ty + 9.1421;
        }

        if (hoodAngle > Constants.HOOD_MAX_LIMIT) {
            hoodAngle = Constants.HOOD_MAX_LIMIT;
        }
        if (hoodAngle < Constants.HOOD_MIN_LIMIT) {
            hoodAngle = Constants.HOOD_MIN_LIMIT;
        }
        return hoodAngle;
    }

    public double getLLShooterSpeed() {

        if(shooting){
            return 1.005*shooterSpeed;
        } else if (llHasValidTarget) {
            return calculateShooterSpeed(llty) ;
        } else {
            return Constants.FLYWHEEL_SPEED_FENDER_HIGH; // Assume the LL can't find the target, shoot for fender.
        }
    }

    public double getLLtx() {
        updateLimelightTracking();
        return lltx;
    }

    public static double calculateShooterSpeed(double ty) {
        if(!shooting){
            shooterSpeed = 1.6523 * Math.pow(ty, 2) + 15.557 * ty + 2366;
        }
        if (shooterSpeed > Constants.SHOOTER_MAX_SPEED) {
            shooterSpeed = Constants.SHOOTER_MAX_SPEED;
        }
        if (shooterSpeed < 0) {
            shooterSpeed = 0;
        }
        return 1.005*shooterSpeed;
    }

    public boolean LLHasValidTarget() {
        return llHasValidTarget;
    }

    public double getLLHoodPosition() {
        if(shooting){
            return hoodAngle;
        }else if (llHasValidTarget) {
            return calculateHoodAngle(llty);
        } else {
            return Constants.HOOD_ANGLE_FENDER_HIGH;
        }
    }

    @Override
    public void periodic() {
        updateLimelightTracking();
    }

}
