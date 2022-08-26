package frc.team5406.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

   /* private final Translation2d m_frontLeftLocation = new Translation2d(0.22, 0.22);
    private final Translation2d m_frontRightLocation = new Translation2d(0.22, -0.22);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.22, 0.22);
    private final Translation2d m_backRightLocation = new Translation2d(-0.22, -0.22);*/


    private final Translation2d m_frontLeftLocation = new Translation2d(-0.22, 0.22);
    private final Translation2d m_frontRightLocation = new Translation2d(0.22, 0.22);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.22, -0.22);
    private final Translation2d m_backRightLocation = new Translation2d(0.22, -0.22);

    private final SwerveModule m_frontLeft = new SwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    private final SwerveModule m_frontRight = new SwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
    private final SwerveModule m_backLeft = new SwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER);
    private final SwerveModule m_backRight = new SwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER);

    private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200); // NavX connected over MXP

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d());

    public DriveSubsystem() {
        m_navx.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.K_MAX_SPEED);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Rotation2d getGyroscopeRotation() {

        if (m_navx.isMagnetometerCalibrated()) {
            // // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_navx.getAngle() * (Constants.GYRO_REVERSED ? -1.0 : 1.0));
    }

    public double getHeadingDegrees() {
        Rotation2d angle = m_navx.getRotation2d();
        SmartDashboard.putNumber("Heading-Degrees", angle.getDegrees());
        return getHeading().getDegrees();
    }

    public void reset() {
        zeroGyroscope();
        m_odometry.resetPosition(new Pose2d(), getHeading());
    }

    public void setPosition(Pose2d pose){
        zeroGyroscope();
        m_odometry.resetPosition(pose, getHeading());
    }

    public void setPValue(){
        m_frontLeft.setDriveP();
        m_frontRight.setDriveP();
        m_backLeft.setDriveP();
        m_backRight.setDriveP();
    }

    public void setFFValue(){
        m_frontLeft.setDriveFF();
        m_frontRight.setDriveFF();
        m_backLeft.setDriveFF();
        m_backRight.setDriveFF();
    }

    //Used for Autos.
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_METERS_PER_SECOND);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public Rotation2d desiredRotation(double start, double end, double startTime, double totalTime){
        Rotation2d setPoint = Rotation2d.fromDegrees(start+(end-start)*((Timer.getFPGATimestamp()-startTime)/totalTime));
        SmartDashboard.putNumber("desiredRot", setPoint.getDegrees());
        return setPoint;
    }
    
    public void setTurnStates(double speed) {
            var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                             ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, speed, m_navx.getRotation2d()));
        setModuleStates(swerveModuleStates);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_navx.getRotation2d(),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }
    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Distance Travelled X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Distance Travelled Y", m_odometry.getPoseMeters().getY());
        

// Convert to chassis speeds
ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
    m_frontLeft.getState(),  m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());

// Getting individual speeds
double forward = chassisSpeeds.vxMetersPerSecond;
double sideways = chassisSpeeds.vyMetersPerSecond;
double angular = chassisSpeeds.omegaRadiansPerSecond;

SmartDashboard.putNumber("Speed Forward", forward);
SmartDashboard.putNumber("Speed Sideways", sideways);
SmartDashboard.putNumber("Rotation Y", angular);

getHeadingDegrees();


    }
}
