package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class FiveBall {

    private final DriveSubsystem drive;

    public FiveBall(DriveSubsystem subsystem) {
        drive = subsystem;
        SmartDashboard.putNumber("P-drv Gain", Constants.kPXController);
        SmartDashboard.putNumber("Auto Dist", 4);

    }

    public Command getAutonomousCommand() {

        drive.reset();

        var autoVoltageConstraint = new SwerveDriveKinematicsConstraint(drive.m_kinematics,
                Constants.MAX_SPEED_METERS_PER_SECOND);

        TrajectoryConfig fwdConfig = new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(drive.m_kinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(false);

        TrajectoryConfig backConfig = new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(drive.m_kinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);


        Trajectory path1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(90))),
                List.of(

                ),
                new Pose2d(0, Units.inchesToMeters(36), new Rotation2d(Units.degreesToRadians(90))),
                fwdConfig);


        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);     


        Trajectory path2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, Units.inchesToMeters(36), new Rotation2d(Units.degreesToRadians(90))),
                List.of(

                ),
                new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(-4), new Rotation2d(Units.degreesToRadians(90))),
                fwdConfig);


        ProfiledPIDController thetaController2 = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController2.enableContinuousInput(-Math.PI, Math.PI);     

        

        drive.setPosition(path1.getInitialPose());


        return   
        new SequentialCommandGroup(
            new SwerveControllerCommand(
                path1,
                drive::getPose,
                drive.m_kinematics,
                new PIDController(Constants.kPXController, 0, 0),
                new PIDController(Constants.kPYController, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive)
                .andThen(() -> drive.drive(0, 0, 0, false)),
            new SwerveControllerCommand(
                path2,
                drive::getPose,
                drive.m_kinematics,
                new PIDController(Constants.kPXController, 0, 0),
                new PIDController(Constants.kPYController, 0, 0),
                thetaController2,
                drive::setModuleStates,
                drive)
                .andThen(() -> drive.drive(0, 0, 0, false))
            );



    }

}
