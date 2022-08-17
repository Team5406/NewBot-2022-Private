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

public class TwoBall {

    private final DriveSubsystem drive;

    public TwoBall(DriveSubsystem subsystem) {
        drive = subsystem;
        SmartDashboard.putNumber("P-drv Gain", Constants.kPXController);
        SmartDashboard.putNumber("Auto Dist", 4);

    }

    public Command getAutonomousCommand() {

         drive.reset();
         drive.setPValue();
         drive.setFFValue();
        /*
        var autoVoltageConstraint = new SwerveDriveKinematicsConstraint(drive.m_kinematics,
                Constants.MAX_SPEED_METERS_PER_SECOND);*/

        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(drive.m_kinematics);
              //  .addConstraint(autoVoltageConstraint);
        config.setReversed(false);
        double dist = SmartDashboard.getNumber("Auto Dist", 0);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(90))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(

                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 1.19, new Rotation2d(Units.degreesToRadians(90))),
                config);

        var thetaController = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        double p = SmartDashboard.getNumber("P-drv Gain", 0);
        
        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
                exampleTrajectory,
                drive::getPose,
                drive.m_kinematics,
                // Position controllers
                new PIDController(p, 0, 0),
                new PIDController(p, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive);
        // Reset odometry to the starting pose of the trajectory.
        drive.setPosition(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveCommand.andThen(() -> drive.drive(0, 0, 0, false));
    }

}