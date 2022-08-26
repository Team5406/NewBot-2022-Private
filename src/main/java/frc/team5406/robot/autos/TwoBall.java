package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithLimelight;
import frc.team5406.robot.commands.FeedInCommand;
import frc.team5406.robot.commands.GateBottomOpen;
import frc.team5406.robot.commands.GateTopClose;
import frc.team5406.robot.commands.GateTopOpen;
import frc.team5406.robot.commands.IntakeDeployCommand;
import frc.team5406.robot.commands.ManualSetShooter;
import frc.team5406.robot.commands.ResetHoodEncoder;
import frc.team5406.robot.commands.SetShooter;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.Shoot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.team5406.robot.subsystems.LimelightSubsystem;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;

public class TwoBall {

    private final DriveSubsystem drive;
    private final FeederSubsystem feeder;
    private final IntakeSubsystem intake;
    private final BoosterSubsystem booster;
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final FrontGateSubsystem frontGate;
    private final BackGateSubsystem backGate;
    private final LimelightSubsystem limelight;

    public TwoBall(DriveSubsystem _drive, FeederSubsystem _feeder, IntakeSubsystem _intake, 
    BoosterSubsystem _booster, FlywheelSubsystem _flywheel, HoodSubsystem _hood, FrontGateSubsystem _frontGate, 
    BackGateSubsystem _backGate, LimelightSubsystem _limelight) {
        drive = _drive;
        feeder = _feeder;
        intake = _intake; 
        booster = _booster;
        flywheel = _flywheel;
        hood = _hood;
        frontGate = _frontGate;
        backGate = _backGate;
        limelight = _limelight;

    }

    public Command getAutonomousCommand() {
         drive.reset();
        /*
        var autoVoltageConstraint = new SwerveDriveKinematicsConstraint(drive.m_kinematics,
                Constants.MAX_SPEED_METERS_PER_SECOND);*/

        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(drive.m_kinematics);
              //  .addConstraint(autoVoltageConstraint);
        config.setReversed(false);

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
        
     
        // Reset odometry to the starting pose of the trajectory.
        drive.setPosition(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(2),
                                new ResetHoodEncoder(hood),
                                new IntakeDeployCommand(intake),
                                new GateBottomOpen(frontGate),
                                new GateTopClose(backGate)
                        ),
                        new ParallelDeadlineGroup(
                                new SwerveControllerCommand(
                                        exampleTrajectory,
                                        drive::getPose,
                                        drive.m_kinematics,
                                        // Position controllers
                                        new PIDController(Constants.kPXController, 0, 0),
                                        new PIDController(Constants.kPYController, 0, 0),
                                        thetaController,
                                        drive::setModuleStates,
                                        drive).andThen(() -> drive.drive(0, 0, 0, false)),
                                new ManualSetShooter(flywheel, hood, booster, 2344, 15.6)
                        ),
                        new AlignWithLimelight(drive, limelight),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                    new WaitCommand(2),
                                    new SetShooter(flywheel, hood, booster, limelight)
                                )
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(1),  
                                new Shoot(backGate)
                        )

                ), new FeedInCommand(feeder)
        
        );

    }

}
