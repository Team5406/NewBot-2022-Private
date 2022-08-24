package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithLimelight;
import frc.team5406.robot.commands.IntakeCommand;
import frc.team5406.robot.commands.ManualSetShooter;
import frc.team5406.robot.commands.ResetHoodEncoder;
import frc.team5406.robot.commands.SetShooter;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.TurnToAngle;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;
import frc.team5406.robot.subsystems.LimelightSubsystem;

public class FiveBall {

        private final DriveSubsystem drive;
        private final FeederSubsystem feeder;
        private final IntakeSubsystem intake;
        private final BoosterSubsystem booster;
        private final FlywheelSubsystem flywheel;
        private final HoodSubsystem hood;
        private final FrontGateSubsystem frontGate;
        private final BackGateSubsystem backGate;
        private final LimelightSubsystem limelight;

    public FiveBall(DriveSubsystem _drive, FeederSubsystem _feeder, IntakeSubsystem _intake, 
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
                new Pose2d(0, Units.inchesToMeters(36), new Rotation2d(Units.degreesToRadians(315))),
                List.of(

                ),
                new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(-4), new Rotation2d(Units.degreesToRadians(315))),
                fwdConfig);


        ProfiledPIDController thetaController2 = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController2.enableContinuousInput(-Math.PI, Math.PI);     


        Trajectory path3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(-4), new Rotation2d(Units.degreesToRadians(15))),
                List.of(

                ),
                new Pose2d(Units.inchesToMeters(233), Units.inchesToMeters(10), new Rotation2d(Units.degreesToRadians(15))),
                fwdConfig);


        ProfiledPIDController thetaController3 = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController3.enableContinuousInput(-Math.PI, Math.PI);     


        Trajectory path4 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(233), Units.inchesToMeters(10), new Rotation2d(Units.degreesToRadians(200))),
                List.of(

                ),
                new Pose2d(Units.inchesToMeters(108), Units.inchesToMeters(4), new Rotation2d(Units.degreesToRadians(200))),
                fwdConfig);


        ProfiledPIDController thetaController4 = new ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController4.enableContinuousInput(-Math.PI, Math.PI);     
        

        drive.setPosition(path1.getInitialPose());


        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new WaitCommand(1.5),
                                new ResetHoodEncoder(hood)
                        ),
                        new ParallelDeadlineGroup(
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
                                        new ManualSetShooter(flywheel, hood, 2344, 15.6)
                        ),
                        new TurnToAngle(-4, drive),
                        new ParallelDeadlineGroup(
                                new AlignWithLimelight(drive, limelight),
                                new SetShooter(flywheel, hood, limelight)
                        ),
                        new ParallelRaceGroup(
                                new WaitCommand(1),
                                new Shoot(booster, feeder)
                        ),
                        new ParallelDeadlineGroup(
                                new SwerveControllerCommand(
                                        path2,
                                        drive::getPose,
                                        drive.m_kinematics,
                                        new PIDController(Constants.kPXController, 0, 0),
                                        new PIDController(Constants.kPYController, 0, 0),
                                         thetaController2,
                                        () -> drive.desiredRotation(4, 90, Timer.getFPGATimestamp(), path2.getTotalTimeSeconds()),
                                        drive::setModuleStates,
                                        drive)
                                        .andThen(() -> drive.drive(0, 0, 0, false)), 
                                new ManualSetShooter(flywheel, hood, 2344, 15.6)
                        ), 
                        new TurnToAngle(-40, drive),
                        new ParallelDeadlineGroup(
                                new AlignWithLimelight(drive, limelight),
                                new SetShooter(flywheel, hood, limelight)
                        ),
                        new ParallelRaceGroup(
                                new WaitCommand(1),
                                new Shoot(booster, feeder)
                        ),
                        new SwerveControllerCommand(
                            path3,
                            drive::getPose,
                            drive.m_kinematics,
                            new PIDController(Constants.kPXController, 0, 0),
                            new PIDController(Constants.kPYController, 0, 0),
                            thetaController3,
                            () -> drive.desiredRotation(40, 45, Timer.getFPGATimestamp(), path3.getTotalTimeSeconds()),
                            drive::setModuleStates,
                            drive)
                            .andThen(() -> drive.drive(0, 0, 0, false)),
                        
                        new ParallelDeadlineGroup(
                                new SwerveControllerCommand(
                                        path4,
                                        drive::getPose,
                                        drive.m_kinematics,
                                        new PIDController(Constants.kPXController, 0, 0),
                                        new PIDController(Constants.kPYController, 0, 0),
                                        thetaController4,
                                        () -> drive.desiredRotation(45, 57, Timer.getFPGATimestamp(), path4.getTotalTimeSeconds()),
                                        drive::setModuleStates,
                                        drive)
                                        .andThen(() -> drive.drive(0, 0, 0, false)), 
                                new ManualSetShooter(flywheel, hood, 2344, 15.6)
                        ),
                        new TurnToAngle(-57, drive),
                        new ParallelDeadlineGroup(
                                new AlignWithLimelight(drive, limelight),
                                new SetShooter(flywheel, hood, limelight)
                        ),
                        new ParallelRaceGroup(
                                new WaitCommand(1),
                                new Shoot(booster, feeder)
                        )
                ),
                new IntakeCommand(intake, feeder, backGate, frontGate)
        );




    }

}
