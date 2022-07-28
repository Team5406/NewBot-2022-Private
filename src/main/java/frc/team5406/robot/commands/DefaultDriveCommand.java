package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DriveSubsystem m_swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DriveSubsystem swerve, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
                
        m_swerve = swerve;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        m_swerve.drive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(), true);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, true);
    }
}