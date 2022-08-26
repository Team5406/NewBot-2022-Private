package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;

public class FeedOutCommand extends CommandBase {
    FeederSubsystem feeder;

    public FeedOutCommand(FeederSubsystem _feeder) {
        feeder = _feeder;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setConveyorBottomSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_BOTTOM);
        feeder.setConveyorTopSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_TOP);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Feed - End");
        feeder.stopConveyorBottom();
        feeder.stopConveyorTop();
    }
}
