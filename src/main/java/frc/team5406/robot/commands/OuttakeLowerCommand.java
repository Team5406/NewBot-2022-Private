package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeLowerCommand extends CommandBase {
    IntakeSubsystem intake;
    FeederSubsystem feeder;
    DoubleSupplier analog;

    public OuttakeLowerCommand(IntakeSubsystem _intake, FeederSubsystem _feeder) {
        intake = _intake;
        feeder = _feeder;
        addRequirements(intake, feeder);
    }

    /*public IntakeCommand(IntakeSubsystem _intake, FeederSubsystem _feeder, DoubleSupplier _analog){
        intake = _intake;
        feeder = _feeder;
        analog = _analog;
        addRequirements(intake, feeder);
    }*/

    @Override
    public void execute() {
        intake.intakeExtend();
        //feeder.setConveyorBottomSpeed(analog.getAsDouble() > 0.1 ? analog.getAsDouble() : 1000); // FIXME
        feeder.setConveyorBottomSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_BOTTOM);
        feeder.setConveyorTopSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_TOP);
        feeder.backGateExtend();
        feeder.frontGateExtend();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Outtake - End");
        intake.intakeRetract();
        feeder.stopConveyorBottom();
        feeder.stopConveyorTop();
        feeder.backGateExtend();
        feeder.frontGateRetract();        
    }
}
