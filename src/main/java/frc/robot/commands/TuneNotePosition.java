package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class TuneNotePosition extends Command {
    private Intake intake;

    public TuneNotePosition(Intake intake, DigitalInput FrontSensor, DigitalInput RearSensor) {
        this.intake=intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }
}
