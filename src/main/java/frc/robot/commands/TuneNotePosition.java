package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;

public class TuneNotePosition extends Command {
    private Intake intake;
    private DigitalInput FrontSensor,RearSensor;
    private boolean front,rear,changeFlag,killSwitch,goodOnFirstCheck;
    private double inSpeed=100;
    private double outSpeed=-100;
    private Timer timecheck;
    private double goalTime=0.25; // seconds to wait 
    public TuneNotePosition(Intake intake, DigitalInput FrontSensor, DigitalInput RearSensor) {
        this.intake=intake;
        this.FrontSensor=FrontSensor;
        this.RearSensor=RearSensor;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        timecheck = new Timer();
        changeFlag= false;
        killSwitch=false;
        goodOnFirstCheck=true;
        
    }

    @Override
    public void execute() {
        front = FrontSensor.get();
        rear= RearSensor.get();
        if (front == rear) {
            timecheck.stop();
            timecheck.reset();
            goodOnFirstCheck=false;
            changeFlag=true;

        }
        if (!front && !rear) {
            intake.runVelocity(inSpeed); // not in the intake? try to pull notes in
        } else if (front && rear) {
            intake.runVelocity(outSpeed); // too far? try to spit it out a bit.
            
        } else {
            timecheck.start();
            intake.stop();
            if (changeFlag) { // this flag only turns true when we exit our goal area, then is true for one loop while we are in our goal;
            inSpeed=inSpeed*0.75;// ONE time within our goal area, we slow down the motors a bit.
            outSpeed=outSpeed*0.75;
            changeFlag=false;
            if (goodOnFirstCheck) { //if  the operator runs this command while there is a note in our goal zone, end immediately;
                killSwitch = true;
            }

            }
        }
    }
    @Override
    public void end(boolean Interrupted) {
        if (Interrupted) {
            intake.stop();
        }
        
    }

    @Override
    public boolean isFinished() {
        return (timecheck.get() > goalTime) || killSwitch; //if were inside our goal tolerance for a certain amount of time, then call it good and quit
        
        
    }
}
