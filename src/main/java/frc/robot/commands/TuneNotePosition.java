package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class TuneNotePosition extends Command {
  private Intake intake;
  private DigitalInput FrontSensor, RearSensor;
  private boolean front, rear, killSwitch, passedRear, change1, change2, change3;
  private double inSpeed = Constants.IntakeSubsystem.intakeInNoteVelRPM;
  private double outSpeed = -Constants.IntakeSubsystem.intakeInNoteVelRPM;
  private double inSpeedSlow = inSpeed / 2;
  private double outSpeedSlow = outSpeed / 3;

  public TuneNotePosition(Intake intake, DigitalInput FrontSensor, DigitalInput RearSensor) {
    this.intake = intake;
    this.FrontSensor = FrontSensor;
    this.RearSensor = RearSensor;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    killSwitch = false;
    passedRear = false;
    change1 = false;
    change2 = false;
    change3 = false;
  }

  @Override
  public void execute() {
    front = !FrontSensor.get();
    rear = !RearSensor.get();
    SmartDashboard.putNumber("Front", front ? 1d : 0d);
    SmartDashboard.putNumber("Rear", rear ? 1d : 0d);

    if (!front && !rear
        && !change1) { // here, we havent triggered either sensor, so move inward at full speed;
      intake.runVelocity(inSpeed); // default speed
      change1 =
          true; // these variables are used as a "latch" they allow each if statement to run only
      // one time.
    }
    if (front
        && !rear
        && !passedRear
        && !change2) { // if we have triggered front sensor, not the back sensor, and we never hit
      // the back sensor move in slower;
      intake.runVelocity(
          inSpeedSlow); // go in slower, no need to rush with a note this far in our robot.
      change2 = true;
    }
    if (rear && !change3) { // if we have finally hit the back sensor, move outward slowly
      passedRear = true; // a flag that means what is says: we have passed the rear;
      intake.runVelocity(outSpeedSlow); // go outward very slowly
      change3 = true;
    }
    if (front
        && !rear
        && passedRear) { // if we are back in between the two sensors, AND we hit the back sensor,
      // we're done!
      killSwitch = true; // gets used in isFinished()
      intake.stop();
    }
    // most loops this code runs, it doesn't do anything at all.
    // once we set our motor spped, we don't need to keep setting it every robot periodic loop
    // (20ms)
    // by adding these latches, the Can bus will likely be used fewer times. If theres issues, we
    // can simply remove them

  }

  @Override
  public void end(boolean Interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return killSwitch;
    // see logic above for end condition.
  }
}
