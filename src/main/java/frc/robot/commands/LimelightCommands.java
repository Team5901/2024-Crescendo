package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.LimelightIO.LimelightIOInputs;
import frc.robot.subsystems.drive.LimelightIONetwork;

public class LimelightCommands extends Command {

  public static Command updateInputs(LimelightIOInputs inputs, LimelightIONetwork Limelight) {

    return Commands.run(() -> Limelight.updateInputs(inputs));
  }
}
