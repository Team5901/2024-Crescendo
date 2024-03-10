package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shoot.Shoot;

public class setShooterRPM extends Command {

    private Shoot shoot;
    private double setpointRPM;
    public setShooterRPM(double setpointRPM, Shoot shoot) {
        this.setpointRPM=setpointRPM;
        this.shoot=shoot;
        addRequirements(shoot);
    }
    @Override
    public void initialize() {
        shoot.runVelocity(setpointRPM);
    }
    @Override
    public boolean isFinished() {
        return Math.abs(shoot.getVelocityRPM() - setpointRPM) <Constants.ShootSubsystem.goalToleranceVelocity;
    }
}
