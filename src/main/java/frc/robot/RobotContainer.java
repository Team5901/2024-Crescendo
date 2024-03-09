// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmMovement;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.LimelightIO.LimelightIOInputs;
import frc.robot.subsystems.drive.LimelightIONetwork;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shoot.ShootIO;
import frc.robot.subsystems.shoot.ShootIOSim;
import frc.robot.subsystems.shoot.ShootIOSparkMax;
import frc.robot.subsystems.slider.Slider;
import frc.robot.subsystems.slider.SliderIO;
import frc.robot.subsystems.slider.SliderIOSim;
import frc.robot.subsystems.slider.SliderIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Shoot shoot;
  private final LimelightIONetwork limelight;
  private final LimelightIOInputs inputs;
  private final Arm arm;
  private final Slider slider;
  // Commands
  private final ArmMovement armMovementCommand;
  // Controller and joystick
  private final Joystick joystick = new Joystick(0);
  private final XboxController controller_2 = new XboxController(1);

  // Intake movement buttons
  private final JoystickButton aimAmp =
      new JoystickButton(controller_2, XboxController.Button.kA.value);
  private final JoystickButton intakeOut =
      new JoystickButton(controller_2, XboxController.Button.kB.value);
  private final JoystickButton aimSpeaker =
      new JoystickButton(controller_2, XboxController.Button.kY.value);
  private final JoystickButton intakeIn =
      new JoystickButton(controller_2, XboxController.Button.kX.value);
  private final JoystickButton aimCustom =
      new JoystickButton(controller_2, XboxController.Button.kStart.value);

  // shooting/roller buttons
  private final JoystickButton IntakeRollersOn =
      new JoystickButton(controller_2, XboxController.Button.kLeftBumper.value);
  // Trigger triggerOperatorLeft = new Trigger(() -> controller_2.getLeftTriggerAxis() > 0.25);
  private final JoystickButton shootAmp =
      new JoystickButton(controller_2, XboxController.Axis.kLeftTrigger.value);
  private final Trigger shootSpeaker = new Trigger(() -> controller_2.getRightTriggerAxis() > 0.25);
  
  private final Trigger moveArm = new Trigger(()-> Math.abs(controller_2.getLeftY())>.1);
  // joystick button to goto specific slider spot
  private final JoystickButton customSliderPositionButton =
      new JoystickButton(controller_2, XboxController.Button.kBack.value);

  // Add joystick button to check april tag
  private final JoystickButton checkAprilTag = new JoystickButton(joystick, 8);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0), // falcon 500's use TalonFX instead of ModuleIOSparkMax
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        limelight = new LimelightIONetwork();
        inputs = new LimelightIOInputs();

        if (Constants.chassisOnly) {
          intake = new Intake(new IntakeIO() {});
          shoot = new Shoot(new ShootIO() {}, intake);
          slider = new Slider(new SliderIO() {});
          arm = new Arm(new ArmIO() {});

        } else {
          intake = new Intake(new IntakeIOSparkMax());
          shoot = new Shoot(new ShootIOSparkMax(), intake);
          slider = new Slider(new SliderIOSparkMax() {});
          arm = new Arm(new ArmIOSparkMax() {});
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        limelight = new LimelightIONetwork();
        inputs = new LimelightIOInputs();

        intake = new Intake(new IntakeIOSim());
        shoot = new Shoot(new ShootIOSim() {}, intake);
        slider = new Slider(new SliderIOSim() {});
        arm = new Arm(new ArmIOSim() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        limelight = new LimelightIONetwork();
        inputs = new LimelightIOInputs();

        intake = new Intake(new IntakeIO() {});
        shoot = new Shoot(new ShootIO() {}, intake);
        slider = new Slider(new SliderIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }
    armMovementCommand = new ArmMovement();
    // Set up auto routines
    NamedCommands.registerCommand(
        "shootspeaker",
        Commands.startEnd(() -> shoot.shootSpeaker(), shoot::stop, shoot).withTimeout(3.0));
    NamedCommands.registerCommand(
        "Pick_Up_Note",
        Commands.startEnd(() -> intake.intakeIn(), intake::stop, intake).withTimeout(1.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick.getRawAxis(translationAxis),
            () -> -joystick.getRawAxis(strafeAxis),
            () -> -joystick.getRawAxis(rotationAxis)));
    /*controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));*/

    // Intake
    IntakeRollersOn.whileTrue(
        new StartEndCommand(() -> intake.intakeIn(), () -> intake.holdCurrent(), intake));
    intakeOut.onTrue(
        new InstantCommand(
            () -> {
              armMovementCommand.goToIntakeOut(slider, arm);
            }));
    // aimSpeaker.onTrue(
    //     new InstantCommand(
    //         () -> {
    //           armMovementCommand.goToAimSpeaker();
    //         }));
    intakeIn.onTrue(
        new InstantCommand(
            () -> {
              armMovementCommand.goToIntakeIn(slider, arm);
            }));
    // aimAmp.onTrue(
    //     new InstantCommand(
    //         () -> {
    //           armMovementCommand.goToAimAmp();
    //         }));
    shootAmp.whileTrue(new StartEndCommand(() -> shoot.shootAmp(), shoot::stop, shoot));
    shootSpeaker.whileTrue(new StartEndCommand(() -> shoot.shootSpeaker(), shoot::stop, shoot));
    // Add code here to print out if tag in view when april tag button pressed
    checkAprilTag.whileTrue(new InstantCommand(() -> limelight.tagCenterButton(inputs)));
    aimCustom.onTrue(new InstantCommand(() -> armMovementCommand.goToANGLESmartDashboard(arm)));
    customSliderPositionButton.onTrue(
        new InstantCommand(() -> armMovementCommand.goToSLIDERSmartDashboard(slider)));
    moveArm.whileTrue(new StartEndCommand(()-> arm.setVoltage(controller_2.getLeftY()*-1), arm::stop, arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
