// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = Intake.getInstance();
  private final Arm arm = Arm.getInstance();
  private final Thrower thrower = Thrower.getInstance();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final static double deadband = 0.12;
  private final double applyDeadband(double in) {
    if (Math.abs(in) < deadband) {
      return 0.0;
    } else if (in > 0) {
      return (in - deadband) / (1.0 - deadband);
    } else {
      return (in + deadband) / (1.0 - deadband);
    }
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(applyDeadband(-joystick.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(applyDeadband(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(applyDeadband(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    this.joystick.a().onTrue(
        this.arm.setUnderhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off())

    );

    this.joystick.y().onTrue(
        this.arm.setOverhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off())

    );

    this.joystick.b().onTrue(
        this.arm.setAmp()
            .alongWith(this.thrower.prepareAmp())
            .alongWith(this.intake.off()));

    this.joystick.x().onTrue(
        this.arm.setStow()
            .alongWith(this.intake.off())
            .alongWith(this.thrower.off())

    );

    // Intake Note
    this.joystick.rightBumper()
        .onTrue(
            this.intake.intakeNote()
                .alongWith(this.arm.setIntake())
                .alongWith(this.thrower.setIntake()))
        .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.off()));

    // Throw note
    this.joystick.rightTrigger()
        .onTrue(this.thrower.launch())
        .onFalse(new SequentialCommandGroup(
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.arm.setStow().alongWith(this.thrower.off())));

    this.thrower.hasNote.onTrue(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.6)))
        .onFalse(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // this.driverController.start().onTrue(this.drive.toggleFieldRelative());

    // this.joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // this.joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // this.joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // this.joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
