// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  public final Intake intake = Intake.getInstance();
  public final Arm arm = Arm.getInstance();
  public final Cantdle lamp = Cantdle.getInstance();

  // configure yeet mode by commenting one or the other

  private final Yeeter thrower = Yeeter.getInstance();
  // private final Thrower thrower = Thrower.getInstance();

  // end configure yeet mode

  public RobotContainer() {
    configureBindings();
  }

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3.0 * Math.PI; // 1.5 rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
      .withDriveRequestType(DriveRequestType.Velocity);
  private final Telemetry logger = new Telemetry(MaxSpeed);

  SwerveRequest.FieldCentricFacingAngle uhh = new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.01)
      .withRotationalDeadband(MaxAngularRate * 0.06)
      .withDriveRequestType(DriveRequestType.Velocity);

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
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(applyDeadband(-joystick.getLeftY()) * MaxSpeed)
            .withVelocityY(applyDeadband(-joystick.getLeftX()) * MaxSpeed)
            .withRotationalRate(applyDeadband(-joystick.getRightX()) * MaxAngularRate)));

    uhh.HeadingController.setPID(12.0, 0, 0.2);
    this.joystick.leftTrigger().whileTrue(
        this.drivetrain.applyRequest(() -> this.uhh.withVelocityX(applyDeadband(-joystick.getLeftY()) * MaxSpeed * 0.5)
            .withVelocityY(applyDeadband(-joystick.getLeftX()) * MaxSpeed * 0.5)
            .withTargetDirection(Rotation2d.fromDegrees(90.0 - joystick.getRightX() * 10.0))));

    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    // this.joystick.a().whileTrue(thrower.YeeterRoutine.dynamic(Direction.kForward));
    // this.joystick.b().whileTrue(thrower.YeeterRoutine.dynamic(Direction.kReverse));
    // this.joystick.x().whileTrue(thrower.YeeterRoutine.quasistatic(Direction.kForward));
    // this.joystick.y().whileTrue(thrower.YeeterRoutine.quasistatic(Direction.kReverse));

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
        .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.hold()));

    this.joystick.leftBumper().onTrue(this.thrower.eject()).onFalse(this.thrower.off());

    // Throw note
    this.joystick.rightTrigger()
        .onTrue(this.thrower.launch())
        .onFalse(new SequentialCommandGroup(
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.arm.setStow().alongWith(this.thrower.off())));

    this.thrower.hasNote
        .onTrue(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.6))
            .alongWith(this.lamp.flashColor(Cantdle.ORANGE, 1.0)))
        .onFalse(new InstantCommand(() -> this.joystick.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    this.thrower.ready.onTrue(this.lamp.flashColor(Cantdle.GREEN, 0.5));

    Trigger FMSconnect = new Trigger(() -> DriverStation.isFMSAttached());
    FMSconnect.onTrue(this.lamp.flashColor(Cantdle.PURPLE, 2.5));
    
    // this.driverController.start().onTrue(this.drive.toggleFieldRelative());

    // this.joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // this.joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // this.joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // this.joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        this.arm.setOverhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off()),
        new WaitCommand(1.0),
        this.thrower.launch(),
        new WaitCommand(1.0),
        this.arm.setIntake().alongWith(this.thrower.off()));
  }

  Field2d f = new Field2d();

  public void periodic() {
    // LimelightHelpers.SetRobotOrientation("limelight",
    // this.drivetrain.getState().Pose.getRotation().getDegrees(),
    // 0, 0, 0, 0, 0);

    // boolean reject = false;
    // LimelightHelpers.PoseEstimate llPose =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    // reject |= (Math.abs(this.drivetrain.getPigeon2().getRate()) >= 360.0);
    // reject |= llPose.avgTagDist >= 4.0;

    // // if (!reject) {
    // // this.drivetrain.addVisionMeasurement(llPose.pose, llPose.timestampSeconds,
    // VecBuilder.fill(0.6, 0.6, 9999999));
    // // }

    // f.setRobotPose(drivetrain.getState().Pose);
  }

  private final Pose2d startBlue = new Pose2d(1.4, 5.56, Rotation2d.fromDegrees(0.0));
  private final Pose2d startRed = new Pose2d(15.1, 5.56, Rotation2d.fromDegrees(180.0));

  public void setStartingPose() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0.0));
        drivetrain.seedFieldRelative(startBlue);
      } else {
        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180.0));
        drivetrain.seedFieldRelative(startRed);
      }
    }
  }
}
