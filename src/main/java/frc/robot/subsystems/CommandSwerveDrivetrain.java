package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.LimeLight.VisionResult;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private ADIS16470_IMU green_gyro;

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MIN_TRANSLATE_MPS)
            .withRotationalDeadband(Constants.Drive.MIN_ROTATE_RPS)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle ampDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Drive.MIN_TRANSLATE_MPS)
            .withRotationalDeadband(Constants.Drive.MIN_ROTATE_RPS)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withTargetDirection(Rotation2d.fromDegrees(-90.0));

    private final SwerveRequest.FieldCentricFacingAngle underhandSpeakerDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Drive.MIN_TRANSLATE_MPS)
            .withRotationalDeadband(Constants.Drive.MIN_ROTATE_RPS)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final PhoenixPIDController headingController = new PhoenixPIDController(30.0, 0.0, 1.0);

    public enum DriveMode {
        FIELD_CENTRIC_DRIVE,
        AMP_DRIVE,
        UNDERHAND_SPEAKER_DRIVE;
    }

    private DriveMode activeMode = DriveMode.FIELD_CENTRIC_DRIVE;

    private Alliance alliance = Alliance.Blue;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // super(driveTrainConstants, 250.0, VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0), modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        green_gyro = new ADIS16470_IMU();

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.ampDrive.HeadingController = this.headingController;
        this.underhandSpeakerDrive.HeadingController = this.headingController;

        this.fieldCentricDrive.ForwardReference = ForwardReference.RedAlliance;
        this.ampDrive.ForwardReference = ForwardReference.RedAlliance;
        this.underhandSpeakerDrive.ForwardReference = ForwardReference.RedAlliance;
    }

    public void setAlliance(Alliance a) {
        this.alliance = a;
    }

    private static final double kBufferDuration = 1.5;
    private final TimeInterpolatableBuffer<Rotation2d> m_HeadingBuffer = TimeInterpolatableBuffer
            .createBuffer(kBufferDuration);

    public Optional<Rotation2d> sampleHeading(double time) {
        return m_HeadingBuffer.getSample(time);
    }

    public Rotation2d targetHeading() {
        VisionResult res = LimeLight.front.getLatest();
        if (res == null || !res.valid) {
            if (this.alliance == Alliance.Blue) {
                return Rotation2d.fromDegrees(180.0);
            } else {
                return Rotation2d.fromDegrees(0.0);
            }
            
        }
        Optional<Rotation2d> oldHeading = sampleHeading(res.timestamp);

        if (oldHeading.isPresent()) {
            SmartDashboard.putNumber("old heading", oldHeading.get().getDegrees());
            return oldHeading.get().plus(Rotation2d.fromDegrees(-res.tx));
        } else {
            return this.getState().Pose.getRotation();
        }
    }

    public Command teleopDrive(DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX) {
        return this.run(() -> {

            double leftX = Util.applyDeadband(-leftStickX.getAsDouble());
            double leftY = Util.applyDeadband(-leftStickY.getAsDouble());
            double rightX = Util.applyDeadband(-rightStickX.getAsDouble());

            // flip direction of sticks if Red, this allows coordinate system to always be
            // same
            // rotation is always CCW positive
            if (this.alliance == Alliance.Red) {
                leftX *= -1;
                leftY *= -1;
            }

            SmartDashboard.putNumber("left stick", leftX);
            SmartDashboard.putString("drive mode", activeMode.name());
            switch (activeMode) {
                case UNDERHAND_SPEAKER_DRIVE:
                    this.setControl(underhandSpeakerDrive
                            .withVelocityX(
                                    leftY * Constants.Drive.MAX_SPEED_MPS * Constants.Drive.SPEAK_SLOWDOWN_SCALAR)
                            .withVelocityY(
                                    leftX * Constants.Drive.MAX_SPEED_MPS * Constants.Drive.SPEAK_SLOWDOWN_SCALAR)
                            .withTargetDirection(
                                    this.targetHeading()));
                    break;
                case AMP_DRIVE:
                    this.setControl(ampDrive
                            .withVelocityX(
                                    leftY * Constants.Drive.MAX_SPEED_MPS * Constants.Drive.AMP_SLOWDOWN_SCALAR)
                            .withVelocityY(
                                    leftX * Constants.Drive.MAX_SPEED_MPS * Constants.Drive.AMP_SLOWDOWN_SCALAR)
                            .withTargetDirection(
                                    Rotation2d.fromDegrees(-90 + rightX * Constants.Drive.AMP_ANGLE_ADJUST_DEG)));
                    break;
                case FIELD_CENTRIC_DRIVE:
                    // intentional fall through
                default:
                    this.setControl(fieldCentricDrive
                            .withVelocityX(leftY * Constants.Drive.MAX_SPEED_MPS)
                            .withVelocityY(leftX * Constants.Drive.MAX_SPEED_MPS)
                            .withRotationalRate(rightX * Constants.Drive.MAX_ROTATION_RATE_RPS));
                    break;
            }
        });
    }

    public Command setDriveMode(DriveMode newMode) {
        return this.runOnce(() -> this.activeMode = newMode).asProxy();
    }

    /**
     * Command to reset Pigeon and Analog Devices gyro internal yaw angle to 0.
     * 
     */
    public Command resetGyros() {
        return this.resetGyros(0.0);
    }

    /**
     * Command to reset Pigeon and Analog Devices gyro internal yaw angle to
     * specified angle.
     * 
     */
    public Command resetGyros(double angle) {
        return this.runOnce(() -> {
            green_gyro.setGyroAngleZ(angle);
            m_pigeon2.setYaw(angle);
        });
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {

        // if (DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue)
        //         .equals(DriverStation.Alliance.Blue)) {
        //     setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
        // } else {
        //     setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
        // }
        SmartDashboard.putNumber("pigeon yaw", this.m_pigeon2.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("green yaw", this.green_gyro.getAngle());
        SmartDashboard.putNumber("diff", this.m_pigeon2.getYaw().getValueAsDouble() -
                this.green_gyro.getAngle());
        SmartDashboard.putNumber("steer angle reference",
                this.Modules[0].getSteerMotor().getClosedLoopReference().getValueAsDouble() % (Math.PI * 2.0));
        SmartDashboard.putNumber("steer angle",
                this.Modules[0].getSteerMotor().getPosition().getValueAsDouble() % (Math.PI * 2.0));
        SmartDashboard.putNumber("steer angle error",
                this.Modules[0].getSteerMotor().getClosedLoopError().getValueAsDouble());

        SmartDashboard.putNumber("wheel velocity reference",
                this.Modules[0].getDriveMotor().getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("wheel velocity", this.Modules[0].getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("wheel velocity error",
                this.Modules[0].getDriveMotor().getClosedLoopError().getValueAsDouble());

        SmartDashboard.putNumber("wheel current",
                this.Modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("drive angle", this.ampDrive.TargetDirection.getDegrees());
        SmartDashboard.putNumber("drive angle 2", Math.toDegrees(this.ampDrive.HeadingController.getSetpoint()));

        m_HeadingBuffer.addSample(Timer.getFPGATimestamp(), this.getState().Pose.getRotation());
    }

    /**
     * 
     * simulation
     * 
     */

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
