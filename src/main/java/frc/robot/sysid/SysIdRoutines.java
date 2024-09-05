package frc.robot.sysid;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;

public class SysIdRoutines {
    private final SwerveRequest.SysIdSwerveTranslation SysIDTranslate = new SwerveRequest.SysIdSwerveTranslation();

    @SuppressWarnings("unused")
    private SysIdRoutine translateRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> TunerConstants.DriveTrain.setControl(SysIDTranslate.withVolts(volts)),
                    null,
                    TunerConstants.DriveTrain));

    private final SwerveRequest.SysIdSwerveRotation SysIDRotate = new SwerveRequest.SysIdSwerveRotation();

    @SuppressWarnings("unused")
    private SysIdRoutine rotateRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> TunerConstants.DriveTrain.setControl(SysIDRotate.withVolts(volts)),
                    null,
                    TunerConstants.DriveTrain));

    private final SwerveRequest.SysIdSwerveSteerGains SysIDSteer = new SwerveRequest.SysIdSwerveSteerGains();

    @SuppressWarnings("unused")
    private SysIdRoutine steerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> TunerConstants.DriveTrain.setControl(SysIDSteer.withVolts(volts)),
                    null,
                    TunerConstants.DriveTrain));
}
