package frc.robot.sysid;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;

public class SysIdRoutines {
        public static final SwerveRequest.SysIdSwerveTranslation SysIDTranslate = new SwerveRequest.SysIdSwerveTranslation();

        public static final SysIdRoutine translateRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        null,
                                        Units.Volts.of(4),
                                        null,
                                        (state) -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                        (volts) -> TunerConstants.DriveTrain
                                                        .setControl(SysIDTranslate.withVolts(volts)),
                                        null,
                                        TunerConstants.DriveTrain));

        private static final SwerveRequest.SysIdSwerveRotation SysIDRotate = new SwerveRequest.SysIdSwerveRotation();

        public static final SysIdRoutine rotateRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        null,
                                        Units.Volts.of(4),
                                        null,
                                        (state) -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                        (volts) -> TunerConstants.DriveTrain.setControl(SysIDRotate.withVolts(volts)),
                                        null,
                                        TunerConstants.DriveTrain));

        private static final SwerveRequest.SysIdSwerveSteerGains SysIDSteer = new SwerveRequest.SysIdSwerveSteerGains();

        public static final SysIdRoutine steerRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        null,
                                        Units.Volts.of(4),
                                        null,
                                        (state) -> SignalLogger.writeString("state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                        (volts) -> TunerConstants.DriveTrain.setControl(SysIDSteer.withVolts(volts)),
                                        null,
                                        TunerConstants.DriveTrain));

        // pretend to output with volts even though is amps
        public static final SysIdRoutine armTorqueCurrentRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        Units.Volts.of(3.0).per(Units.Seconds.of(1)),
                                        Units.Volts.of(8.0),
                                        null,
                                        (state) -> SignalLogger.writeString("arm-tc-state", state.toString())),
                        new SysIdRoutine.Mechanism(
                                        (amps) -> Arm.getInstance().setAmpsSysID(amps.magnitude()),
                                        null,
                                        Arm.getInstance()));

}
