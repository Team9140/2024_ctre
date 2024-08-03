package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Yeeter extends SubsystemBase {
    private static Yeeter instance;

    public static Yeeter getInstance() {
        return instance == null ? instance = new Yeeter() : instance;
    }

    private final WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.Ports.THROWER_FEEDER);

    private double feederVolts;

    private final TalonFX leftRollers;
    private final TalonFX rightRollers;

    private final VelocityVoltage leftSpeed;
    private final VelocityVoltage rightSpeed;

    private final SlewRateLimiter leftSlew;
    private final SlewRateLimiter rightSlew;

    private double leftSpeedTarget;
    private double rightSpeedTarget;

    private final VoltageOut SysIDController = new VoltageOut(0.0).withEnableFOC(true);

    public SysIdRoutine YeeterRoutine = new SysIdRoutine(new SysIdRoutine.Config(
            null, Units.Volts.of(4.0), null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> this.SysIDController.withOutput(volts.magnitude()), null, this));

        

    private Yeeter() {
        leftRollers = new TalonFX(6, "jama");
        rightRollers = new TalonFX(5, "jama");

        CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Thrower.Launcher.MAX_CURRENT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyTimeThreshold(1.0)
                .withSupplyCurrentLimit(100.0)
                .withSupplyCurrentLimitEnable(true);

        Slot0Configs configs = new Slot0Configs()
                .withKP(0.26)
                .withKV(0.15)
                .withKS(0.36806);

        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(launcherCurrentLimits)
                .withSlot0(configs);

        this.leftRollers.getConfigurator().apply(launcherConfiguration
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        this.rightRollers.getConfigurator().apply(launcherConfiguration
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

        this.leftSpeed = new VelocityVoltage(0.0).withEnableFOC(true);
        this.rightSpeed = new VelocityVoltage(0.0).withEnableFOC(true);

        double slewRate = 12000.0 / 60.0 / 1.0; // zero to full speed in one second
        this.leftSlew = new SlewRateLimiter(slewRate);
        this.rightSlew = new SlewRateLimiter(slewRate);

        this.feeder.setInverted(true);
        this.feeder.configContinuousCurrentLimit(Constants.Thrower.Feeder.MAX_CURRENT);

        
    }

    @Override
    public void periodic() {
        // this.leftRollers.setControl(SysIDController);
        // this.rightRollers.setControl(SysIDController);
        this.leftRollers.setControl(this.leftSpeed.withVelocity(this.leftSlew.calculate(this.leftSpeedTarget)));
        this.rightRollers.setControl(this.rightSpeed.withVelocity(this.rightSlew.calculate(this.rightSpeedTarget)));
        this.feeder.setVoltage(this.feederVolts);
    }

    public double getFeederCurrent() {
        return this.feeder.getSupplyCurrent();
    }

    public Command setIntake() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.INTAKE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.INTAKE_RPS;
            this.rightSpeedTarget = Constants.Yeeter.INTAKE_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.INTAKE_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.INTAKE_RPS);
        });
    }

    public Command hold() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = 0.0;
            this.rightSpeedTarget = 0.0;
            // this.leftSpeed.withVelocity(0.0);
            // this.rightSpeed.withVelocity(0.0);
        });
    }

    public Command prepareSpeaker() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.LEFT_SPEAKER_RPS;
            this.rightSpeedTarget = Constants.Yeeter.RIGHT_SPEAKER_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.LEFT_SPEAKER_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.RIGHT_SPEAKER_RPS);
        });
    }

    public Command prepareAmp() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.AMP_RPS;
            this.rightSpeedTarget = Constants.Yeeter.AMP_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.AMP_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.AMP_RPS);
        });
    }

    public Command launch() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.LAUNCH_VOLTAGE;
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            this.feederVolts = 0.0;
            this.leftSpeedTarget = 0.0;
            this.rightSpeedTarget = 0.0;
        });
    }

    private final Debouncer noty = new Debouncer(0.25);

    public final Trigger hasNote = new Trigger(
            () -> noty.calculate(Math.abs(this.feeder.getStatorCurrent()) > Constants.INTAKE_NOTIFY_CURRENT)
                    && this.feederVolts == Constants.Thrower.Feeder.INTAKE_VOLTAGE);
}
