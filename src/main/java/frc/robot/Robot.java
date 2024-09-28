// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LimeLight;

import java.util.concurrent.atomic.AtomicReference;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  SendableChooser<String> startSpot = new SendableChooser<>();
  // PowerDistribution pd = new PowerDistribution(0, ModuleType.kRev);

  @Override
  public void robotInit() {
    LimeLight.front.start();
    LimeLight.back.start();

    // pd.setSwitchableChannel(true);
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    SignalLogger.setPath("/media/sda");

    // for (int port = 5800; port <= 5809; port++) {
    //   PortForwarder.add(port, "limelight.local", port);
    // }

    startSpot.addOption("amp", "amp");
    startSpot.setDefaultOption("center", "center");
    startSpot.addOption("source", "source");

    SmartDashboard.putData(startSpot);

    SmartDashboard.putNumber("underhand", Constants.Arm.Positions.UNDERHAND);

    AtomicReference<String> autoName = new AtomicReference<>();

    new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
            .onTrue(Commands.runOnce(() -> this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand(autoName.get())).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand(autoName.get())).ignoringDisable(true));

    SendableChooser<String> autoChooser = new SendableChooser<>();
    autoChooser.addOption("Skibidi Rizz", "TestPath");
    autoChooser.addOption("Source side", "SourceSide");
    autoChooser.addOption("Amp side", "AmpSide");
    autoChooser.setDefaultOption("None", null);
    autoChooser.onChange((name) -> {
      autoName.set(name);
      this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand(autoName.get());
    });
    SmartDashboard.putData(autoChooser);

    System.out.println("skibidi yeah!");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.isDSAttached()) {
      SmartDashboard.putNumber("got a ds trash number", Timer.getFPGATimestamp());
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SignalLogger.start();

    // default to Blue Alliance
    Alliance ally = DriverStation.getAlliance().orElse(Alliance.Blue);
    m_robotContainer.drivetrain.setAlliance(ally);

    if (ally.equals(Alliance.Blue)) {
      LimeLight.front.setPriorityTag(7);
    } else {
      LimeLight.front.setPriorityTag(4);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
