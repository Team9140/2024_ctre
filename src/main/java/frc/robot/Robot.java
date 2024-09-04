// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
 
  SendableChooser<String> startSpot = new SendableChooser<>();
  // PowerDistribution pd = new PowerDistribution(0, ModuleType.kRev);

  @Override
  public void robotInit() {
    // pd.setSwitchableChannel(true);
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    SignalLogger.setPath("/media/sda");

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    startSpot.addOption("amp", "amp");
    startSpot.setDefaultOption("center", "center");
    startSpot.addOption("source", "source");

    SmartDashboard.putData(startSpot);

    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[] { 4, 3, 7, 8 });

    SmartDashboard.putNumber("underhand", Constants.Arm.Positions.UNDERHAND);
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
    if (DriverStation.isFMSAttached()) {
      m_robotContainer.setStartingPose(startSpot.getSelected());
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    if (DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Blue)) {
          m_robotContainer.drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
    } else {
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
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
