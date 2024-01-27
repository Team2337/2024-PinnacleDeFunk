// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightColor;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private final Pigeon2 pigeon = new Pigeon2(0);

  private final boolean UseLimelight = true;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    pigeon.getYaw();
    SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
    if (UseLimelight) {    
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
      //SmartDashboard.putNumber("Robot Vision Pose", Vision.getVisionPoseY(LimelightColor.ORANGE))
    }
  }

  @Override
  public void disabledInit() {
  }
  
  @Override
  public void disabledPeriodic() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            m_robotContainer.allianceColor = "red";
        }
        if (ally.get() == Alliance.Blue) {
            m_robotContainer.allianceColor = "blue";
        }
    }
    
    SmartDashboard.putString("Alliance Color", m_robotContainer.allianceColor);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    pigeon.setYaw(0);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
