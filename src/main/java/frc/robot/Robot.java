// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightColor;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private final Pigeon2 pigeon = new Pigeon2(0);

  //TODO: Validate This
  private final boolean UseLimelight = false;
  private double visionCounter = 0;
  private final Matrix<N3, N1> visionStdDevs = new Matrix<>(Nat.N3(), Nat.N1());


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);

    /**
   * Sets the value at the given indices.
   *
   * @param row The row of the element.
   * @param col The column of the element.
   * @param value The value to insert at the given location.
   */
    visionStdDevs.set(0,0,2); 
    visionStdDevs.set(1,0,2);
    visionStdDevs.set(2,0,Math.toRadians(90));
    m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevs);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    pigeon.getYaw();
    SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
    if (UseLimelight) {    
      var lastResult = LimelightHelpers.getLatestResults("limelight-orange").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
      

      if ((lastResult.valid) && (visionCounter > 0)) {
        m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + m_robotContainer.getVisionLatency(LimelightColor.ORANGE) + 2) / 1000));
        visionCounter = 0;
      } else {
        visionCounter++;
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
    
    //SmartDashboard.putString("Alliance Color", m_robotContainer.allianceColor);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
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
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    pigeon.setYaw(0);
    m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.335, 5.55), Rotation2d.fromDegrees(0)));
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
