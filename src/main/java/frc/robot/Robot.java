// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.nerdyfiles.vision.LimelightHelpers.RawFiducial;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private final Pigeon2 pigeon = new Pigeon2(0);

  //private final boolean UseLimelight = true;
  private double visionCounter = 0;
  private final Matrix<N3, N1> visionStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  private final Matrix<N3, N1> visionStdDevsMultiTags = new Matrix<>(Nat.N3(), Nat.N1());
  private Pose2d llPose, llPoseBattery;
  private boolean didAutoRun, multiBlueTargets, multiBatteryTargets = false;
  private Timer gcTimer = new Timer();
  


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);

    RobotController.setBrownoutVoltage(6.3);

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

    
    visionStdDevsMultiTags.set(0,0,0.75); //0.5
    visionStdDevsMultiTags.set(1,0,0.75); //0.5
    visionStdDevsMultiTags.set(2,0,Math.toRadians(30));//20
    m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevs);

    gcTimer.start();

  }
  @Override
  public void robotPeriodic() {
   // LimelightHelpers.SetRobotOrientation("limelight-blue", m_robotContainer.drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

      
      

    CommandScheduler.getInstance().run(); 
    
    pigeon.getYaw();
    SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
    if (m_robotContainer.drivetrain.useLimelight) {  
      if (DriverStation.isAutonomous() ) {
        //mt1_bat();
        mt1_blue();
      } else {
        
        mt1_blue();
      }
      
    }

    if (DriverStation.isDisabled()) {
      mt1_blue();
      //TODO:Take out before matches
      //mt1_bat();
      
    }

    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
        SmartDashboard.putString("Alliance Color", m_robotContainer.allianceColor);
    }
    
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
    didAutoRun = true;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.instantiateSubsystemsTeleop();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (didAutoRun) {
      //m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.335, 5.55), Rotation2d.fromDegrees(0)));
      //if (m_robotContainer.allianceColor == "red") {
        m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(new Translation2d(m_robotContainer.drivetrain.getState().Pose.getX(), m_robotContainer.drivetrain.getState().Pose.getY()), m_robotContainer.drivetrain.getState().Pose.getRotation()));
      //}
    } 
    //if (m_robotContainer.driverJoystick.y().getAsBoolean()) {pigeon.reset();}
    //pigeon.setYaw(0);
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

  public void mt1_blue() {
    //var lastResult = LimelightHelpers.getLatestResults("limelight-blue").targetingResults;
    LimelightHelpers.PoseEstimate lastResult = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-blue");
      
          //System.out.println(lastResult.tagCount);
      if (lastResult.tagCount > 0) {
        
          //System.out.println("Alex was here");
        multiBlueTargets = true;
        if (lastResult.tagCount > 1) {
          multiBatteryTargets = false;
        }
      } else {
        multiBlueTargets = false;
      }
      //llPose = lastResult.getBotPose2d_wpiBlue();
      double latency = LimelightHelpers.getLatency_Pipeline("limelight-blue");


      if ((lastResult.tagCount > 0)) {
        if (lastResult.tagCount > 1) {
          m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevsMultiTags);
        } else {
          m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevs);
        }
        if (multiBlueTargets) {
          m_robotContainer.drivetrain.addVisionMeasurement(lastResult.pose, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + latency + 2) / 1000));
        }
      }
        
  }


  public void mt1_bat() {
      var lastResultBattery = LimelightHelpers.getLatestResults("limelight-battery").targetingResults;
      
      if (lastResultBattery.valid ) {
        multiBatteryTargets = true;
        if (lastResultBattery.targets_Fiducials.length > 1) {
          multiBlueTargets = false;
        }
      } else {
        multiBatteryTargets = false;
      }

      llPoseBattery = lastResultBattery.getBotPose2d_wpiBlue();
      double latencyBattery = LimelightHelpers.getLatency_Pipeline("limelight-battery");


      
        if ((lastResultBattery.valid)) {
          if (lastResultBattery.targets_Fiducials.length > 1) {
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevsMultiTags);
          } else {
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(visionStdDevs);
          }
          if (multiBatteryTargets) {
            m_robotContainer.drivetrain.addVisionMeasurement(llPoseBattery, Timer.getFPGATimestamp() - ((Constants.Vision.IMAGE_PROCESSING_LATENCY_MS + latencyBattery + 2) / 1000));
          }
        }
  }
  
  public void mt2_blue() {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight-blue", m_robotContainer.drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      SmartDashboard.putNumber("Rob Angle", m_robotContainer.drivetrain.getPose().getRotation().getDegrees());
      LimelightHelpers.PoseEstimate mt2_blue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-blue");
      //SmartDashboard.putNumber("MT2-Blue Tag Count", mt2_blue.tagCount);

      if(Math.abs(pigeon.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2_blue.tagCount == 0) {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
         m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
         m_robotContainer.drivetrain.addVisionMeasurement(
            mt2_blue.pose,
            mt2_blue.timestampSeconds);
      }
  }

  public void mt2_bat() {
    boolean doRejectUpdateBat = false;
      LimelightHelpers.SetRobotOrientation("limelight-blue", m_robotContainer.drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

      LimelightHelpers.PoseEstimate mt2_bat = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-battery");

      //SmartDashboard.putNumber("MT2-Bat Tag Count", mt2_bat.tagCount);

      if(Math.abs(pigeon.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdateBat = true;
      }
      if (mt2_bat.tagCount == 0) {
        doRejectUpdateBat = true;
      }
      if(!doRejectUpdateBat)
      {
         m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
         m_robotContainer.drivetrain.addVisionMeasurement(
            mt2_bat.pose,
            mt2_bat.timestampSeconds);
      }
  }

}
