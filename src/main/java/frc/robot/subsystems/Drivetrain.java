package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field = new Field2d();
    public double rotationAngle = 0;
    public boolean driveAtAngle, endGame, lockdownEnabled, pointAtSpeaker = false;
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d modifyPose() {
        Pose2d pose = this.getState().Pose;
        Rotation2d newRotation = Rotation2d.fromDegrees(-pose.getRotation().getDegrees());
        double x = pose.getX();
        double y = pose.getY();
        return new Pose2d(x, y, newRotation);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            //this::modifyPose,
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0), //Drive
                                            new PIDConstants(2.5, 0, 0), //Turn
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->false, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

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

    /**
     * Sets the rotation angle for use with Field Centric Facing Angle
     * @param angle
     */
     public void setRotationAngle(double angle) {
        rotationAngle = angle;
    }

    /**
     * Sets drive at angle to true or false based on a button press to turn on or off field centric facing angle
     */
    public void setToDriveAtAngle() {
        if (driveAtAngle) {
            driveAtAngle = false;
            rotationAngle = 0;
        } else {
            driveAtAngle = true;
        }
    }

    /**
     * Sets boolean to indicate we are in endgame from the operator station
     * @param end
     */
    public void setEndGame(boolean end) {
        endGame = end;
    }

    /**
     * Sets a boolean to indicate whether lockdown is enabled for use with swerve drive command
     */
    public void enableLockdown() {
        if (lockdownEnabled) {
            lockdownEnabled = false;
        } else {
            lockdownEnabled = true;
        }
    }

    public void setPointAtSpeaker(boolean pointSpeaker) {
        pointAtSpeaker = pointSpeaker;
    }
    
    @Override
    public void periodic() {
        //field.setRobotPose(this.getState().Pose); 
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //field.getObject("target pose").setPose(pose);
        field.setRobotPose(pose);
      });
      
      log();
    }

    public void log() {
        if (Constants.DashboardLogging.SWERVE) {
            SmartDashboard.putBoolean("Drive At Angle", driveAtAngle);
            SmartDashboard.putNumber("Rotation Angle", rotationAngle);
            SmartDashboard.putData("Field", field);
        }
        SmartDashboard.putBoolean("Lockdown Enabled", lockdownEnabled);
        SmartDashboard.putBoolean("End Game", endGame);
    }

   
}
