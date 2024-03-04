package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    //private Field2d field = new Field2d();
    public double rotationAngle = 0;
    public boolean driveAtAngle, endGame, lockdownEnabled, pointAtSpeaker, pointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp, noteDetection = false;
    
    public boolean useLimelight = true;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupShuffleboard();
    }
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupShuffleboard();
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
            //()->false, // Change this if the path needs to be flipped on red vs blue
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
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
    // public void setToDriveAtAngle() {
    //     if (driveAtAngle) {
    //         driveAtAngle = false;
    //         rotationAngle = 0.5;
    //     } else {
    //         driveAtAngle = true;
    //     }
    // }

    public void setDriveAtAngleTrue() {
        driveAtAngle = true;
    }

    public void setDriveAtAngleFalse() {
        driveAtAngle = false;
    }

    public void setAngleToZero() {
        rotationAngle = 0.5;
        driveAtAngle = true;
    }

    /**
     * Sets boolean to indicate we are in endgame from the operator station
     * @param end
     */
    public void setEndGame(boolean end) {
        endGame = end;
    }

    public void setNoteDetection(boolean note) {
        noteDetection = note;
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

    public void setPointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp(boolean pointRandom) {
        pointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp = pointRandom;
    }

    public void setCartesianVectorFalse() {
        pointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp = false;
    }
    
    @Override
    public void periodic() {
        //field.setRobotPose(this.getState().Pose); 
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //field.getObject("target pose").setPose(pose);
        //TODO: If auto doesn't work, uncomment
        //field.setRobotPose(pose);
      });
      //if (DriverStation.isAutonomous()) {
        if (this.getState().Pose.getX() >= Constants.Swerve.DISABLE_LIMELIGHT_DISTANCE) {
            useLimelight = false;
        } else {
            useLimelight = true;
        }
      //} 
      log();
    }

    public void log() {
        if (Constants.DashboardLogging.SWERVE) {
            SmartDashboard.putBoolean("Drive At Angle", driveAtAngle);
            SmartDashboard.putNumber("Rotation Angle", rotationAngle);
        }
        SmartDashboard.putBoolean("Lockdown Enabled", lockdownEnabled);
        SmartDashboard.putBoolean("End Game", endGame);
    }

    public void setupShuffleboard() {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
      
      systemsCheck.addDouble("Pose Rotation", () -> this.getState().Pose.getRotation().getDegrees())
        .withPosition(SystemsCheckPositions.POSE_ROTATION.x, SystemsCheckPositions.POSE_ROTATION.y)
        .withSize(2, 1);
    }

    public Command followPathCommand(PathPlannerPath path) {
        //PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
                path,
                ()-> this.getState().Pose, // Robot pose supplier
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0,//0.44958, // Drive base radius in meters. Distance from robot center to furthest module. Was 0.4
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

   
}
