package frc.robot.commands.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class SwerveDriveCommand extends Command{
    
    private Drivetrain drivetrain;
    private CommandXboxController driverJoystick;
    private SwerveRequest swerveRequest;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Swerve.angularDeadband);
    
    private SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Swerve.angularDeadband);

        private final SwerveRequest.SwerveDriveBrake lockdown = new SwerveRequest.SwerveDriveBrake();
        
        private PhoenixPIDController turnPID = new PhoenixPIDController(5, 0, 0);
        private double forward, rotation, strafe, speakerY, speakerX, randomX, randomY = 0;
        private Supplier<Double> yVelocity;
        private Supplier<String> allianceColor;

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController driverJoystick, Supplier<Double> yVelocity, Supplier<String> allianceColor) {
        this.drivetrain = drivetrain;
        this.driverJoystick = driverJoystick;
        this.yVelocity = yVelocity;
        this.allianceColor = allianceColor;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveFacingAngle.HeadingController = turnPID;
    }

    @Override
    public void execute() {

        //Read Inputs From Joysticks
        forward = Utilities.deadband(-driverJoystick.getLeftY(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        strafe = Utilities.deadband(-driverJoystick.getLeftX(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        rotation = Utilities.deadband(-driverJoystick.getRightX(), Constants.Swerve.driveDeadband) * Constants.Swerve.MaxAngularRate;
        // if (allianceColor.get() == "red") {
        //     forward = -forward;
        //     strafe = -strafe;
        // }
        //If we have set an angle to drive at and driver has hit drive at angle button, drive at that angle
        if ((drivetrain.rotationAngle != 0) && (drivetrain.driveAtAngle)) {
            swerveRequest = driveFacingAngle
                .withTargetDirection(Rotation2d.fromDegrees(drivetrain.rotationAngle))
                .withVelocityX(forward)
                .withVelocityY(strafe);
            // If endgame switch is true, drive robot centric
        } else if (drivetrain.pointAtSpeaker) {
            
            if (allianceColor.get() == "blue") {
                speakerX = Constants.FieldElements.blueSpeakerCenter.getX();
                speakerY = Constants.FieldElements.blueSpeakerCenter.getY();
            } else {
                speakerX = Constants.FieldElements.redSpeakerCenter.getX();
                speakerY = Constants.FieldElements.redSpeakerCenter.getY();
            }
            Pose2d currentPose = drivetrain.getPose();
            double currentPoseX = currentPose.getX();
            double currentPoseY = currentPose.getY();

            double angleToSpeakerRad = Math.atan2(currentPoseY - speakerY, currentPoseX - speakerX);
            double angleToSpeaker = Math.toDegrees(angleToSpeakerRad);
            double modAngleToSpeaker = angleToSpeaker + (yVelocity.get() * 8);
            if (allianceColor.get() == "red") {
                modAngleToSpeaker = modAngleToSpeaker - 180;
            }
            // SmartDashboard.putNumber("Angle to Speaker", angleToSpeaker);
            // SmartDashboard.putNumber("Mod Angle to Speaker", modAngleToSpeaker);
            // SmartDashboard.putNumber("Y Velocity", yVelocity.get());

         swerveRequest = driveFacingAngle
                .withTargetDirection(Rotation2d.fromDegrees(modAngleToSpeaker))
                .withVelocityX(forward)
                .withVelocityY(strafe);

        } else if(drivetrain.visionRotate) {
            
            double visionKP = 0.7;
            rotation = LimelightHelpers.getTX("limelight-blue");
            SmartDashboard.putNumber("Vision Rotation", rotation);
            if(Math.abs(rotation) > 0) {
                rotation = Utilities.scaleVisionToOne(rotation);
                rotation = rotation * Constants.Swerve.MaxAngularRate;
                rotation = -rotation * visionKP; //Needs to be greater than 0.48 to turn
            } else {
                rotation = 0;
            }
            swerveRequest = drive
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotation);
        } else if (drivetrain.pointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp) {
            
            if (allianceColor.get() == "blue") {
                randomX = Constants.FieldElements.randomPointBlue.getX();
                randomY = Constants.FieldElements.randomPointBlue.getY();
            } else {
                randomX = Constants.FieldElements.randomPointRed.getX();
                randomY = Constants.FieldElements.randomPointRed.getY();
            }
            Pose2d currentPose = drivetrain.getPose();
            double currentPoseX = currentPose.getX();
            double currentPoseY = currentPose.getY();

            double angleToRandomRad = Math.atan2(currentPoseY - randomY, currentPoseX - randomX);
            double angleToRandom = Math.toDegrees(angleToRandomRad);
            if (allianceColor.get() == "red") {
                angleToRandom = angleToRandom - 180;
            }
            // SmartDashboard.putNumber("Angle to Speaker", angleToSpeaker);
            // SmartDashboard.putNumber("Mod Angle to Speaker", modAngleToSpeaker);
            // SmartDashboard.putNumber("Y Velocity", yVelocity.get());

         swerveRequest = driveFacingAngle
                .withTargetDirection(Rotation2d.fromDegrees(angleToRandom))
                .withVelocityX(forward)
                .withVelocityY(strafe);
        
        } else if (true){//(drivetrain.endGame) {
            swerveRequest = robotCentric
                .withVelocityX(-forward)
                .withVelocityY(-strafe)
                .withRotationalRate(rotation);

        } else if (drivetrain.noteDetection) {
            var lastResult = LimelightHelpers.getLatestResults("limelight-coral").targetingResults;
            if ((lastResult.valid)) {
                double tx = LimelightHelpers.getTX("limelight-coral");
                rotation = (Utilities.scaleVisionToOne(-tx) / 0.5);
            }

            if (rotation <= 0.1 && rotation >= -0.1) {
                rotation = 0;
            } 
            SmartDashboard.putNumber("Note Detection", rotation);
            swerveRequest = drive
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotation);
            // Enable Lockdown Mode
        } else if (drivetrain.lockdownEnabled) {
            swerveRequest = lockdown;
            // Otherwise, drive field centric
        } else {
            // SmartDashboard.putNumber("Joystick Rotation", rotation);
            swerveRequest = drive
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotation);
        }
        drivetrain.setControl(swerveRequest);

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
