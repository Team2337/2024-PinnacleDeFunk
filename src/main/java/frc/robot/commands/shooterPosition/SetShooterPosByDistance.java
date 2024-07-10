package frc.robot.commands.shooterPosition;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.ShooterPosPot;


public class SetShooterPosByDistance extends Command {
    private ShooterPosPot shooterPosPot;
    private Supplier<Pose2d> currentPose;
    private Translation2d speakerPose;
    private double speakerX, speakerY, currentX, currentY, distanceInMeters, newSetpoint, modNewSetpoint;
    private double minStringPotValue = 7;//5.3;
    private double maxStringPotValue = 15;//10.1;
    private Supplier<String> allianceColor;
    private Supplier<Double> xVelocity, yVelocity;
    private Supplier<Boolean> topSensor;
    private double distanceToFloor = 1.4478; //TODO:  Validate to center of tags 4 + 7
    private double cameraHeight = 0.2667;



    public SetShooterPosByDistance(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose, Supplier<String> allianceColor, Supplier<Double> xVelocity, Supplier<Double> yVelocity, Supplier<Boolean> topSensor) {
        this.shooterPosPot = shooterPosPot;
        this.currentPose = currentPose;
        this.allianceColor = allianceColor;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.topSensor = topSensor;
        addRequirements(shooterPosPot);
    }

    @Override
    public void initialize() {
        if (allianceColor.get() == "blue") {
            LimelightHelpers.setPriorityTagID("limelight-blue", 7);
       } else {
            LimelightHelpers.setPriorityTagID("limelight-blue", 4);
       }
    }
    
    @Override
    public void execute() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-blue");
        double visionDistance = limelightMeasurement.avgTagDist;

        double visionDistanceInMeters = Math.sqrt(Math.pow(visionDistance, 2) + Math.pow((distanceToFloor-cameraHeight), 2));
        visionDistanceInMeters = Math.sqrt(Math.pow(visionDistanceInMeters, 2) - Math.pow(distanceToFloor, 2));
        SmartDashboard.putNumber("Shooter/Vision Distance in meters", visionDistanceInMeters);

        if (allianceColor.get() == "blue") {
            speakerPose = Constants.FieldElements.blueSpeakerCenter;
        } else {
            speakerPose = Constants.FieldElements.redSpeakerCenter;
        }
        speakerX = speakerPose.getX();      
        speakerY = speakerPose.getY(); 
        currentX = (currentPose.get().getX() - (xVelocity.get()/5));
        currentY = (currentPose.get().getY() - (yVelocity.get()/5));
        distanceInMeters = Math.sqrt(Math.pow((currentX - speakerX), 2) + Math.pow((currentY - speakerY), 2));
        //newSetpoint = (-0.34540235 * Math.pow(distanceInMeters, 2)) + (3.7274448 * distanceInMeters) + 4.1656188; //FUDGE
        newSetpoint = (0.114 * Math.pow(distanceInMeters, 3)) + (-1.64 * Math.pow(distanceInMeters, 2)) + (8.34 * distanceInMeters) + -1.127;//-0.927 shot low //1.127 Blue -0.2
        //newSetpoint = (-0.42026111 * Math.pow(distanceInMeters, 2)) + (4.2693814 * distanceInMeters) + 3.2881356; //RAW
        //newSetpoint = (-0.42233788 * Math.pow(distanceInMeters, 2)) + (4.2114015 * distanceInMeters) + 3.5361784; //REMOVE POINTS

        // if (xVelocity.get() > 0) {
        //     modNewSetpoint = newSetpoint + (xVelocity.get() / 2.1); //  Was 2
        // } else if (xVelocity.get() < 0) {
        //     modNewSetpoint = newSetpoint + (xVelocity.get() / 1.5); //  Was 2
        // }

        if (allianceColor.get() == "blue") {
            newSetpoint -= 0.2;
        } else {
            newSetpoint -= 0.2;
            //speakerPose = Constants.FieldElements.redSpeakerCenter;
        }

        modNewSetpoint = newSetpoint + (xVelocity.get() / 2);

        if (modNewSetpoint < minStringPotValue) {
            modNewSetpoint = minStringPotValue;
        } 
        if (modNewSetpoint > maxStringPotValue) {
            modNewSetpoint = maxStringPotValue;
        }
        
        SmartDashboard.putNumber("Shooter/Distance in meters", distanceInMeters);
        // SmartDashboard.putNumber("Shooter/New Position Setpoint", newSetpoint);
        // SmartDashboard.putNumber("Shooter/Mod New Position Setpoint", modNewSetpoint);
        // SmartDashboard.putNumber("Shooter/X Velocity", xVelocity.get());
        if (topSensor.get()) { //  currentX <= Constants.FieldElements.midFieldInMeters && 
            shooterPosPot.setSetpoint(modNewSetpoint);
        } else {
            shooterPosPot.setSetpoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterPosPot.getAndSetSetPoint();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}