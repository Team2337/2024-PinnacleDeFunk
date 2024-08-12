package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.ShooterPosPot;


public class AutoShooterPos extends Command {
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
    private double prevX, prevY;



    public AutoShooterPos(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose, Supplier<String> allianceColor, Supplier<Double> xVelocity, Supplier<Double> yVelocity, Supplier<Boolean> topSensor) {
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

        //Checks alliance to un-flip gyro from driving
        if(allianceColor.get() == "blue") {
            LimelightHelpers.SetRobotOrientation("limelight-blue", currentPose.get().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        } else {
            LimelightHelpers.SetRobotOrientation("limelight-blue", (currentPose.get().getRotation().getDegrees() - 0), 0, 0, 0, 0, 0);
        }

        //Gets Vision Pos
        LimelightHelpers.PoseEstimate mt2_blue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-blue");

        //Sets the target goal
        if (allianceColor.get() == "blue") {
            speakerPose = Constants.FieldElements.blueSpeakerCenter;
        } else {
            speakerPose = Constants.FieldElements.redSpeakerCenter;
        }

        speakerX = speakerPose.getX();      
        speakerY = speakerPose.getY(); 

        SmartDashboard.putNumber("Auto Shots/Vision X", mt2_blue.pose.getX());
        SmartDashboard.putNumber("Auto Shots/Vision Y", mt2_blue.pose.getY());
        SmartDashboard.putNumber("Auto Shots/Robot X", currentPose.get().getX());
        SmartDashboard.putNumber("Auto Shots/Robot Y", currentPose.get().getY());
        SmartDashboard.putNumber("Auto Shots/Robot Dis", Math.sqrt(Math.pow((currentPose.get().getX() - speakerX), 2) + Math.pow((currentPose.get().getY() - speakerY), 2)));

        if (mt2_blue.tagCount > 0) {
            currentX = mt2_blue.pose.getX(); //(currentPose.get().getX() - (xVelocity.get()/5));
            currentY = mt2_blue.pose.getY(); //(currentPose.get().getY() - (yVelocity.get()/5));

            prevX = mt2_blue.pose.getX();
            prevY = mt2_blue.pose.getY();
        }
        distanceInMeters = Math.sqrt(Math.pow((currentX - speakerX), 2) + Math.pow((currentY - speakerY), 2));
        newSetpoint = (0.114 * Math.pow(distanceInMeters, 3)) + (-1.64 * Math.pow(distanceInMeters, 2)) + (8.34 * distanceInMeters) + -1.127;//-0.927 shot low //1.127 Blue -0.2

        

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
        
        SmartDashboard.putNumber("Auto/Vision Distance", distanceInMeters);
        // SmartDashboard.putNumber("Shooter/New Position Setpoint", newSetpoint);
        // SmartDashboard.putNumber("Shooter/Mod New Position Setpoint", modNewSetpoint);
        // SmartDashboard.putNumber("Shooter/X Velocity", xVelocity.get());
        if (topSensor.get()) { //  currentX <= Constants.FieldElements.midFieldInMeters && 
            //shooterPosPot.setSetpoint(modNewSetpoint);
            shooterPosPot.setSetpoint(13.7);//13.5, 13.8
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