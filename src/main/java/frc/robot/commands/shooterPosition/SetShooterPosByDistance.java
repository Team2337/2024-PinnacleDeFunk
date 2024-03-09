package frc.robot.commands.shooterPosition;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private double minStringPotValue = 7.5;//5.3;
    private double maxStringPotValue = 15;//10.1;
    private Supplier<String> allianceColor;
    private Supplier<Double> xVelocity;
    private Supplier<Boolean> topSensor;
    private double distanceToFloor = 1.4478; //TODO:  Validate to center of tags 4 + 7
    private double cameraHeight = 0.2667;


    public SetShooterPosByDistance(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose, Supplier<String> allianceColor, Supplier<Double> xVelocity, Supplier<Boolean> topSensor) {
        this.shooterPosPot = shooterPosPot;
        this.currentPose = currentPose;
        this.allianceColor = allianceColor;
        this.xVelocity = xVelocity;
        this.topSensor = topSensor;
        addRequirements(shooterPosPot);
    }

    @Override
    public void initialize() {

        if (allianceColor.get() == "blue") {
             NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry("priorityid").setNumber(7);
        } else {
             NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry("priorityid").setNumber(4);
        }

    }
    
    @Override
    public void execute() {
        double visionDistance = NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry("botpose_avgdist,");
        LimelightHelpers.getBotPose(getName());
        double visionDistanceInMeters = Math.sqrt(Math.pow(visionDistance, 2) + Math.pow((distanceToFloor-cameraHeight), 2));
        SmartDashboard.putNumber("Shooter/vision Distance in meters", visionDistanceInMeters);



        if (allianceColor.get() == "blue") {
            speakerPose = Constants.FieldElements.blueSpeakerCenter;
        } else {
            speakerPose = Constants.FieldElements.redSpeakerCenter;
        }
        speakerX = speakerPose.getX();      
        speakerY = speakerPose.getY(); 
        currentX = currentPose.get().getX();
        currentY = currentPose.get().getY();
        distanceInMeters = Math.sqrt(Math.pow((currentX - speakerX), 2) + Math.pow((currentY - speakerY), 2));
        newSetpoint = (-0.44429609 * Math.pow(distanceInMeters, 2)) + (4.3429355 * distanceInMeters) + 3.3590452; //FUDGE
        //newSetpoint = (-0.42026111 * Math.pow(distanceInMeters, 2)) + (4.2693814 * distanceInMeters) + 3.2881356; //RAW
        //newSetpoint = (-0.42233788 * Math.pow(distanceInMeters, 2)) + (4.2114015 * distanceInMeters) + 3.5361784; //REMOVE POINTS

        // if (xVelocity.get() > 0) {
        //     modNewSetpoint = newSetpoint + (xVelocity.get() / 2.1); // TODO: Was 2
        // } else if (xVelocity.get() < 0) {
        //     modNewSetpoint = newSetpoint + (xVelocity.get() / 1.5); // TODO: Was 2
        // }

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
        if (currentY <= Constants.FieldElements.midFieldInMeters && topSensor.get()) { //TODO  
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
