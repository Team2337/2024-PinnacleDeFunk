package frc.robot.commands.shooterPosition;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPosPot;


public class SetShooterPosByDistance extends Command {
    private ShooterPosPot shooterPosPot;
    private Supplier<Pose2d> currentPose;
    private Translation2d speakerPose;
    private double speakerX, speakerY, currentX, currentY, distanceInMeters, newSetpoint;
    private double minStringPotValue = 5.3;
    private double maxStringPotValue = 10.1;
    private Supplier<String> allianceColor;


    public SetShooterPosByDistance(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose, Supplier<String> allianceColor) {
        this.shooterPosPot = shooterPosPot;
        this.currentPose = currentPose;
        this.allianceColor = allianceColor;
        addRequirements(shooterPosPot);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
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
        //TODO: Make a new one
        newSetpoint = (-0.20335152 * Math.pow(distanceInMeters, 2)) + (2.4585446 * distanceInMeters) + 2.7245656;
        
        if (newSetpoint < minStringPotValue) {
            newSetpoint = minStringPotValue;
        } 
        if (newSetpoint > maxStringPotValue) {
            newSetpoint = maxStringPotValue;
        }
        
        SmartDashboard.putNumber("Shooter/Distance in meters", distanceInMeters);
        SmartDashboard.putNumber("Shooter/New Position Setpoint", newSetpoint);
        if (currentY <= Constants.FieldElements.midFieldInMeters) { 
            shooterPosPot.setSetpoint(newSetpoint);
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
