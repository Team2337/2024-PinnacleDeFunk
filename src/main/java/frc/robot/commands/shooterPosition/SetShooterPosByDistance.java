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
    private double speakerX, speakerY, currentX, currentY, distanceInMeters, newSetpoint, modNewSetpoint;
    private double minStringPotValue = 5;//5.3;
    private double maxStringPotValue = 12;//10.1;
    private Supplier<String> allianceColor;
    private Supplier<Double> xVelocity;


    public SetShooterPosByDistance(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose, Supplier<String> allianceColor, Supplier<Double> xVelocity) {
        this.shooterPosPot = shooterPosPot;
        this.currentPose = currentPose;
        this.allianceColor = allianceColor;
        this.xVelocity = xVelocity;
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
        //newSetpoint = (-0.20335152 * Math.pow(distanceInMeters, 2)) + (2.4585446 * distanceInMeters) + 2.7245656;
        newSetpoint = (-0.28906779 * Math.pow(distanceInMeters, 2)) + (3.3739266 * distanceInMeters) + 1.6819181;

        modNewSetpoint = newSetpoint + (xVelocity.get() / 2);

        if (modNewSetpoint < minStringPotValue) {
            modNewSetpoint = minStringPotValue;
        } 
        if (modNewSetpoint > maxStringPotValue) {
            modNewSetpoint = maxStringPotValue;
        }
        
        SmartDashboard.putNumber("Shooter/Distance in meters", distanceInMeters);
        SmartDashboard.putNumber("Shooter/New Position Setpoint", newSetpoint);
        SmartDashboard.putNumber("Shooter/Mod New Position Setpoint", modNewSetpoint);
        SmartDashboard.putNumber("Shooter/X Velocity", xVelocity.get());
        if (currentY <= Constants.FieldElements.midFieldInMeters) { 
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
