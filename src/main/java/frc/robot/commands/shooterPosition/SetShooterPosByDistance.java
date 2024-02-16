package frc.robot.commands.shooterPosition;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.ShooterPosPot;


public class SetShooterPosByDistance extends Command {
    private ShooterPosPot shooterPosPot;
    private Supplier<Pose2d> currentPose;
    private Translation2d speakerPose;
    private double speakerX, speakerY, currentX, currentY, distanceInMeters, newSetpoint;
    private double minMetersFromSpeaker = 1.335;
    private double maxMetersFromSpeaker = 4.15;//5.08;
    private double minStringPotValue = 5.3;//5.15;
    private double maxStringPotValue = 9.95;


    public SetShooterPosByDistance(ShooterPosPot shooterPosPot, Supplier<Pose2d> currentPose) {
        this.shooterPosPot = shooterPosPot;
        this.currentPose = currentPose;
        addRequirements(shooterPosPot);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        speakerPose = Constants.FieldElements.blueSpeakerCenter;
        speakerX = speakerPose.getX();      
        speakerY = speakerPose.getY(); 
        currentX = currentPose.get().getX();
        currentY = currentPose.get().getY();
        distanceInMeters = Math.sqrt(Math.pow((currentX - speakerX), 2) + Math.pow((currentY - speakerY), 2));
        newSetpoint = Utilities.scaleAnyToAny(distanceInMeters, minMetersFromSpeaker, maxMetersFromSpeaker, minStringPotValue, maxStringPotValue);
        
        if (newSetpoint < minStringPotValue) {
            newSetpoint = minStringPotValue;
        } 
        if (newSetpoint > maxStringPotValue) {
            newSetpoint = maxStringPotValue;
        }
        
        SmartDashboard.putNumber("Shooter/Distance in meters", distanceInMeters);
        SmartDashboard.putNumber("Shooter/New Position Setpoint", newSetpoint);
        shooterPosPot.setSetpoint(newSetpoint);
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
