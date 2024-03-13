package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliveryServo;

public class DeliveryServoOverride extends Command {
    private DeliveryServo deliveryServo;
    
    public DeliveryServoOverride(DeliveryServo deliveryServo) {
        this.deliveryServo = deliveryServo;
        addRequirements(deliveryServo);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
            deliveryServo.disengageNoteStop();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
