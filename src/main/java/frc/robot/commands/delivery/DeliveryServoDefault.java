package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliveryServo;

public class DeliveryServoDefault extends Command {
    private DeliveryServo deliveryServo;
    private Supplier<Boolean> deliveryTopSensor;
    private double waitTime, waitTime2 = 0;

    public DeliveryServoDefault(DeliveryServo deliveryServo, Supplier<Boolean> deliveryTopSensor) {
        this.deliveryServo = deliveryServo;
        this.deliveryTopSensor = deliveryTopSensor;
        addRequirements(deliveryServo);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {


        if (!deliveryTopSensor.get() && waitTime2 > 10) {
            deliveryServo.engageNoteStop();
           waitTime2 = 0;
        } else if (deliveryTopSensor.get() && waitTime > 13.5) {
            deliveryServo.disengageNoteStop();
            waitTime = 0;
        } else if (!deliveryTopSensor.get()) {
             waitTime2++;
        } else if (deliveryTopSensor.get()) {
             waitTime++;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
