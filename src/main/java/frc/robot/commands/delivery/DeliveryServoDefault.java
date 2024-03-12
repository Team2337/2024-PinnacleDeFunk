package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliveryServo;

public class DeliveryServoDefault extends Command {
    private DeliveryServo deliveryServo;
    private Supplier<Boolean> deliveryTopSensor;
    private double waitTime = 0;

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


        if (!deliveryTopSensor.get()) {
            deliveryServo.engageNoteStop();
           
        } else if (deliveryTopSensor.get() && waitTime > 50) {
            deliveryServo.disengageNoteStop();
            waitTime = 0;
        } else {
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
