package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class DeliveryDefault extends Command {
    private Delivery delivery;
    private Supplier<Boolean> intakeSensor;
    private Supplier<Double> shooterSpeed;
    private double waitTime = 0;

    public DeliveryDefault(Delivery delivery, Supplier<Boolean> intakeSensor, Supplier<Double> shooterSpeed) {
        this.delivery = delivery;
        this.intakeSensor = intakeSensor;
        this.shooterSpeed = shooterSpeed;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if(delivery.getDeliveryTopSensor()) {
            delivery.stopMotors();
        } else if (delivery.getDeliveryBottomSensor()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_SLOW_SPEED);
        } else if (intakeSensor.get()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_FORWARD_SPEED);
        } else {
            delivery.stopMotors();
        }

        // if (!delivery.getDeliveryTopSensor()) {
        //     delivery.engageNoteStop();
           
        // } else if (delivery.getDeliveryTopSensor() && waitTime > 50) {
        //     delivery.disengageNoteStop();
        //     waitTime = 0;
        // } else {
        //      waitTime++;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        delivery.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
