package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class DeliveryFinger extends Command {
    private Delivery delivery;
    private Supplier<Boolean> intakeSensor;
    private double waitTime = 0;

    public DeliveryFinger(Delivery delivery, Supplier<Boolean> intakeSensor) {
        this.delivery = delivery;
        this.intakeSensor = intakeSensor;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if(delivery.getDeliveryBottomSensor() || intakeSensor.get()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_FORWARD_SPEED);
            waitTime = 0;
        } else if (delivery.getDeliveryTopSensor() && waitTime < 7) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_SLOW_SPEED);
            waitTime++;
        } else {
            delivery.stopMotors();
        }

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
