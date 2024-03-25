package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class AutoStartDeliveryToSensor extends Command{
    
    private Delivery delivery;

    public AutoStartDeliveryToSensor(Delivery delivery) {
        this.delivery = delivery;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        
        if (!delivery.getDeliveryBottomSensor() && !delivery.getDeliveryTopSensor()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_FORWARD_SPEED);
        } else if (delivery.getDeliveryBottomSensor() && !delivery.getDeliveryTopSensor()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_SLOW_SPEED);
            //TODO: delivery.setDeliverySpeed(0.45);
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
        return delivery.getDeliveryTopSensor(); 
    }
}
