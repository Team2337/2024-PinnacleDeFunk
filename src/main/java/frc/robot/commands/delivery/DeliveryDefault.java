package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Delivery;

public class DeliveryDefault extends Command {
    private Delivery delivery;
    private double speed;

    public DeliveryDefault(Delivery delivery) {
        this.delivery = delivery;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (delivery.getDeliveryBottomSensor() && !delivery.getDeliveryTopSensor()) {
            delivery.setDeliverySpeed(speed);
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
