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
        
        delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_FORWARD_SPEED);
        
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
