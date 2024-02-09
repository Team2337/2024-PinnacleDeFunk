package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class AutoStartDelivery extends Command{
    
    private Delivery delivery;
    private Supplier<Boolean> upToSpeed;

    public AutoStartDelivery(Delivery delivery, Supplier<Boolean> upToSpeed) {
        this.delivery = delivery;
        this.upToSpeed = upToSpeed;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (upToSpeed.get()) {
            delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_SPEED);
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
