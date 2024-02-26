package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Delivery;

public class SetDeliverySpeed extends Command {
    private Delivery delivery;
    private double speed;
    private Supplier<Boolean> upToSpeed;

    public SetDeliverySpeed(Delivery delivery, double speed, Supplier<Boolean> upToSpeed) {
        this.delivery = delivery;
        this.speed = speed;
        this.upToSpeed = upToSpeed;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (upToSpeed.get()) {
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
