package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Delivery;

public class DelayedDelivery extends Command {
    private Delivery delivery;
    private double speed;
    private Supplier<Boolean> upToSpeed;
    private double wait = 0;

    public DelayedDelivery(Delivery delivery, double speed, Supplier<Boolean> upToSpeed) {
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
        if (wait >= 25) {
            if (upToSpeed.get()) {
                delivery.setDeliverySpeed(speed);
            } else {
                delivery.stopMotors();
            }
        } else {
            wait++;
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
