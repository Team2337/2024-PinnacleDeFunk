package frc.robot.commands.delivery;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Delivery;

public class DelayedDelivery extends Command {
    private Delivery delivery;
    private double speed;
    private Supplier<Boolean> upToSpeed, shooterInPos;

    public DelayedDelivery(Delivery delivery, double speed, Supplier<Boolean> upToSpeed, Supplier<Boolean> shooterInPos) {
        this.delivery = delivery;
        this.speed = speed;
        this.upToSpeed = upToSpeed;
        this.shooterInPos = shooterInPos;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
            if (upToSpeed.get() && shooterInPos.get()) {
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
