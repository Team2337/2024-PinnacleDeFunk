package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class SetDeliverySpeed extends Command {
    private Delivery delivery;
    private double speed;

    public SetDeliverySpeed(Delivery delivery, double speed) {
        this.delivery = delivery;
        this.speed = speed;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        delivery.setDeliverySpeed(speed);
        
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
