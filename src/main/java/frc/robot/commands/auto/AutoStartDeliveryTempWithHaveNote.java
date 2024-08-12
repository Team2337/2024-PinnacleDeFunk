package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class AutoStartDeliveryTempWithHaveNote extends Command{
    
    private Delivery delivery;
    private Supplier<Boolean> haveNote;
    private int i;

    public AutoStartDeliveryTempWithHaveNote(Delivery delivery, Supplier<Boolean> haveNote) {
        this.delivery = delivery;
        this.haveNote = haveNote;
        addRequirements(delivery);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        
        delivery.setDeliverySpeed(Constants.Delivery.DELIVERY_FORWARD_SPEED);
        i++;
    }

    @Override
    public void end(boolean interrupted) {
        delivery.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return (!haveNote.get() || (i > 50)); 
    }
}
