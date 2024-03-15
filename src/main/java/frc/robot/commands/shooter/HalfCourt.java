package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class HalfCourt extends Command {
    private Shooter shooter;
    private Supplier<Double> robotY;

    public HalfCourt(Shooter shooter, Supplier<Double> robotY) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (robotY.get() >= Constants.FieldElements.cartman && robotY.get() <= Constants.FieldElements.longwood) {
            shooter.halfCourtChain();
        } else {
            shooter.halfCourt();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycleZero();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }


}
