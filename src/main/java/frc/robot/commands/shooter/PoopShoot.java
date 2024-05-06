package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PoopShoot extends Command {
    private Shooter shooter;

    public PoopShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
            shooter.halfHalfCourt();
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
