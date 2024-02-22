package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoStartShoot extends Command{
    
    private Shooter shooter;

    public AutoStartShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        shooter.setAllPercentVelocity();
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
