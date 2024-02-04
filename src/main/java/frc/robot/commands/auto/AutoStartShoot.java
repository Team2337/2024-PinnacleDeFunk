package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoStartShoot extends Command{
    
    private Shooter shooter;
    private double velocity;

    public AutoStartShoot(Shooter shooter, double velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        shooter.setShooterVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
