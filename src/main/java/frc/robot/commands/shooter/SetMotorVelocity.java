package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetMotorVelocity extends Command {
    private Shooter shooter;
    private double speed;

    public SetMotorVelocity(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        shooter.setShooterVelocity(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setBrake();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }


}
