package frc.robot.commands.shooterPosition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPositionVelocity;

public class SetShooterPositionVelocity extends Command {
    private ShooterPositionVelocity shooterPositionVelocity;
    private double xVelocity;

    public SetShooterPositionVelocity(ShooterPositionVelocity shooterPositionVelocity, double xVelocity) {
        this.shooterPositionVelocity = shooterPositionVelocity;
        this.xVelocity = xVelocity;
        addRequirements(shooterPositionVelocity);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        shooterPositionVelocity.setShooterPositionVelocity(xVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPositionVelocity.setBrake();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
