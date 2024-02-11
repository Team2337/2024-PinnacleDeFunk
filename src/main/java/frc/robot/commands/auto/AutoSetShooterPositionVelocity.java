package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPositionVelocity;

public class AutoSetShooterPositionVelocity extends Command {
    private ShooterPositionVelocity shooterPositionVelocity;
    private double xVelocity, position;
    private boolean atPosition;

    public AutoSetShooterPositionVelocity(ShooterPositionVelocity shooterPositionVelocity, double xVelocity, double position) {
        this.shooterPositionVelocity = shooterPositionVelocity;
        this.xVelocity = xVelocity;
        this.position = position;
        addRequirements(shooterPositionVelocity);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if(xVelocity > 0 && (shooterPositionVelocity.getShooterPositionPosition() > position)) {
            atPosition = true;
        } else if(xVelocity < 0 && (shooterPositionVelocity.getShooterPositionPosition() < (position + 1))) {
            atPosition = true;
        } else {
            atPosition = false;
        }
        shooterPositionVelocity.setShooterPositionVelocity(xVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPositionVelocity.setBrake();
    }

    @Override
    public boolean isFinished() {
        return atPosition; 
    }
}
