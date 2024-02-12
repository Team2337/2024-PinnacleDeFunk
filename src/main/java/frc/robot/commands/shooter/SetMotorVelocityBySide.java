package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetMotorVelocityBySide extends Command {
    private Shooter shooter;
    private double leftVelocity, rightVelocity;

    public SetMotorVelocityBySide(Shooter shooter, double leftVelocity, double rightVelocity) {
        this.shooter = shooter;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        // shooter.setLeftShooterVelocity(leftVelocity);
        // shooter.setRightShooterVelocity(rightVelocity);
        shooter.setAllPercentVelocity(2, 2, 75);
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
