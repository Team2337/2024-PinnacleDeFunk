package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetMotorVelocityBySide extends Command {
    private Shooter shooter;

    public SetMotorVelocityBySide(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        //shooter.setAllPercentVelocityByPercent(2, 2, 75);
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
