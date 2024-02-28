package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetMotorVelocityBySide extends Command {
    private Shooter shooter;
    private Supplier<Boolean> ampMode;

    public SetMotorVelocityBySide(Shooter shooter, Supplier<Boolean> ampMode) {
        this.shooter = shooter;
        this.ampMode = ampMode;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        //shooter.setAllPercentVelocityByPercent(2, 2, 75);
        if (!ampMode.get()) {
            shooter.setAllPercentVelocity();
        } else {
            shooter.setAllPercentVelocityAmp();
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
