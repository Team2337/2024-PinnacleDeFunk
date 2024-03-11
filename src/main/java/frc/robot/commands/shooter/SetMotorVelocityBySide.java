package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetMotorVelocityBySide extends Command {
    private Shooter shooter;
    private Supplier<Boolean> ampMode;
    private Supplier<Boolean> trapMode;

    public SetMotorVelocityBySide(Shooter shooter, Supplier<Boolean> ampMode, Supplier<Boolean> trapMode) {
        this.shooter = shooter;
        this.ampMode = ampMode;
        this.trapMode = trapMode;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        //shooter.setAllPercentVelocityByPercent(2, 2, 75);
        if (ampMode.get()) {
            shooter.setAllPercentVelocityAmp();
        } else if (trapMode.get()) {
            shooter.setAllPercentVelocityTrap();
        } else {
            shooter.setAllPercentVelocity();
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
