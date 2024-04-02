package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HalfCourt extends Command {
    private Shooter shooter;
    private Supplier<Boolean> chainShot;

    public HalfCourt(Shooter shooter, Supplier<Boolean> chainShot) {
        this.shooter = shooter;
        this.chainShot = chainShot;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (!chainShot.get()) {
            shooter.halfCourt();
        } else {
            //shooter.halfCourtChain();
            shooter.halfCourt();
        }
        SmartDashboard.putBoolean("Chain Shot", chainShot.get());
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
