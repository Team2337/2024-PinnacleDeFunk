package frc.robot.commands.shooterPosition;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterPosPot;


public class SetShooterPosPot extends Command {
    private ShooterPosPot shooterPosPot;
    private double speed;
    CommandXboxController operatorJoystick;
    Supplier<Boolean> operatorPovUp, operatorPovDown;

    public SetShooterPosPot(ShooterPosPot shooterPosPot, Supplier<Boolean> operatorPovUp, Supplier<Boolean> operatorPovDown) {
        this.shooterPosPot = shooterPosPot;
        this.operatorPovUp = operatorPovUp;
        this.operatorPovDown = operatorPovDown;
        addRequirements(shooterPosPot);
    }

    @Override
    public void initialize() {
        shooterPosPot.disable();
    }
    
    @Override
    public void execute() {
            if (operatorPovUp.get()) {
                speed = 0.1;
            } else if (operatorPovDown.get()) {
                speed = -0.1;
            } else {
                speed = 0;
            }
        shooterPosPot.setShooterPosPotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPosPot.stopMotor();
        shooterPosPot.getAndSetSetPoint();
        shooterPosPot.enable();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
