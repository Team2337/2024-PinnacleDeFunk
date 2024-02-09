package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeVelocity extends Command {
    private Intake intake;
    private Supplier<Double> xVelocity;

    public SetIntakeVelocity(Intake intake, Supplier<Double> xVelocity) {
        this.intake = intake;
        this.xVelocity = xVelocity;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        intake.setDriveOver(xVelocity.get() * 25);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
