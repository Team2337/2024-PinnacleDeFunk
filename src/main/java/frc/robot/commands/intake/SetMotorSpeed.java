package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetMotorSpeed extends Command {
    private Intake intake;
    private double speed;
    private Supplier<Boolean> haveNote;

    public SetMotorSpeed(Intake intake, double speed, Supplier<Boolean> haveNote) {
        this.intake = intake;
        this.speed = speed;
        this.haveNote = haveNote;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        intake.setIntakeVelocity(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }

    @Override
    public boolean isFinished() {
        //return haveNote.get(); 
        return false;
    }
}
