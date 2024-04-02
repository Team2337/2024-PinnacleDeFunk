package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTripped extends Command {
    private Supplier<Boolean> doWeHaveNote;
    private Boolean haveNote = false;


    public IntakeTripped(Supplier<Boolean> doWeHaveNote) {
        this.doWeHaveNote = doWeHaveNote;
    }

    @Override
    public void initialize() {
        haveNote = doWeHaveNote.get();
    }
    
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return haveNote; 
    }
}
