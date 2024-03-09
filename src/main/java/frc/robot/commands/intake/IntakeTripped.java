package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTripped extends Command {
    private Intake intake;
    private Boolean haveNote = false;


    public IntakeTripped(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        haveNote = intake.getIntakeSensor();
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
