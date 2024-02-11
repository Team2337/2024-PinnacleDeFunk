package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SetIntakeVelocity extends Command {
    private Intake intake;
    private Supplier<Double> xVelocity;
    private Supplier<Boolean> shooterAtIntake;
    private Supplier<Boolean> haveNote;


    public SetIntakeVelocity(Intake intake, Supplier<Double> xVelocity, Supplier<Boolean> shooterAtIntake, Supplier<Boolean> haveNote) {
        this.intake = intake;
        this.xVelocity = xVelocity;
        this.shooterAtIntake = shooterAtIntake;
        this.haveNote = haveNote;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (!haveNote.get()) {
            intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
        } else if (!intake.getIntakeSensor()) {
            intake.setDriveOver(xVelocity.get() * 25 + 5);
        } else if (shooterAtIntake.get())  {
            // TODO: Decide how to get shooter into position 
            //intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
            intake.setDriveOver(xVelocity.get() * 25 + 5);

        } else {
            intake.stopMotors();
        }
            
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
