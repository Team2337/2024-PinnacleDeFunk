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
    private Supplier<Boolean> override;
    private Supplier<Boolean> bottomSensor, topSensor;


    public SetIntakeVelocity(Intake intake, Supplier<Double> xVelocity, Supplier<Boolean> shooterAtIntake, Supplier<Boolean> haveNote, Supplier<Boolean> override, Supplier<Boolean> bottomSensor, Supplier<Boolean> topSensor) {
        this.intake = intake;
        this.xVelocity = xVelocity;
        this.shooterAtIntake = shooterAtIntake;
        this.haveNote = haveNote;
        this.override = override;
        this.bottomSensor = bottomSensor;
        this.topSensor = topSensor;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        //TODO: Go through again and validate Yes we actually need to do this
        if (!override.get()) {
            if (!haveNote.get()) {
                intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
            } else if (!intake.getIntakeSensor()) {
                intake.setDriveOver(xVelocity.get() * 25 + 5);
            } else if (intake.getIntakeSensor() && topSensor.get()) {
                intake.setDriveOver(xVelocity.get() * 25 + 5);
                //intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
            } else if (intake.getIntakeSensor() && !bottomSensor.get()) {
                intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
            } else if (shooterAtIntake.get() && intake.getIntakeSensor())  {
                intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
                //intake.setDriveOver(xVelocity.get() * 25 + 5);
            } else {
                intake.stopMotors();
                
                //intake.setIntakeVelocity(Constants.Intake.INTAKE_VELOCITY);
            }
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
