package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetMotorSpeed extends Command {
    private Intake intake;
    private double speed;

    public SetMotorSpeed(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(speed);
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
