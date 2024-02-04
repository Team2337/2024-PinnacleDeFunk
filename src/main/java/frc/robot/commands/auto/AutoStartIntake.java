package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoStartIntake extends Command {
    
    private Intake intake;
    private double speed;

    public AutoStartIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}
    
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
        return (intake.getIntakeTopSensor()); 
    }
}
