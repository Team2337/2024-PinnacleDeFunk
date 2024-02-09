package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;


public class SetClimbSpeed extends Command {
    private Climber climber;
    private double speed;
    Supplier<Double> operatorY;

    public SetClimbSpeed(Climber climber, Supplier<Double> operatorY) {
        this.climber = climber;
        this.operatorY = operatorY;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.enablePID(false);
    }
    
    @Override
    public void execute() {
        speed = operatorY.get();
        if ( Math.abs(speed) > Constants.Climber.CLIMBER_MAX_JOYSTICK_SPEED) {
            speed = Math.copySign(Constants.Climber.CLIMBER_MAX_JOYSTICK_SPEED, speed);
        }
        climber.setClimbSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotors();
        climber.getSetSetPoint();
        climber.enablePID(true);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
