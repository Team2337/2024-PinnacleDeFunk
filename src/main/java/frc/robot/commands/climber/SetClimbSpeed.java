package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberPosition;


public class SetClimbSpeed extends Command {
    private ClimberPosition climber;
    private double speed;
    Supplier<Double> operatorY;

    public SetClimbSpeed(ClimberPosition climber, Supplier<Double> operatorY) {
        this.climber = climber;
        this.operatorY = operatorY;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
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
        climber.setClimberPosition(climber.getClimberPosition());
        climber.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
