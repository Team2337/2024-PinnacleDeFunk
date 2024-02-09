package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class SetElevatorSpeed extends Command {
    private Elevator elevator;
    private double speed;
    Supplier<Double> operatorY;

    public SetElevatorSpeed(Elevator elevator, Supplier<Double> operatorY) {
        this.elevator = elevator;
        this.operatorY = operatorY;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.enablePID(false);
    }
    
    @Override
    public void execute() {
        speed = operatorY.get();
        if ( Math.abs(speed) > Constants.Elevator.ELEVATOR_MAX_JOYSTICK_SPEED) {
            speed = Math.copySign(Constants.Elevator.ELEVATOR_MAX_JOYSTICK_SPEED, speed);
        }
        elevator.setElevatorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotors();
        elevator.getSetSetPoint();
        elevator.enablePID(true);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
