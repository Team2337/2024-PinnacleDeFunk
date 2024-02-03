package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;


public class SetClimbSpeed extends Command {
    private Climber climber;
    CommandXboxController operatorJoystick;

    public SetClimbSpeed(Climber climber, CommandXboxController operatorJoystick) {
        this.climber = climber;
        this.operatorJoystick = operatorJoystick;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    if (climber.getOverrideState()) {
        climber.enablePID(false);
        }
    }
    
    @Override
    public void execute() {
        if (climber.getOverrideState()) {
                climber.setClimbSpeed(Utilities.deadband(operatorJoystick.getRightY(), 0.1));
            }
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
