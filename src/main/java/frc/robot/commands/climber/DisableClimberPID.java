package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;


public class DisableClimberPID extends Command {
    private Climber climber;
    CommandXboxController operatorJoystick;

    public DisableClimberPID(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.enablePID(false);
        climber.enableOverride();
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}
