package frc.robot.commands.auto;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoGetSkipPath extends Command{
    
    private Supplier<Boolean> skipPath;
    
    public AutoGetSkipPath(Supplier<Boolean> skipPath) {
        this.skipPath = skipPath;
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return skipPath.get(); 
    }
}
