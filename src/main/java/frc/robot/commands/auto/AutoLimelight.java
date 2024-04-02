package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.nerdyfiles.vision.LimelightHelpers;

public class AutoLimelight extends Command{
    private boolean seeNote = false;
    private Supplier<Boolean> startDetection;
    
    public AutoLimelight(Supplier<Boolean> startDetection) {
        this.startDetection = startDetection;
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        if (startDetection.get()) {
            var lastResult = LimelightHelpers.getLatestResults("limelight-coral").targetingResults;
            seeNote = lastResult.valid;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return seeNote; 
    }
}
