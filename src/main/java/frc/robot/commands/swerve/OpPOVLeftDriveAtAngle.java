package frc.robot.commands.swerve;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class OpPOVLeftDriveAtAngle extends InstantCommand {
    private Drivetrain drivetrain;
    private Supplier<Boolean> isAllianceColorBlue;

    public OpPOVLeftDriveAtAngle(Drivetrain drivetrain, Supplier<Boolean> isAllianceColorBlue) {
        this.drivetrain = drivetrain;
        this.isAllianceColorBlue = isAllianceColorBlue;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (drivetrain.getState().Pose.getX() > Constants.FieldElements.midFieldInMeters) {
            if (isAllianceColorBlue.get()) {
                drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_BLUE);
            } else {
                drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_RED);
            }
        } else {
            if (isAllianceColorBlue.get()) {
                drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_SNIPE_BLUE);
            } else {
                drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_SNIPE_RED);
            }
        }
    }
}