package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.nerdyfiles.vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class VisionRotate extends Command{
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Swerve.angularDeadband);
    private Drivetrain drivetrain;
    private CommandXboxController driverJoystick;
    private SwerveRequest swerveRequest;
    private double forward, rotation, strafe = 0;
    private String limelightName;
    private double kP = 0.25;

    public VisionRotate(Drivetrain drivetrain, CommandXboxController driverJoystick, String limelightName) {
        this.drivetrain = drivetrain;
        this.driverJoystick = driverJoystick;
        this.limelightName = limelightName;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 3);
    }

    @Override
    public void execute() {

        //Read Inputs From Joysticks
        forward = Utilities.deadband(-driverJoystick.getLeftY(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        strafe = Utilities.deadband(-driverJoystick.getLeftX(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        rotation = LimelightHelpers.getTX(limelightName);
        //if(Math.abs(rotation) > 0) {
            rotation = Utilities.scaleVisionToOne(rotation);
            rotation = rotation * Constants.Swerve.MaxAngularRate;
            rotation = -rotation * kP;
        // } else {
        //     rotation = 0;
        // }
        SmartDashboard.putNumber("Rotation", rotation);
        //rotation = 0;
        swerveRequest = drive
        .withVelocityX(forward)
        .withVelocityY(strafe)
        .withRotationalRate(rotation);
        
        drivetrain.setControl(swerveRequest);


    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
