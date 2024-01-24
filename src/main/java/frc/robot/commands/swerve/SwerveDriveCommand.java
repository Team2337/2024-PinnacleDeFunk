package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Drivetrain;

public class SwerveDriveCommand extends Command{
    
    private Drivetrain drivetrain;
    private CommandXboxController driverJoystick;
    private SwerveRequest swerveRequest;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Swerve.angularDeadband);
    
    private SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Swerve.angularDeadband);

    private final SwerveRequest.SwerveDriveBrake lockdown = new SwerveRequest.SwerveDriveBrake();

    private PhoenixPIDController turnPID = new PhoenixPIDController(5, 0, 0);
    private double forward, rotation, strafe = 0;

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController driverJoystick) {
        this.drivetrain = drivetrain;
        this.driverJoystick = driverJoystick;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveFacingAngle.HeadingController = turnPID;
    }

    @Override
    public void execute() {

        //Read Inputs From Joysticks
        forward = Utilities.deadband(-driverJoystick.getLeftY(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        strafe = Utilities.deadband(-driverJoystick.getLeftX(), Constants.Swerve.driveDeadband) * (Constants.Swerve.MaxSpeed/Constants.Swerve.driveAdjustment);
        rotation = -driverJoystick.getRightX() * Constants.Swerve.MaxAngularRate;

        //If we have set an angle to drive at and driver has hit drive at angle button, drive at that angle
        if ((drivetrain.rotationAngle != 0) && (drivetrain.driveAtAngle)) {
            swerveRequest = driveFacingAngle
                .withTargetDirection(Rotation2d.fromDegrees(drivetrain.rotationAngle))
                .withVelocityX(forward)
                .withVelocityY(strafe);
            // If endgame switch is true, drive robot centric
        } else if (drivetrain.endGame) {
            swerveRequest = robotCentric
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotation);
            // Enable Lockdown Mode
        } else if (drivetrain.lockdownEnabled) {
            swerveRequest = lockdown;
            // Otherwise, drive field centric
        } else {
            swerveRequest = drive
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotation);
        }
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
