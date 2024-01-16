// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.nerdyfiles.utilities.Utilities;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double driveAdjustment = 1;
  private double driveDeadband = 0.1;
  private double angularDeadband = 0.1;
  private double driveScale = 5;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withRotationalDeadband(MaxAngularRate * angularDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withRotationalDeadband(MaxAngularRate * angularDeadband) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 


  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    joystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    
        drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(Utilities.deadband(-joystick.getLeftY(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive forward with negative Y (forward)
            .withVelocityY(Utilities.deadband(-joystick.getLeftX(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.rightBumper().onTrue(drivetrain.applyRequest(() -> driveFacingAngle.withTargetDirection(new Rotation2d().fromDegrees(60))
      .withVelocityX(Utilities.deadband(-joystick.getLeftY(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive forward with negative Y (forward)
      .withVelocityY(Utilities.deadband(-joystick.getLeftX(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive left with negative X (left)
      ).ignoringDisable(true));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }
  public void setMaxSpeed(double speed) {
    driveAdjustment = speed;
  }

  public RobotContainer() {

    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
