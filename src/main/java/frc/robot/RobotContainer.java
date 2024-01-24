// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.delivery.SetDeliverySpeed;
import frc.robot.commands.intake.SetMotorSpeed;
import frc.robot.commands.shooter.SetMotorVelocity;
import frc.robot.commands.shooter.SetMotorVelocityBySide;
import frc.robot.generated.TunerConstants;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPosition;

public class RobotContainer {
  

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1); 

  public final Drivetrain drivetrain = TunerConstants.DriveTrain; 

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
  //private Command runAuto = drivetrain.getAutoPath("DifferentAuto");

  private final Delivery delivery = new Delivery();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final ShooterPosition shooterPosition = new ShooterPosition();
  private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);
  private final SendableChooser<Command> autonChooser;
  

  private void configureBindings() {
    driverJoystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(Constants.Swerve.driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    
        drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(Utilities.deadband(-driverJoystick.getLeftY(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive forward with negative Y (forward)
            .withVelocityY(Utilities.deadband(-driverJoystick.getLeftX(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive left with negative X (left)
            .withRotationalRate(driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));
    driverJoystick.rightBumper().onTrue(drivetrain.applyRequest(() -> driveFacingAngle.withTargetDirection(Rotation2d.fromDegrees(60))
      .withVelocityX(Utilities.deadband(-driverJoystick.getLeftY(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive forward with negative Y (forward)
      .withVelocityY(Utilities.deadband(-driverJoystick.getLeftX(), driveDeadband) * (MaxSpeed/driveAdjustment)) // Drive left with negative X (left)
      ).ignoringDisable(true));

    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    //*************Operator Control ******************/
    operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 0.1));
    operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -0.1));
    operatorJoystick.x().whileTrue(new SetMotorVelocityBySide(shooter, 500, 1000));
    operatorJoystick.y().whileTrue(new SetMotorVelocity(shooter, 1000));
    operatorJoystick.b().onTrue(new InstantCommand(() -> shooterPosition.setShooterPosition(10), shooterPosition));
    operatorJoystick.a().whileTrue(new SetDeliverySpeed(delivery, 0.1));
  }
  public void setMaxSpeed(double speed) {
    driveAdjustment = speed;
  }

  public RobotContainer() {
    
    NamedCommands.registerCommand("StartIntake", new SetMotorSpeed(intake, 0.1));
    NamedCommands.registerCommand("StopIntake", new SetMotorSpeed(intake, 0.0));
    NamedCommands.registerCommand("StartShooter", new SetMotorVelocity(shooter, 5));
    NamedCommands.registerCommand("StopShooter", new SetMotorVelocity(shooter, 0));
    autonChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autonChooser);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autonChooser.getSelected();
  }


}
