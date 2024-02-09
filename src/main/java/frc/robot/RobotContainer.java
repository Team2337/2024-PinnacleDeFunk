// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climber.SetClimbSpeed;
import frc.robot.commands.delivery.DeliveryDefault;
import frc.robot.commands.delivery.SetDeliverySpeed;
import frc.robot.commands.elevator.SetElevatorSpeed;
import frc.robot.commands.intake.SetIntakeVelocity;
import frc.robot.commands.intake.SetMotorSpeed;
import frc.robot.commands.intake.SetTempIntakeVelocity;
import frc.robot.commands.shooter.SetMotorVelocity;
import frc.robot.commands.shooter.SetMotorVelocityBySide;
import frc.robot.commands.shooterPosition.SetShooterPositionVelocity;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.swerve.VisionRotate;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsPractice;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.ShooterPositionVelocity;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightColor;

public class RobotContainer {
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1); 
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);
  private final CommandXboxController testJoystick = new CommandXboxController(5);

  public final Drivetrain drivetrain;
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  


  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("DifferentAuto");

  private final Climber climb = new Climber(operatorJoystick);
  private final Delivery delivery = new Delivery();
  private final Elevator elevator = new Elevator(operatorJoystick);
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  //private final ShooterPosition shooterPosition = new ShooterPosition();
  private final ShooterPositionVelocity shooterPositionVelocity = new ShooterPositionVelocity();
  private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);
  private final Vision vision = new Vision(this);
  private final SendableChooser<Command> autonChooser;

  public String allianceColor = null;
  
  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick));
    //intake.setDefaultCommand(new SetIntakeVelocity(intake, () -> getDrivetrainVelocityX(), () -> isShooterAtIntake(), () -> doWeHaveNote()));    
    delivery.setDefaultCommand(new DeliveryDefault(delivery));
    intake.setDefaultCommand(new SetTempIntakeVelocity(intake, () -> getDrivetrainVelocityX()));

    driverJoystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(Constants.Swerve.driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    driverJoystick.povLeft().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(90)));
    driverJoystick.povRight().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(-90)));
    driverJoystick.povUp().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(1)));
    driverJoystick.povDown().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(179)));

    driverJoystick.a().onTrue(new InstantCommand(() -> drivetrain.setPointAtSpeaker(true)));
    driverJoystick.a().onFalse(new InstantCommand(() -> drivetrain.setPointAtSpeaker(false)));
    driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));
    driverJoystick.x().toggleOnTrue(new InstantCommand(() -> drivetrain.setToDriveAtAngle()));
    driverJoystick.y().toggleOnTrue(new InstantCommand(() -> drivetrain.enableLockdown()));
    
    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverJoystick.rightBumper().whileTrue(new VisionRotate(drivetrain, driverJoystick, "limelight-orange"));

    //*************Operator Control ******************/
    // operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 0.1, () -> doWeHaveNote()));
    // operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -0.1, () -> doWeHaveNote()));
    operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 40, () -> doWeHaveNote()));
    operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -40, () -> doWeHaveNote()));
    operatorJoystick.x().whileTrue(new SetMotorVelocityBySide(shooter, 70, 65));
    operatorJoystick.y().whileTrue(new SetMotorVelocity(shooter, 70));
    operatorJoystick.a().whileTrue(new SetDeliverySpeed(delivery));
    //operatorJoystick.povUp().onTrue(new InstantCommand(() -> shooterPosition.setShooterPosition(30)));
    // operatorJoystick.povUp().whileTrue(new InstantCommand(() -> shooterPositionVelocity.setShooterPositionVelocity(5)));
    // operatorJoystick.povDown().whileTrue(new InstantCommand(() -> shooterPositionVelocity.setShooterPositionVelocity(-5)));
    operatorJoystick.povUp().whileTrue(new SetShooterPositionVelocity(shooterPositionVelocity, 5));
    operatorJoystick.povDown().whileTrue(new SetShooterPositionVelocity(shooterPositionVelocity, -5));

    // operatorJoystick.b().onTrue(new InstantCommand(() -> climb.setClimberSetpoint(2.06)));
    // operatorJoystick.b().onFalse(new InstantCommand(() -> climb.getSetSetPoint()));
    // operatorJoystick.a().onTrue(new InstantCommand(() -> climb.setClimberSetpoint(10)));
    // operatorJoystick.a().onFalse(new InstantCommand(() -> climb.getSetSetPoint()));
    // operatorJoystick.povUp().whileTrue(new SetClimbSpeed(climb, () -> Utilities.deadband(operatorJoystick.getRightY(), 0.1)));

    
    //*************Operator Station *****************/
    // operatorStation.blackSwitch.onTrue(new InstantCommand(() -> drivetrain.setEndGame(true)));
    // operatorStation.blackSwitch.onFalse(new InstantCommand(() -> drivetrain.setEndGame(false)));
    
     //************* Test Joystick *****************/
     // testJoystick.a().onTrue(new InstantCommand(() -> climb.setClimberSetpoint(10)));
    }
  public void setMaxSpeed(double speed) {
    Constants.Swerve.driveAdjustment = speed;
  }

  public RobotContainer() {
    RobotType.Type robotType = RobotType.getRobotType();
    switch (robotType) {
      case SKILLSBOT:
        drivetrain = TunerConstants.DriveTrain;
        break;
      case PRACTICE:
        drivetrain = TunerConstantsPractice.DriveTrain;
        break;
      case COMPETITION:
      default:
        drivetrain = TunerConstants.DriveTrain;

       
        break;
    }
    

    NamedCommands.registerCommand("StartIntake", new SetMotorSpeed(intake, 0.1, () -> doWeHaveNote()));
    NamedCommands.registerCommand("StopIntake", new SetMotorSpeed(intake, 0.0, () -> doWeHaveNote()));
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

  public double getVisionLatency(LimelightColor color) {
    return vision.getLatency(color);
  }

  public double getDrivetrainVelocityX() {
    //return drivetrain.drivetrainVelocityX;
    return logger.getXVelocity();
  }

  public boolean isShooterAtTrap() {
    //return shooterPosition.shooterAtTrap;
    return shooterPositionVelocity.shooterAtIntake;
  }
  
  public boolean isShooterAtIntake() {
    //return shooterPosition.shooterAtIntake;
    return shooterPositionVelocity.shooterAtIntake;
  }

  public boolean doWeHaveNote() {
    return(intake.getIntakeSensor() || delivery.getDeliveryBottomSensor() || delivery.getDeliveryTopSensor() || delivery.getTrapSensor());
  }
}
