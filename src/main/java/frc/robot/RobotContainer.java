// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoStartDelivery;
import frc.robot.commands.auto.AutoStartDeliveryTemp;
import frc.robot.commands.auto.AutoStartDeliveryToSensor;
import frc.robot.commands.auto.AutoStartIntake;
import frc.robot.commands.delivery.DeliveryDefault;
import frc.robot.commands.delivery.SetDeliverySpeed;
import frc.robot.commands.intake.SetIntakeVelocity;
import frc.robot.commands.intake.SetMotorSpeed;
import frc.robot.commands.shooter.SetMotorVelocityBySide;
import frc.robot.commands.shooterPosition.SetShooterPosByDistance;
import frc.robot.commands.shooterPosition.SetShooterPosPot;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.swerve.VisionRotate;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsPractice;
import frc.robot.generated.TunerConstantsPracticeWithKraken;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPosPot;
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
  private final Shooter shooter = new Shooter(() -> getAllianceColor(), () -> getPoseY());
  private final ShooterPosPot shooterPot = new ShooterPosPot(operatorJoystick);
  private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);
  private final Vision vision = new Vision(this);
  private final SendableChooser<Command> autonChooser;

  public String allianceColor = null;
  
  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick));   

    driverJoystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(Constants.Swerve.driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    driverJoystick.povLeft().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(90)));
    driverJoystick.povRight().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(-90)));
    driverJoystick.povUp().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(1)));
    driverJoystick.povDown().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(179)));

    driverJoystick.a().whileTrue(new InstantCommand(() -> drivetrain.setPointAtSpeaker(true)));
      //.alongWith(new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose())));
    driverJoystick.a().onFalse(new InstantCommand(() -> drivetrain.setPointAtSpeaker(false)));
    //driverJoystick.b().whileTrue(new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose()));
    driverJoystick.x().toggleOnTrue(new InstantCommand(() -> drivetrain.setToDriveAtAngle()));
    driverJoystick.y().toggleOnTrue(new InstantCommand(() -> drivetrain.enableLockdown()));
    driverJoystick.rightTrigger().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_FORWARD_SPEED));
    //driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));
    
    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverJoystick.rightBumper().whileTrue(new VisionRotate(drivetrain, driverJoystick, "limelight-orange"));
    
    //*************Operator Control ******************/
    // operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 0.1, () -> doWeHaveNote()));
    // operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -0.1, () -> doWeHaveNote()));
    operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 40, () -> doWeHaveNote()));
    operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -40, () -> doWeHaveNote()));
    operatorJoystick.a().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(5.15))); 
    // operatorJoystick.b().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED).withTimeout(0.2)
    //   .andThen(new SetMotorVelocityBySide(shooter)));
    operatorJoystick.b().whileTrue(new SetMotorVelocityBySide(shooter));
    operatorJoystick.x().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(8.1)));
    operatorJoystick.y().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(9.95)));
    operatorJoystick.back().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED));
    operatorJoystick.start().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(6.1)));

    //operatorJoystick.povUp().onTrue(new InstantCommand(() -> shooterPosition.setShooterPosition(30)));
    // operatorJoystick.povUp().whileTrue(new InstantCommand(() -> shooterPositionVelocity.setShooterPositionVelocity(5)));
    // operatorJoystick.povDown().whileTrue(new InstantCommand(() -> shooterPositionVelocity.setShooterPositionVelocity(-5)));
    // operatorJoystick.povUp().whileTrue(new SetShooterPositionVelocity(shooterPositionVelocity, 5));
    // operatorJoystick.povDown().whileTrue(new SetShooterPositionVelocity(shooterPositionVelocity, -5));



    // operatorJoystick.b().onTrue(new InstantCommand(() -> climb.setClimberSetpoint(2.06)));
    // operatorJoystick.b().onFalse(new InstantCommand(() -> climb.getSetSetPoint()));
    // operatorJoystick.a().onTrue(new InstantCommand(() -> climb.setClimberSetpoint(10)));
    // operatorJoystick.a().onFalse(new InstantCommand(() -> climb.getSetSetPoint()));
    // operatorJoystick.povUp().whileTrue(new SetClimbSpeed(climb, () -> Utilities.deadband(operatorJoystick.getRightY(), 0.1)));

    
    //*************Operator Station *****************/
    // operatorStation.blackSwitch.onTrue(new InstantCommand(() -> drivetrain.setEndGame(true)));
    // operatorStation.blackSwitch.onFalse(new InstantCommand(() -> drivetrain.setEndGame(false)));
    operatorStation.greenButton.whileTrue(new SetShooterPosPot(shooterPot, () -> operatorJoystick.povUp().getAsBoolean(), () -> operatorJoystick.povDown().getAsBoolean()));
    
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
        drivetrain = TunerConstantsPracticeWithKraken.DriveTrain;
        break;
      case COMPETITION:
      default:
        drivetrain = TunerConstants.DriveTrain;

       
        break;
    }
    

    NamedCommands.registerCommand("AutoStartIntake", new AutoStartIntake(intake, 40));
    //NamedCommands.registerCommand("SetShooterPosition", new InstantCommand(() -> shooterPositionVelocity.setShooterPosition(10), shooterPositionVelocity));
    NamedCommands.registerCommand("AutoStartDelivery", new AutoStartDelivery(delivery, () -> shooter.getShooterUpToSpeed()).withTimeout(2));
    // NamedCommands.registerCommand("AutoSetShooterPositionVelocityUp", new AutoSetShooterPositionVelocity(shooterPositionVelocity, 10, 7).withTimeout(2));
    // NamedCommands.registerCommand("AutoSetShooterPositionVelocityDown", new AutoSetShooterPositionVelocity(shooterPositionVelocity, -10, 0).withTimeout(2));
    NamedCommands.registerCommand("ShooterPositionClose", new InstantCommand(() -> shooterPot.setShooterPositionPoint(5.15))); //5.15, 8.1, 9.95
    NamedCommands.registerCommand("ShooterPositionFar", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8.1)));
    NamedCommands.registerCommand("ShooterPositionFaryFar", new InstantCommand(() -> shooterPot.setShooterPositionPoint(9.95)));
    NamedCommands.registerCommand("ShooterPositionByDistance", new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose()));
    NamedCommands.registerCommand("AutoStartDeliveryToSensor", new AutoStartDeliveryToSensor(delivery));
    NamedCommands.registerCommand("AutoStartDeliveryTemp", new AutoStartDeliveryTemp(delivery).withTimeout(2));
    NamedCommands.registerCommand("StartShooter", new SetMotorVelocityBySide(shooter));
    autonChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autonChooser);
  }

  public void instantiateSubsystemsTeleop() {
    delivery.setDefaultCommand(new DeliveryDefault(delivery));
    //intake.setDefaultCommand(new SetTempIntakeVelocity(intake, () -> getDrivetrainVelocityX()));
    intake.setDefaultCommand(new SetIntakeVelocity(intake, () -> getDrivetrainVelocityX(), () -> isShooterAtIntake(), () -> doWeHaveNote())); 
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
    return shooterPot.shooterAtTrap;
  }
  
  public boolean isShooterAtIntake() {
    //return shooterPosition.shooterAtIntake;
    return shooterPot.shooterAtIntake;
  }

  public String getAllianceColor () {
    return allianceColor;
  }

  public double getPoseY () {
    return drivetrain.getState().Pose.getY();
  }

  public boolean doWeHaveNote() {
    return(intake.getIntakeSensor() || delivery.getDeliveryBottomSensor() || delivery.getDeliveryTopSensor() || delivery.getTrapSensor());
  }
}
