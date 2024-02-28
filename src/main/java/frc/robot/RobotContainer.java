// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LED.LEDRunnable;
import frc.robot.commands.auto.AutoStartDelivery;
import frc.robot.commands.auto.AutoStartDeliveryTemp;
import frc.robot.commands.auto.AutoStartDeliveryToSensor;
import frc.robot.commands.auto.AutoStartIntake;
import frc.robot.commands.climber.SetClimbSpeed;
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
import frc.robot.generated.TunerConstantsComp;
import frc.robot.generated.TunerConstantsPracticeWithKraken;
import frc.robot.nerdyfiles.leds.LED;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.ClimberPosition;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPosPot;

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

  private final ClimberPosition climb = new ClimberPosition(operatorJoystick);
  private final Delivery delivery = new Delivery();
  //private final Elevator elevator = new Elevator(operatorJoystick);
  private final Intake intake = new Intake();
  private final LED led = new LED();
  private final Shooter shooter = new Shooter(() -> getAllianceColor(), () -> getPoseY());
  private final ShooterPosPot shooterPot = new ShooterPosPot(operatorJoystick, () -> doWeHaveNote());
  private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);
  private final SendableChooser<Command> autonChooser;

  public String allianceColor = null;
  
  private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  // private ComplexWidget autoChooser = autoTab
  //   .add("Auto Chooser", autonChooser)
  //   .withSize(3,2);
  
  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick, () -> getDrivetrainVelocityY(), () -> getAllianceColor()));   
    led.setDefaultCommand(new LEDRunnable(led, ()-> intake.getIntakeSensor(), () -> delivery.getDeliveryTopSensor(), () -> shooter.getShooterUpToSpeed()).ignoringDisable(true));

    driverJoystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(Constants.Swerve.driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    driverJoystick.povLeft().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(90)));
    driverJoystick.povRight().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(-90)));
    driverJoystick.povUp().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(1)));
    driverJoystick.povDown().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(179)));

    driverJoystick.a().whileTrue(new InstantCommand(() -> drivetrain.setPointAtSpeaker(true))
      .alongWith(new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX())));
    driverJoystick.a().onFalse(new InstantCommand(() -> drivetrain.setPointAtSpeaker(false)));
    //driverJoystick.x().toggleOnFalse(new InstantCommand(() -> drivetrain.setToDriveAtAngle()));
    driverJoystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverJoystick.y().toggleOnTrue(new InstantCommand(() -> drivetrain.enableLockdown()));
    driverJoystick.leftBumper().toggleOnTrue(new InstantCommand(() -> drivetrain.setToDriveAtAngle()));
    driverJoystick.rightBumper().toggleOnTrue(new InstantCommand(() -> drivetrain.setAngleToZero()));
    driverJoystick.rightBumper().toggleOnFalse(new InstantCommand(() -> drivetrain.setToDriveAtAngle()));
    driverJoystick.rightTrigger().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_FORWARD_SPEED, () -> shooter.getShooterUpToSpeed()));
    //driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));
    
    // reset the field-centric heading on left bumper press
    // driverJoystick.rightBumper().whileTrue(new VisionRotate(drivetrain, driverJoystick, "limelight-orange"));
    
    //*************Operator Control ******************/
    operatorJoystick.rightBumper().whileTrue(new SetMotorSpeed(intake, 40, () -> doWeHaveNote()));
    operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -40, () -> doWeHaveNote()));

    // operatorJoystick.a().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(6))); 
    // operatorJoystick.b().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP))); 
    // operatorJoystick.x().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(5.3))); 

    // operatorJoystick.b().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED).withTimeout(0.2)
    //   .andThen(new SetMotorVelocityBySide(shooter)));
    operatorJoystick.y().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED, () -> true).withTimeout(0.05));
    operatorJoystick.rightTrigger().whileTrue(new SetMotorVelocityBySide(shooter, () -> operatorStation.blackButton.getAsBoolean()));
    operatorJoystick.start().whileTrue(new SetShooterPosPot(shooterPot, () -> operatorJoystick.povUp().getAsBoolean(), () -> operatorJoystick.povDown().getAsBoolean()));

    operatorJoystick.a().onTrue(new InstantCommand(() -> climb.setClimberPosition(-20)));
    operatorJoystick.b().onTrue(new InstantCommand(() -> climb.setClimberPosition(0)));
    operatorJoystick.back().whileTrue(new SetClimbSpeed(climb, () -> Utilities.deadband(operatorJoystick.getRightY(), 0.1)));

    
    //*************Operator Station *****************/
    operatorStation.blackSwitch.onTrue(new InstantCommand(() -> drivetrain.setEndGame(true)));
    operatorStation.blackSwitch.onFalse(new InstantCommand(() -> drivetrain.setEndGame(false)));
    operatorStation.whiteButton.whileTrue(new SetShooterPosPot(shooterPot, () -> operatorJoystick.povUp().getAsBoolean(), () -> operatorJoystick.povDown().getAsBoolean()));
    operatorStation.blackButton.onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(getAmpRotationAngle()))
      .andThen(new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTERPOT_AT_AMP))));
    operatorStation.redLeftSwitch.onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(getEndgameRotationAngleLeft())));
    operatorStation.redRightSwitch.onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(getEndgameRotationAngleRight())));
    
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
        drivetrain = TunerConstantsComp.DriveTrain;

       
        break;
    }
    

    NamedCommands.registerCommand("AutoStartIntake", new AutoStartIntake(intake, 40));
    NamedCommands.registerCommand("AutoStartDelivery", new AutoStartDelivery(delivery, () -> shooter.getShooterUpToSpeed()).withTimeout(2));
    NamedCommands.registerCommand("ShooterPositionPickup", new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP))); //5.15, 8.1, 9.95
    NamedCommands.registerCommand("ShooterPositionClose", new InstantCommand(() -> shooterPot.setShooterPositionPoint(6)));
    NamedCommands.registerCommand("ShooterPositionFaryFar", new InstantCommand(() -> shooterPot.setShooterPositionPoint(9.95)));
    NamedCommands.registerCommand("ShooterPositionByDistance", new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX()).withTimeout(2));
    NamedCommands.registerCommand("AutoStartDeliveryToSensor", new AutoStartDeliveryToSensor(delivery));
    NamedCommands.registerCommand("AutoStartDeliveryTemp", new AutoStartDeliveryTemp(delivery).withTimeout(0.5));
    //NamedCommands.registerCommand("AutoStartDeliveryBackTemp", new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED).withTimeout(0.1));
    NamedCommands.registerCommand("StartShooter", new SetMotorVelocityBySide(shooter, () -> false));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooter.setShooterDutyCycleZero()));
    autonChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    autoTab.add("Auton Chooser", autonChooser)
    .withSize(3,1)
    .withPosition(3, 0);
    // SmartDashboard.putData("Auto Chooser", autonChooser);

  }

  public void instantiateSubsystemsTeleop() {
    delivery.setDefaultCommand(new DeliveryDefault(delivery, () -> intake.getIntakeSensor()));
    intake.setDefaultCommand(new SetIntakeVelocity(intake, () -> getDrivetrainVelocityX(), () -> isShooterAtIntake(), () -> doWeHaveNote(), () -> operatorStation.isBlackSwitchOn(), () -> delivery.getDeliveryBottomSensor())); 
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autonChooser.getSelected();
  }

  public double getDrivetrainVelocityX() {
    return logger.getXVelocity();
  }

  public double getDrivetrainVelocityY() {
    return logger.getYVelocity();
  }

  public boolean isShooterAtTrap() {
    return shooterPot.shooterAtTrap;
  }
  
  public boolean isShooterAtIntake() {
    return shooterPot.shooterAtIntake;
  }

  public String getAllianceColor () {
    return allianceColor;
  }

  public double getPoseY () {
    return drivetrain.getState().Pose.getY();
  }

  public double getAmpRotationAngle() {
    if (allianceColor == "red") {
      return 90;
    } else {
      return -90;
    }
  }

  public double getEndgameRotationAngleLeft() {
    return -30;
  }

  public double getEndgameRotationAngleRight() {
    return 30;
  }

  public boolean doWeHaveNote() {
    return(intake.getIntakeSensor() || delivery.getDeliveryBottomSensor() || delivery.getDeliveryTopSensor());
  }

}
