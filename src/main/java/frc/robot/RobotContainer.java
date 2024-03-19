// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LED.LEDRunnable;
import frc.robot.commands.auto.AutoStartDelivery;
import frc.robot.commands.auto.AutoStartDeliveryTemp;
import frc.robot.commands.auto.AutoStartDeliveryToSensor;
import frc.robot.commands.auto.AutoStartIntake;
import frc.robot.commands.climber.SetClimbSpeed;
import frc.robot.commands.delivery.DeliveryDefault;
import frc.robot.commands.delivery.DeliveryDefaultNowWithLasers;
import frc.robot.commands.delivery.DeliveryServoDefault;
import frc.robot.commands.delivery.DeliveryServoOverride;
import frc.robot.commands.delivery.SetDeliverySpeed;
import frc.robot.commands.intake.IntakeTripped;
import frc.robot.commands.intake.SetIntakeVelocity;
import frc.robot.commands.intake.SetMotorSpeed;
import frc.robot.commands.shooter.HalfCourt;
import frc.robot.commands.shooter.PoopShoot;
import frc.robot.commands.shooter.SetMotorVelocityBySide;
import frc.robot.commands.shooterPosition.SetShooterPosByDistance;
import frc.robot.commands.shooterPosition.SetShooterPosByVision;
import frc.robot.commands.shooterPosition.SetShooterPosPot;
import frc.robot.commands.swerve.OpPOVLeftDriveAtAngle;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.generated.TunerConstantsPracticeWithKraken;
import frc.robot.nerdyfiles.leds.LED;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.ClimberPosition;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DeliveryServo;
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
  private final DeliveryServo servo = new DeliveryServo();
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
    
    //driverJoystick.back().whileTrue(new InstantCommand(() -> setMaxSpeed(Constants.Swerve.driveScale))).onFalse(new InstantCommand(() -> setMaxSpeed(1)));
    driverJoystick.povLeft().onTrue(new InstantCommand(() -> drivetrain.setRotationAngle(90)));
    driverJoystick.povUp().onTrue(new InstantCommand(() -> drivetrain.setNoteDetection(true)));
    driverJoystick.povUp().onFalse(new InstantCommand(() -> drivetrain.setNoteDetection(false)));

    driverJoystick.povRight().whileTrue(Commands.sequence(
        new InstantCommand(() -> drivetrain.setRotationAngle(getAmpRotationAngle())),
        new InstantCommand(() -> drivetrain.setDriveAtAngleTrue()),
        Commands.parallel(
          drivetrain.followPathCommand(goToAmp()),
          new InstantCommand(() -> shooterPot.setSetpoint(Constants.ShooterPosPot.SHOOTERPOT_AT_AMP)),
          new SetMotorVelocityBySide(shooter, () -> true, () -> false)
        )
      ));

    // driverJoystick.a().whileTrue((new InstantCommand(() -> drivetrain.setPointAtSpeaker(true))
    //   .alongWith(new SetShooterPosByVision(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX(), () -> delivery.getDeliveryTopSensor()))));
    // driverJoystick.a().onFalse(new InstantCommand(() -> drivetrain.setPointAtSpeaker(false)));
    driverJoystick.leftStick().whileTrue(new SetShooterPosByVision(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX(), () -> delivery.getDeliveryTopSensor()));
    // driverJoystick.b().onTrue(new InstantCommand(() -> delivery.engageNoteStop()));
    // driverJoystick.b().onFalse(new InstantCommand(() -> delivery.disengageNoteStop()));
    //driverJoystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverJoystick.y().toggleOnTrue(new InstantCommand(() -> drivetrain.enableLockdown()));
    driverJoystick.leftBumper().onTrue(new InstantCommand(() -> drivetrain.setDriveAtAngleTrue()));
    driverJoystick.leftBumper().onFalse(new InstantCommand(() -> drivetrain.setDriveAtAngleFalse()));
    driverJoystick.rightBumper().onTrue(new InstantCommand(() -> drivetrain.setAngleToZero()));
    driverJoystick.rightBumper().onFalse(new InstantCommand(() -> drivetrain.setDriveAtAngleFalse()));

    driverJoystick.leftTrigger().whileTrue(Commands.parallel(new HalfCourt(shooter, () -> drivetrain.getState().Pose.getY()),
      new InstantCommand(() -> drivetrain.setDriveAtAngleFalse()),
      new ConditionalCommand(
        new InstantCommand(() -> shooterPot.setSetpoint(Constants.ShooterPosPot.SHOOTERPOT_HALF_COURT)),
        new InstantCommand(() -> shooterPot.setSetpoint(Constants.ShooterPosPot.SHOOTERPOT_HALF_CHAIN_COURT)),
        () -> (drivetrain.getState().Pose.getY() >= Constants.FieldElements.cartman && drivetrain.getState().Pose.getY() <= Constants.FieldElements.longwood)
      ),
      // new DelayedDelivery(delivery, Constants.Delivery.DELIVERY_FORWARD_SPEED, () -> shooter.getShooterUpToSpeed()),
      new ConditionalCommand(
      new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.HALF_COURT_BLUE)), 
      new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.HALF_COURT_RED)),
      () -> isAllianceColorBlue()),
      new InstantCommand(() -> drivetrain.setPointAtCartesianVectorOfTheSlopeBetweenTheStageAndTheAmp(true))
    ));
    driverJoystick.leftTrigger().onFalse(Commands.sequence(
      new ConditionalCommand(
        new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_BLUE)), 
        new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_RED)),
        () -> isAllianceColorBlue()),
      new InstantCommand(() -> drivetrain.setCartesianVectorFalse()),
      new InstantCommand(() -> drivetrain.setDriveAtAngleTrue())
    ));
    driverJoystick.rightTrigger().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_FORWARD_SPEED, () -> shooter.getShooterUpToSpeed()));
    driverJoystick.start().onTrue(new InstantCommand(() -> drivetrain.setEndGame(true)));
    driverJoystick.start().onFalse(new InstantCommand(() -> drivetrain.setEndGame(false)));
    //driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));
    
    // reset the field-centric heading on left bumper press
    // driverJoystick.rightBumper().whileTrue(new VisionRotate(drivetrain, driverJoystick, "limelight-orange"));
    
    //*************Operator Control ******************/
    operatorJoystick.rightBumper().whileTrue(new SetIntakeVelocity(intake, () -> getDrivetrainVelocityX(), () -> isShooterAtIntake(), () -> doWeHaveNote(), () -> operatorStation.isBlackSwitchOn(), () -> delivery.getDeliveryBottomSensor(), () -> delivery.getDeliveryTopSensor()));
    operatorJoystick.leftBumper().whileTrue(new SetMotorSpeed(intake, -40, () -> doWeHaveNote()));
    
    operatorJoystick.leftTrigger().whileTrue(new InstantCommand(() ->servo.disengageNoteStop()).andThen(new PoopShoot(shooter)));
    operatorJoystick.rightTrigger().whileTrue(new SetMotorVelocityBySide(shooter, () -> operatorJoystick.x().getAsBoolean(), () -> operatorStation.isYellowSwitchOn()));
    
    operatorJoystick.a().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(8.1))); //Subwoofer Shot
    operatorJoystick.b().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP))); //Also Chain Shot
    operatorJoystick.y().onTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(12.6))); //Manual Amp Zone Shot
    
    operatorJoystick.x().whileTrue(new InstantCommand(() -> drivetrain.setRotationAngle(getAmpRotationAngle()))
    .andThen(new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTERPOT_AT_AMP)))
    .andThen(new SetMotorVelocityBySide(shooter, () -> true, () -> false)));
    
    
    //TODO: operatorJoystick.back().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED, () -> true).withTimeout(0.05));
    operatorJoystick.back().whileTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(6.1))
      .andThen(new SetMotorVelocityBySide(shooter, () -> false, () -> true)));//Yellow Switch = Trap Mode Shooter
    operatorJoystick.start().whileTrue(new SetShooterPosPot(shooterPot, () -> operatorJoystick.povUp().getAsBoolean(), () -> operatorJoystick.povDown().getAsBoolean()));
    
    // operatorJoystick.povLeft().onTrue(new ConditionalCommand(
      //   new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_BLUE)), 
      //   new InstantCommand(() -> drivetrain.setRotationAngle(Constants.Swerve.ROBOT_AT_INTAKE_RED)),
      //   () -> isAllianceColorBlue()));
      
      operatorJoystick.povLeft().onTrue(new OpPOVLeftDriveAtAngle(drivetrain, () -> isAllianceColorBlue()));
      operatorJoystick.povRight().whileTrue(new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_FORWARD_SPEED, () -> shooter.getShooterUpToSpeed()));
      
      
      
      
      //operatorJoystick.back().whileTrue(new SetClimbSpeed(climb, () -> Utilities.deadband(operatorJoystick.getRightY(), 0.1)));
    //*************Operator Station *****************/
    //operatorStation.blueButton.onTrue(new InstantCommand(() -> climb.setClimberPosition(-65)));
    operatorStation.blueButton.whileTrue(new SetClimbSpeed(climb, () -> -0.6));
    operatorStation.whiteButton.whileTrue(new SetClimbSpeed(climb, () -> -0.6));
    operatorStation.yellowButton.whileTrue(new SetClimbSpeed(climb, () -> 0.6));
    operatorStation.blackButton.whileTrue(new InstantCommand(() -> drivetrain.setRotationAngle(getAmpRotationAngle()))
    .andThen(new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTERPOT_AT_AMP)))
    .andThen(new SetMotorVelocityBySide(shooter, () -> true, () -> false)));
    //Operator Station Black Switch = Intake Override
    operatorStation.yellowSwitch.whileTrue(new InstantCommand(() -> shooterPot.setShooterPositionPoint(6.1))
      .andThen(new SetMotorVelocityBySide(shooter, () -> false, () -> true)));//Yellow Switch = Trap Mode Shooter
    operatorStation.blueSwitch.whileTrue(new SetShooterPosPot(shooterPot, () -> operatorJoystick.povUp().getAsBoolean(), () -> operatorJoystick.povDown().getAsBoolean()));//Shooter Pos Kill Switch
    operatorStation.clearSwitch.whileTrue(new DeliveryServoOverride(servo));
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
    

    NamedCommands.registerCommand("AutoStartIntake", new AutoStartIntake(intake, 70));
    NamedCommands.registerCommand("AutoStartDelivery", new AutoStartDelivery(delivery, () -> shooter.getShooterUpToSpeed()).withTimeout(2));
    NamedCommands.registerCommand("ShooterPositionPickup", new InstantCommand(() -> shooterPot.setShooterPositionPoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP))); //5.15, 8.1, 9.95
    NamedCommands.registerCommand("ShooterPosSpeakerSide", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8)).withTimeout(1));

    //The HOT Team ally auto
    NamedCommands.registerCommand("ShooterPos5-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(7.85)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos5-N1", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos5-N2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.5)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos5-N6", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.6)).withTimeout(1));

    //Manual Shooter Pos for Blue-SpeakerRight-C0-C3-C8-C7-C2 Updated
    NamedCommands.registerCommand("ShooterPos7-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos7-N3", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.3)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos7-N8", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.9)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos7-N7", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.7)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos7-N2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10.4)).withTimeout(1));

    NamedCommands.registerCommand("ShooterPos7-N7-2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos7-N6", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.6)).withTimeout(1));

    //Manual Shooter Pos for Red-SpeakerRight-C0-C3-C8-C7-C2 Updated
    NamedCommands.registerCommand("ShooterPos17-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos17-N3", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.2)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos17-N8", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos17-N7", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.6)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos17-N2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10.3)).withTimeout(1));

    NamedCommands.registerCommand("ShooterPos17-N7-2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos17-N6", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.6)).withTimeout(1));

    //Manual Shooter Pos for Blue-SpeakerCenter-C0-C1-C2-C3
    NamedCommands.registerCommand("ShooterPos6-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(7.85)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos6-N1", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos6-N2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.3)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos6-N3", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10)).withTimeout(1));

    NamedCommands.registerCommand("ShooterPos16-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(7.85)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos16-N1", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos16-N2", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13.5)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos16-N3", new InstantCommand(() -> shooterPot.setShooterPositionPoint(10)).withTimeout(1));

    //Manual Shooter Pos for Blue-SpeakerCenter-C0-C1-C2-C3
    NamedCommands.registerCommand("ShooterPos8-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos8-N4", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos8-N5", new InstantCommand(() -> shooterPot.setShooterPositionPoint(15)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos8-N6", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13)).withTimeout(1));

    
    NamedCommands.registerCommand("ShooterPos18-N0", new InstantCommand(() -> shooterPot.setShooterPositionPoint(8)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos18-N4", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos18-N5", new InstantCommand(() -> shooterPot.setShooterPositionPoint(15)).withTimeout(1));
    NamedCommands.registerCommand("ShooterPos18-N6", new InstantCommand(() -> shooterPot.setShooterPositionPoint(13)).withTimeout(1));

    NamedCommands.registerCommand("ShooterPositionByDistance", new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX(), () -> doWeHaveNote()).withTimeout(1));
    NamedCommands.registerCommand("ShooterPositionByDistanceDrive", new SetShooterPosByDistance(shooterPot, () -> drivetrain.getPose(), () -> getAllianceColor(), () -> getDrivetrainVelocityX(), () -> delivery.getDeliveryTopSensor()));
    NamedCommands.registerCommand("AutoStartDeliveryToSensor", new AutoStartDeliveryToSensor(delivery).withTimeout(6));
    NamedCommands.registerCommand("AutoStartDeliveryToSensorNoTimeout", new AutoStartDeliveryToSensor(delivery));
    NamedCommands.registerCommand("AutoStartDeliveryToSensorTinyTimeout", new AutoStartDeliveryToSensor(delivery).withTimeout(0.2));
    NamedCommands.registerCommand("AutoStartDeliveryTemp", new AutoStartDeliveryTemp(delivery).withTimeout(0.2));
    NamedCommands.registerCommand("AutoStartDeliveryTempLong", new AutoStartDeliveryTemp(delivery).withTimeout(2));
    //NamedCommands.registerCommand("AutoStartDeliveryBackTemp", new SetDeliverySpeed(delivery, Constants.Delivery.DELIVERY_REVERSE_SPEED).withTimeout(0.1));
    NamedCommands.registerCommand("StartShooter", new SetMotorVelocityBySide(shooter, () -> false, () -> false));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooter.setShooterDutyCycleZero()));
    NamedCommands.registerCommand("UseLimelight", new InstantCommand(() -> useLimelight()));
    NamedCommands.registerCommand("DontUseLimelight", new InstantCommand(() -> dontUseLimelight()));

    
    /* 
     * Run at the beginning of auto to enable the ability to change using vision for localization during auto. If not used, the default vision is to turn it on and off based on odometry.
     * The default value after running this command is for vision to be on through all of auto.
     * Normal usage would be to run at the beginning of the auto to enable vision for the entirety of auto, or use with AutoDontUseLimeLight to turn it off. 
     */
    NamedCommands.registerCommand("AutoModLimelight", new InstantCommand(() -> autoModLimelight()));
    // Used with autoModLimelight.  Can be modified at any time during auto.
    NamedCommands.registerCommand("AutoUseLimelight", new InstantCommand(() -> autoUseLimelight()));
    NamedCommands.registerCommand("AutoDontUseLimelight", new InstantCommand(() -> autoDontUseLimelight()));

    
    NamedCommands.registerCommand("DoWeHaveNote", new IntakeTripped(() -> doWeHaveNote()));
    NamedCommands.registerCommand("EngageDeliverServo", new InstantCommand(() -> servo.engageNoteStop()));
    NamedCommands.registerCommand("DisengageDeliverServo", new InstantCommand(() -> servo.disengageNoteStop()));

    //TODO: Bring back autos
    autonChooser = AutoBuilder.buildAutoChooser();
    //autonChooser = null;
    configureBindings();
    autoTab.add("Auton Chooser", autonChooser)
    .withSize(3,1)
    .withPosition(3, 0);

  }

  public void instantiateSubsystemsTeleop() {
    delivery.setDefaultCommand(new DeliveryDefault(delivery, () -> intake.getIntakeSensor(), () -> shooter.getShooterVelocity()));
    servo.setDefaultCommand(new DeliveryServoDefault(servo, ()-> delivery.getDeliveryTopSensor()));
    //intake.setDefaultCommand(new SetIntakeVelocity(intake, () -> getDrivetrainVelocityX(), () -> isShooterAtIntake(), () -> doWeHaveNote(), () -> operatorStation.isBlackSwitchOn(), () -> delivery.getDeliveryBottomSensor(), () -> delivery.getDeliveryTopSensor())); 
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autonChooser.getSelected();
    //return new InstantCommand();
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

  public boolean isAllianceColorBlue() {
    if (allianceColor == "blue") {
      return true;
    } else {
      return false;
    }
  }

  public double getEndgameRotationAngleLeft() {
    return 120;
  }

  public double getEndgameRotationAngleRight() {
    return -120;
  }

  public void useLimelight() {
    drivetrain.useLimelight = true;
  }

  public void dontUseLimelight() {
    drivetrain.useLimelight = false;
  }

  public void autoUseLimelight() {
    drivetrain.autoUseLimelight = true;
  }

  public void autoDontUseLimelight() {
    drivetrain.autoUseLimelight = false;
  }
  /**
     * Run at the beginning of auto to enable the ability to change using vision for localization during auto. If not used, the default vision is to turn it on and off based on odometry.
     * The default value after running this command is for vision to be on through all of auto.
     * Normal usage would be to run at the beginning of the auto to enable vision for the entirety of auto, or use with AutoDontUseLimeLight to turn it off. 
   */
  public void autoModLimelight() {
    drivetrain.autoModLimelight = true;
  }

  public PathPlannerPath goToAmp() {
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    Translation2d ampLocation;
    if (allianceColor == "red") {
      ampLocation = Constants.FieldElements.redAmpRobotLocation;
    } else {
      ampLocation = Constants.FieldElements.blueAmpRobotLocation;
    }
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(drivetrain.getState().Pose.getX(),drivetrain.getState().Pose.getY(),drivetrain.getState().Pose.getRotation()),
            new Pose2d(ampLocation.getX(), ampLocation.getY(), Rotation2d.fromDegrees(getAmpRotationAngle()))
    );

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 1.5, 10 * Math.PI, 16 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(getAmpRotationAngle())) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return path;
  }

  public boolean doWeHaveNote() {
    return(intake.getIntakeSensor() || delivery.getDeliveryBottomSensor() || delivery.getDeliveryTopSensor());
  }

}
