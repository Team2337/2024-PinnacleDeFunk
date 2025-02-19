package frc.robot.commands.LED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.nerdyfiles.leds.LED;
import frc.robot.nerdyfiles.vision.LimelightHelpers;

public class LEDRunnable extends Command {
  private final LED led;
  private Supplier<Boolean> intakeSensor, deliveryTopSensor, deliveryBottomSensor, upToSpeed, intakeSpinning;
  

  public LEDRunnable(LED led, Supplier<Boolean> intakeSensor, Supplier<Boolean> deliveryTopSensor, Supplier<Boolean> deliveryBottomSensor, Supplier<Boolean> upToSpeed, Supplier<Boolean> intakeSpinning) {
    this.led = led;
    this.intakeSensor = intakeSensor;
    this.deliveryTopSensor = deliveryTopSensor;
    this.deliveryBottomSensor = deliveryBottomSensor;
    this.upToSpeed = upToSpeed;
    this.intakeSpinning = intakeSpinning;
    addRequirements(led);
  }
  @Override
  public void execute() {
    if (DriverStation.isTeleopEnabled() && upToSpeed.get()) {
        led.setColor(Color.kGreen);
    } else if (DriverStation.isTeleopEnabled() && deliveryTopSensor.get()) {
        led.setColor(Color.kRed);
    }  else if (DriverStation.isTeleopEnabled() && deliveryBottomSensor.get()) {
        led.setColor(Color.kBlue);
    }  else if (DriverStation.isTeleopEnabled() && intakeSensor.get()) {
        led.setLowerUprightColors(Color.kBlue);
    } else if (DriverStation.isTeleopEnabled() && intakeSpinning.get()) {
        led.setIntakeColor(Color.kRed);
    } else if (DriverStation.isTeleopEnabled()) {
        led.setColor(Color.kBlack);
    }
    
    if (DriverStation.isAutonomous()) {
      double time = DriverStation.getMatchTime();
      SmartDashboard.putNumber("Match Time", time);
    }

    if (DriverStation.isAutonomousEnabled()) {
      double time = DriverStation.getMatchTime();
      //led.setAutoColor(Color.kRed, time);
        led.setColor(Color.kBlack); 
    }
    
    if (DriverStation.isDisabled()) {
        led.setColor(Color.kRed);
    } 

    // if (DriverStation.isTeleop() && hasGamepiece.get() == true) {
    //   led.setLeftColor(Color.kRed);
    //   led.setRightColor(Color.kRed);
    // }
    
    // if (robotContainer.getYellowSwitchStatus() && robotContainer.getGyroscopeRoll() < Constants.CLIMBER_ROLL) {
    //   led.setColor(Color.kPurple);
    // } else if (robotContainer.isShooterUpToLEDSpeed() && robotContainer.hasActiveTarget()) {
    //   led.setColor(Color.kRed, robotContainer.getTx());
    // } else if (robotContainer.isShooterUpToLEDSpeed()) {
    //   led.setColor(Color.kBlue);
    // } else if (robotContainer.getOperatorStartStatus() || robotContainer.getOperatorBackStatus()) {
    //   led.setColorMiddle();
    // } else if (robotContainer.hasActiveTarget()) {
    //   led.setColor(Color.kYellow, robotContainer.getTx());
    // } else if (robotContainer.getOperatorRightTriggerStatus() && DriverStation.isTeleop()) {
    //   led.setColorMiddle();
    // } else {
    //   led.setOff();
    // }
  }
}