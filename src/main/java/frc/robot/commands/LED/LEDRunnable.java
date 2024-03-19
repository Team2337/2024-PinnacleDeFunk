package frc.robot.commands.LED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.nerdyfiles.leds.LED;

public class LEDRunnable extends Command {
  private final LED led;
  private Supplier<Boolean> intakeSensor, deliveryTopSensor, upToSpeed;
  

  public LEDRunnable(LED led, Supplier<Boolean> intakeSensor, Supplier<Boolean> deliveryTopSensor, Supplier<Boolean> upToSpeed) {
    this.led = led;
    this.intakeSensor = intakeSensor;
    this.deliveryTopSensor = deliveryTopSensor;
    this.upToSpeed = upToSpeed;

    addRequirements(led);
  }
  @Override
  public void execute() {
    if (DriverStation.isTeleopEnabled() && upToSpeed.get()) {
        led.setColor(Color.kGreen);
    } else if (DriverStation.isTeleopEnabled() && deliveryTopSensor.get()) {
        led.setColor(Color.kBlue);
    }  else if (DriverStation.isTeleopEnabled() && intakeSensor.get()) {
        led.setColor(Color.kRed);
    } else if (DriverStation.isTeleopEnabled()) {
        led.setColor(Color.kBlack);
    }
    
    if (DriverStation.isDisabled()) {
        led.setColor(Color.kRed);
       // led.setShooterColors(Color.kBlack);
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