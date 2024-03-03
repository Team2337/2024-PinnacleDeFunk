package frc.robot.commands.LED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.nerdyfiles.leds.Blinken;

public class BlinkenRunnable extends Command {
  private final Blinken blinken;
  private Supplier<Boolean> intakeSensor, deliveryTopSensor, upToSpeed;
  

  public BlinkenRunnable(Blinken blinken, Supplier<Boolean> intakeSensor, Supplier<Boolean> deliveryTopSensor, Supplier<Boolean> upToSpeed) {
    this.blinken = blinken;
    this.intakeSensor = intakeSensor;
    this.deliveryTopSensor = deliveryTopSensor;
    this.upToSpeed = upToSpeed;

    addRequirements(blinken);
  }
  @Override
  public void execute() {
    if (DriverStation.isTeleopEnabled() && upToSpeed.get()) {
        blinken.setColor(Blinken.BlinkenColor.kGreen);
    } else if (DriverStation.isTeleopEnabled() && deliveryTopSensor.get()) {
        blinken.setColor(Blinken.BlinkenColor.kBlue);
    }  else if (DriverStation.isTeleopEnabled() && intakeSensor.get()) {
        blinken.setColor(Blinken.BlinkenColor.kRed);
    } else if (DriverStation.isTeleopEnabled()) {
        blinken.setColor(Blinken.BlinkenColor.kBlack);
    }
    
    if (DriverStation.isDisabled()) {
        blinken.setColor(Blinken.BlinkenColor.kRed);
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