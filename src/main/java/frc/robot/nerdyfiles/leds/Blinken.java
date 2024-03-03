package frc.robot.nerdyfiles.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Blinken extends SubsystemBase {

	private Spark blinken = new Spark(1);
	private static int LED_LENGTH = 273;
	private double setColor = 0;

	/**
	 * Controls the LEDs on the Robot 
	 * 
	 * @param pwm The PWM port that the blinkin is plugged into
	 */
	public Blinken() {

	}

	/**
	 * Sets the color of the LEDs
	 * 
	 * @param color A Color object reflecting the color you want to use on the LEDs.  i.e.  kRed, kBlue, kSeashell
	 */
	public void setColor(double color) {
		if (color != setColor) {
			blinken.set(color);
		}
	}

  
	public static void setLeftOff() {

  }

  
	public static void setRightOff() {

  }

	public static void setOff() {

  }

  public final static class BlinkenColor {
	public static double kGreen = 0.76;
	public static double kBlue = 0.87;
	public static double kRed = 0.61;
	public static double kYellow = 0.69;
	public static double kBlack = 0.99;
  }
} 