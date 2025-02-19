package frc.robot.nerdyfiles.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class Utilities {
  /**
   * Convert a Rotation2d to a rotation value between (-180, 180).
   *
   * @return A Rotation2d from (-180, 180) degrees.
   */
  public static Rotation2d convertRotationToRelativeRotation(Rotation2d rotation) {
    double rotationDegrees = rotation.getDegrees() % 360;
    if (rotationDegrees > 180) {
      rotationDegrees -= 360;
    } else if (rotationDegrees < -180) {
      rotationDegrees += 360;
    }
    return Rotation2d.fromDegrees(rotationDegrees);
  }

  /**
   * Checks to see if the absolute value of the input is less than the deadband
   * @param input - Value in which the deadband will be applied (0 < input < 1)
   * @param deadband - Deadband to set on the input (double)
   * @return - input double value adjusted for the deadband
   */
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) return 0;
    return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
  }

  public static double ctreDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) return 0;
    return Math.copySign(Math.abs(input) - ((1 - input) / (1 - deadband)), input);
  }

  /**
   * Squares the input value
   * @param value - double value wanting to be squared
   * @return - squared input double value
   */
  public static double squareValues(double value) {
    return Math.copySign(Math.pow(value, 2), value);
  }

  /**
   * Deadband + square joystick axis values.
   */
  public static double deadbandAndSquare(double value) {
    return deadbandAndSquare(value, 0.05);
  }

  /**
   * Deadband + square joystick axis values.
   */
  public static double deadbandAndSquare(double value, double deadband) {
    // Deadband
    value = deadband(value, deadband);
    // Square the axis
    return Math.copySign(value * value, value);
  }

   /**
   * To be used to reduce drive inputs for fine control
   * 
   * Checks to see if the absolute value of the input is less than the deadband
   * @param input - Value in which the deadband will be applied (0 < input < 1)
   * @param deadband - Deadband to set on the input (double)
   * @param scale - Scale the input outside the deadband to reduce input (double) 
   * A deadband of .1 and a scale factor of 2 leads to a scaling of rougly 50% of max output
   * A deadband of .1 and a scale factor or 3 leads to a scaling of rougly 30% of max output
   * A deadband of .1 and a scale factor or 4 leads to a scaling of rougly 23% of max output
   * A deadband of .1 and a scale factor or 5 leads to a scaling of rougly 18% of max output
   * @return - input double value adjusted for the deadband
   */

   public static double deadbandAndScale(double input, double deadband, double scale) {
    if (Math.abs(input) < deadband) return 0;
    return Math.copySign((Math.abs(input) - deadband) / (scale - deadband), input);
  }

  /**
   * Calculates derivative based on the current and previous sensor inputs
   * @param error - double value reading the current sensor error (current - target)
   * @param lastError - double value reading the previous sensor error (current - target)
   * @param dt - Change in time from when the previous sensor error was gotten to the time the current was gotten
   * @return - returns derivative double value to add to the speed of the motor
   */
  public static double calculateDerivative(double error, double lastError, double dt) {
    if (Double.isFinite(lastError)) {
      return (error - lastError) / dt;
    } else {
      return 0;
    }
  }

  /**
   * Determines whether or not the given value is within a certain amount of a target
   *
   * @param target The desired value
   * @param current The current value
   * @param tolerance A range that the given value can be within the target value before returning true
   * @return Whether or not the current value is within a tolerance of the target
   */
  public static boolean withinTolerance(double target, double current, double tolerance) {
    return Math.abs(target - current) <= tolerance;
  }

  /**
   * Converts a temperature in Celsius to Fahrenheit
   *
   * @param celsius The degrees in Celsius
   * @return The degrees in Fahrenheit
   */
  public static double convertCelsiusToFahrenheit(double celsius) {
    return (celsius * (9.0/5.0)) + 32.0;
  }

  /**
   * Convert Q2.14 -> double
   *
   * http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html#a2a964e72cdd29bd9ca52b24d7d02e326
   * https://en.wikipedia.org/wiki/Q_(number_format)#Q_to_float
   */
  public static double q(short val) {
    return val * Math.pow(2, -14);
  }

  /**
   * Scales any input to -1 to 1 to mimic a joystick input
   * @param input
   * @param oldMax
   * @param oldMin
   * @return
   */
  public static double scaleVisionToOne(double input, double oldMax, double oldMin) {
    double newMin = -1;
    double newMax = 1;

    return (((input - oldMin)*(newMax - newMin))/(oldMax - oldMin))+newMin;
  }

  /**
   * Scales output from limelight to -1 to 1 to mimic a joystick input
   * Limelight output is in range of -29.8 to 29.8 degrees
   * @param input
   * @param oldMax
   * @param oldMin
   * @return
   */
  public static double scaleVisionToOne(double input) {
    double newMin = -1;
    double newMax = 1;
    double oldMin = -29.8;
    double oldMax = 29.8;

    return (((input - oldMin)*(newMax - newMin))/(oldMax - oldMin))+newMin;
  }

  /**
   * Scales any input to any output
   * @param input
   * @param oldMin
   * @param oldMax
   * @param newMin
   * @param newMax
   * @return
   */
  public static double scaleAnyToAny(double input, double oldMin, double oldMax, double newMin, double newMax) {
    return (((input - oldMin)*(newMax - newMin))/(oldMax - oldMin))+newMin;
  }
}