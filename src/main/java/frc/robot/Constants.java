package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to static finalally import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static double STARTING_ANGLE = 25;

  

  private static Constants instance;

  public static final class DashboardLogging {
    public static final boolean AUTO = false;
    public static final boolean CLIMB = true;
    public static final boolean DELIVERY = true;
    public static final boolean DRIVETRAIN = false;
    public static final boolean HEADING = false;
    public static final boolean INTAKE = true;
    public static final boolean PDH = false;
    public static final boolean SHOOTER = true;
    public static final boolean SWERVE = true;
    public static final boolean VISION = true;
  }

  // Driver dashboard
  // 28x13
  public static enum DriverDashboardPositions {
    AUTON_CHOOSER(9, 6, 6, 3),
    STARTING_POS_CHOOSER(9, 0, 6, 3),
    STARTING_ANGLE_CHOOSER(9, 3, 6, 3),
    AUTODRIVE_COMMAND(9, 9, 6, 3),
    GYRO_DEGREES(12, 9, 3, 3),
    ALLIANCE(9, 9, 6, 3),
    SHOULDERLAMPREY(15,0,3,3),
    ELBOWLAMPREY(18,0,3,3),
    INTAKESPINNERLAMPREY(21,0,3,3),
    INTAKESENSOR(15, 6, 3, 3),
    PASTARMPOSITION(15,3,6,3);

    public final int x, y, width, height;

    private DriverDashboardPositions(int x, int y, int w, int h) {
      this.x = x;
      this.y = y;
      this.width = w;
      this.height = h;
    }
  }
  public static final ShuffleboardTab DRIVER_DASHBOARD = Shuffleboard.getTab("DRIVER DASHBOARD");

  // Systems check
  public static enum SystemsCheckPositions {
    // Temperatures (3x4 widgets)
    MODULE0(9, 4),
    MODULE1(12,4),
    MODULE2(15, 4),
    MODULE3(18, 4),
    INTAKE_TEMP(0, 0),
    DELIVERY_TEMP(3, 0),
    L_SHOOTER_TEMP(0, 4),
    R_SHOOTER_TEMP(3, 4),
    L_CLIMBER_TEMP(0, 8),
    R_CLIMBER_TEMP(3, 8),
    // Delivery Sensors (3x3 widgets)
    L_COLOR_SENSOR(7, 0),
    R_COLOR_SENSOR(10, 0),
    CENTERING_SENSOR(13, 0),
    // Other sensors (also 3x3 widgets)
    STRING_POT(7, 3),
    PIXY_CAM(10, 3),
    LIMELIGHT(13, 3),
    ROLL_DEGREES(19, 0);

    public final int x, y;

    private SystemsCheckPositions(int x, int y) {
      this.x = x;
      this.y = y;
    }
  }
  public static final boolean DO_SYSTEMS_CHECK = true;
  public static final ShuffleboardTab SYSTEMS_CHECK_TAB = Shuffleboard.getTab("SYSTEMS CHECK");

  public static Constants getInstance() {
    if (instance == null) {
      instance = new Constants();
    }
    return instance;
  }

  public Constants() {
    RobotType.Type robotType = RobotType.getRobotType();
    // SmartDashboard.putString("Startup/Mac-Address", RobotType.getMACAddress());
    SmartDashboard.putString("Startup/Robot Type", robotType.description);
    switch (robotType) {
      case SKILLSBOT:
        
        break;
      case PRACTICE:
        
        break;
      case COMPETITION:
      default:
        

       
        break;
    }
  }

  public static final class FieldElements {
    public static Translation2d speakerCenter = new Translation2d(0,5.55);
  }

  public static final class Auto {
  }


  // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {

    public static double MaxSpeed = 6; // 6 meters per second desired top speed
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity @ 1.5
    public static double driveAdjustment = 1;
    public static double driveDeadband = 0.1;
    public static double angularDeadband = 0.1;
    public static double driveScale = 5;
  }

  public static final class Pixy {
    public static final double RATIO_TOLERANCE = 0.4;
  }

  public static final class Vision {
    public static final double IMAGE_PROCESSING_LATENCY_MS = 11;
    public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);
    public static final int RED_PIPELINE_INDEX = 0;
    public static final int BLUE_PIPELINE_INDEX = 1;
    public static final double VISION_CAMERA_FIELD_ORIENTATION_SWITCHER = 7.5;
  }

  public static final double MOTOR_MINIMUM_TEMP_CELSIUS = 15.0; // Used in Shuffleboard for temperature dials
  public static final double MOTOR_SHUTDOWN_TEMP_CELSIUS = 70.0;

  public static final int LEDSTRIP_PWM_ID = 9;

  public static final double VISION_TOLERANCE = 1.5;

  public static final String UPPER_CANIVORE_ID = "Upper";

  public static final double VISION_OFFSET = 0;

  public static class Climber {
    public static final double CLIMBER_MAX_SETPOINT = 10;
    public static final double CLIMBER_MIN_SETPOINT = 2.06;
    public static final double CLIMBER_MAX_JOYSTICK_SPEED = 0.2;
    public static final double CLIMBER_MAX_PID_SPEED = 0.2;
  }

  public static class Delivery {
    public static final double DELIVERY_SPEED = 0.9;
  }

  public static class Elevator {
    public static final double ELEVATOR_MAX_SETPOINT = 10;
    public static final double ELEVATOR_MIN_SETPOINT = 2.06;
    public static final double ELEVATOR_MAX_JOYSTICK_SPEED = 0.2;
    public static final double ELEVATOR_MAX_PID_SPEED = 0.2;
  }

  public static class Intake {
    public static final double INTAKE_VELOCITY = 5;
  }

  public static enum LEDState {
    Cone,
    Cube,
    HasGamePiece,
    Nothing;
  }

  public static enum AllianceColor {
    Red,
    Blue,
    UNKNOWN;

    public static AllianceColor getAllianceColor() {
      // If we're blue, return blue. Otherwise default to red (if red or invalid).
      return DriverStation.getAlliance().get() == Alliance.Blue ? Blue : Red;
    }
    public static AllianceColor getOpposingColor() {
      // The inverse of getAllianceColor
      return DriverStation.getAlliance().get() == Alliance.Blue ? Red : Blue;
    }
  }

}
