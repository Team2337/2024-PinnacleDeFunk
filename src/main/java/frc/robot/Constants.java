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



  // Location of the Hub on the field - the center of the field
  public static final Translation2d kHub = new Translation2d(
    Units.feetToMeters(27),
    Units.feetToMeters(13.5)
  );
  public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(103.8);
  public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);

  public static final class Auto {
  }

  // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {

    public enum ModulePosition {
      FRONT_RIGHT(0),
      FRONT_LEFT(1),
      BACK_LEFT(2),
      BACK_RIGHT(3);

      public final int value;

      ModulePosition(int value) {
        this.value = value;
      }
    }
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

  public static final int CLIMBER_LEFT_MOTOR_ID = 16;
  public static final int CLIMBER_RIGHT_MOTOR_ID = 3;
  public static final int CLIMBER_STRING_POT_ID = 3;
  public static final int LEFT_SERVO_ID = 7;
  public static final int RIGHT_SERVO_ID = 8;

  public static final int KICKER_MOTOR = 20;

  public static final int SHOOTER_LEFT_MOTOR = 7;
  public static final int SHOOTER_RIGHT_MOTOR = 14;

  public static final int DELIVERY_MOTOR_ID = 21;
  public static final double DELIVERY_SPEED = 0.275;
  public static final double BOTTOM_TO_TOP_SPEED = 0.4; //0.35

  public static final int INTAKE_MOTOR_ID = 17;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

  public static final int SHOOTER_BEAM_ID = 2;

  public static final int LEDSTRIP_PWM_ID = 9;

  public static final double VISION_TOLERANCE = 1.5;

  public static final String UPPER_CANIVORE_ID = "Upper";

  public static final double VISION_OFFSET = 0;

  public static final double CLIMBER_ROLL = 15;
  public static class Arm {
    public static final double SHOULDER_ARM_LENGTH = 25;
    public static final double ELBOW_ARM_LENGTH = 34;
    public static final double ARM_GEAR_RATIO = 218.75;
    public static final double shoulderP = 0.005;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double WRIST_LOWER_LIMIT = -180.0;
    public static final double WRIST_UPPER_LIMIT = 540.0;
    public static final double SHOULDER_CLOSED_LOOP_SPEED = 0.7;
    public static final double SHOULDER_MAX_SPEED = 1.0;
    public static final double SHOULDER_JOYSTICK_SPEED = 0.4;
    public static final double ELBOW_CLOSED_LOOP_SPEED = 0.7;
    public static final double ELBOW_MAX_SPEED = 1.0;
    public static final double ELBOW_JOYSTICK_SPEED = 0.4;
    public static final double elbowP = 0.2;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;
    public static final double intakeP = 0.05;
    public static final double intakeI = 0.0;
    public static final double intakeD = 0.0;

    public static final double WRIST_ANGLE_ADJUSTMENT = 10.0;
    public static final double ELBOW_LIMIT = 155;
    public static final double IntakeSpinnerOffset = 11;
    
    // Comp Arm
    public enum ArmPosition {
      SCOREHIGHINTERMEDIATE(109.0,32.0,109.0,54.0,161.0,23.0),
      SCOREHIGH(117.0,10.0,114.0,40.0,111.0,23.0), //Shoulder Cone: 121.0 Elbow Cone: 10.0, Wrist Cone: 145
      SCOREHIGHDROP(117.0,18.0,114.0,40.0,111.0,23.0), //Shoulder Cone: 121, Elbow Cone: 15, Wrist Cone: 145
      SCOREMID(75.0,104.0,91.0,100.0,150.0,23.0), //Shoulder Cone: 82, Elbow Cone: 101, Wrist Cone: 165
      SCOREMIDINTAKESIDE(62.0,-70.0,42.0,-62.0,210.0,125.0), //Elbow Cone: 105, Wrist Cone: 165
      SCOREHIGHINTAKESIDE(26.0,-23.0,18.0,-17.0,156.0,52.0), //Elbow Cone: 39.5, Wrist Cone: 145
      SCORELOW(-21.0,145.0,-21.0,145.0,97.0,25.0),
      SUBSTATION(86.0,-69.0,86.0,-76.0,253.0,154.0),
      SUBSTATIONPICKUP(86.0,-85.0,86.0,-92.0,253.0,154.0),
      TELESTANDINGCONE(51.0,-111.0,35.0,-72.0,184.0,75.0),
      TELEFALLINGCONE(24.0,-102.0,35.0,-102.5,162.0,59.0),
      AUTOTELEFALLINGCONE(24.0,-102.0,33.0,-102.5,132.0,59.0),
      AUTOPICKUP(-5.0,9.0,-5.0,9.0,35.0,25.0),
      CARRY(-21.0,145.0,-21.0,145.0,67.0,243.0),
      AUTOCARRY(-21.0,145.0,-21.0,145.0,67.0,85.0),
      SUBSTATIONCARRY(-21.0,145.0,-21.0,145.0,254.0,25.0),
      CARRYINTERMEDIATE(-21, 32,-21,32,35.0,25.0),
      AUTOCARRYINTERMEDIATE(-21,120,-21,120,35.0,25.0),
      FEEDSTATION(70, -134, 70, -130, 67, 184),
      FEEDSTATIONFRONT(79, 140, 79, 140, 280, 178),
      AUTOSCOREHIGH(115.0,36.0,114.0,40.0,111.0,23.0),
      ALTERNATECARRY(90,-115,90,-115,253,59),
      ALTERNATEINTERMEDIATE(90,-90,90,-90,253,59),
      ALTERNATECARRYEND(90,-148,90,-148,253,59),
      AUTOSCOREMID(75.0,107.0,91.0,100.0,140.0,23.0),
      SCORESIDEPICKUPLOW(102, 143, 120, 142, 219, 310),
      FLOORPICKUPYOSHI(-10,-12, -10, -12, 85, 85),
      FLOORPICKUP2YOSHI(-10,-12, -12, -12, 85, 85),
      FLOORPICKUP3YOSHI(-10,-12, -12, -14, 85, 85),
      FLOORPICKUPWRISTYOSHI(-10,-12, -12, -15, 85, 122),//(-10,-12)cube
      FLOORPICKUPWRISTYOSHIRED(-10,-12, 7.0, -33, 85, 122),//(-10,-12)cube
      FLOORPICKUPWRIST2YOSHIRED(-10,-12, 4.0, -47, 85, 122),//(-10,-12)cube
      FLOORPICKUPWRIST2YOSHIYEET(-10,-12, 7.0, -33, 85, 122),//(-10,-12)cube
      FLOORPICKUPWRISTYOSHIYEET(-10,-12, -1.0, -44, 85, 122),//(-10,-12)cube
      FLOORPICKUPWRIST2YOSHI(-10,-12, -12, -14, 85, 100),
      FLOORPICKUPWRIST3YOSHI(-10,-12, -10, -13, 85, 120),//(-13,-16)cube
      FLOORPICKUPWRIST3YOSHIBLUE(-10,-12, -14, -16, 85, 110),//(-13,-16)cube
      FLOORPICKUPWRIST3YOSHIBLUEYEET(-10,-12, -5, -33, 85, 110),//(-13,-16)cube
      FLOORPICKUPWRIST3YOSHIRED(-10,-12, -14, -16, 85, 110),//(-13,-16)cube
      FLOORPICKUPWRIST3YOSHIREDYEET(-10,-12, -5, -33, 85, 110),//(-13,-16)cube
      FLOORPICKUPYOSHIBUMP(-10,-12, -12, -13, 85, 120), //(-9, -9)cube
      FLOORPICKUPYOSHIBUMP1(-10,-12, -14, -13, 85, 120), //(-9, -9)cube
      FLOORPICKUPYOSHIBUMPBLUE(-10,-12, -13, -13, 85, 120), //(-9, -9)cube
      FLOORPICKUP2YOSHIBUMP(-10,-12, -10, -12, 85, 120),
      FLOORPICKUP3YOSHIBUMP(-10,-12, -15.5, -13, 85, 120),//(-12,-14)cube
      FLOORPICKUP3YOSHIBUMPBLUE(-10,-12, -15, -13, 85, 120),//(-12,-14)cube
      FLOORPICKUP4YOSHIBUMP(-10,-12, 0.0, -9, 85, 85),
      NEWAUTOPICKUP(-25.0, 5.0, -25.0, 5.0, 125, 125),
      NEWAUTOPICKUPRED(-25.0, 7.0, -25.0, 7.0, 125, 125),
      NEWAUTOPICKUPREDBUMP(-25.0, 10.0, -25.0, 10.0, 125, 125),
      NEWAUTOPICKUPREDBUMP2(-25.0, 10.0, -25.0, 10.0, 125, 125),
      NEWAUTOPICKUPREDBUMP3(-25.0, 5.0, -25.0, 5.0, 125, 125),
      NEWAUTOPICKUPBLUEBUMP(-25.0, 8.0, -25.0, 8.0, 125, 125);
      //67, -137
      
             
      public final double shoulderCone;
      public final double shoulderCube;
      public final double elbowCone;
      public final double elbowCube;
      public final double wristCone;
      public final double wristCube;

      ArmPosition(double shoulderCone, double elbowCone, double shoulderCube, double elbowCube, double wristCone, double wristCube) {
        this.shoulderCone = shoulderCone;
        this.shoulderCube = shoulderCube;
        this.elbowCone = elbowCone;
        this.elbowCube = elbowCube;
        this.wristCone = wristCone;
        this.wristCube = wristCube;
      }
    }
  
    // Practice Arm
    /* 
    public enum ArmPosition {
    
      SCOREHIGH(110.0,54.0,110.0,70.0,165.0,23.0),
      SCOREMID(78.0,129.0,110.0,90.0,165.0,23.0),
      SCORELOW(100.0,145.0,100.0,145.0,185.0,75.0),
      SUBSTATION(66.0,-48.0,66.0,-61.0,255.0,114.0),
      AUTOPICKUP(-5.0,9.0,-5.0,9.0,35.0,25.0),
      TELESTANDINGCONE(35.0,-72.0,35.0,-72.0,194.0,75.0),
      TELEFALLINGCONE(29.0,-76.0,29.0,-76.0,130.0,75.0),
      CARRY(-21.0,155.0,-21.0,155.0,35.0,25.0),
      CARRYINTERMEDIATE(-21,90,-21,90,35.0,25.0);
      

      public final double shoulderCone;
      public final double shoulderCube;
      public final double elbowCone;
      public final double elbowCube;
      public final double wristCone;
      public final double wristCube;

      ArmPosition(double shoulderCone, double elbowCone, double shoulderCube, double elbowCube, double wristCone, double wristCube) {
        this.shoulderCone = shoulderCone;
        this.shoulderCube = shoulderCube;
        this.elbowCone = elbowCone;
        this.elbowCube = elbowCube;
        this.wristCone = wristCone;
        this.wristCube = wristCube;
      }
    }
 */
  }
/**
 * Based on a 75:1 Gear Ratio
 */
  public static final double TICKS_PER_DEGREES = 426.66667;

  public static final double ELBOW_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS = (TICKS_PER_DEGREES * 90.0);//-54.0);
  public static final double SHOULDER_OFFSET_FOR_PREMADE_SETPOINTS_IN_TICKS = (TICKS_PER_DEGREES * 90.0);//115.0);

  public static enum GamePiece {
    Cone,
    Cube,
    Nothing;
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
