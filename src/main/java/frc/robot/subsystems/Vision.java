package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

  private enum LimelightKey {
    Pose("botpose"),
    Pipeline("pipeline"),
    LEDMode("ledMode"),
    X("tx"),
    Y("ty"),
    LATENCY("tl"),
    VALID_TARGET("tv"),
    STREAM("stream");


    private String key;

    private LimelightKey(String key) {
      this.key = key;
    }
  }

  public enum LimelightColor {
    BLUE,
    ORANGE,
    PINK
  }

  public static enum Pipeline {
    DEFAULT(0);

    private int number;

    private Pipeline(int number) {
      this.number = number;
    }

    public static Pipeline withNumber(int number) {
      for (Pipeline pipeline : Pipeline.values()) {
        if (pipeline.number == number) {
          return pipeline;
        }
      }
      return null;
    }
  }

  public static enum LEDMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int value;

    private LEDMode(int value) {
      this.value = value;
    }

    public static LEDMode withValue(int value) {
      for (LEDMode mode : LEDMode.values()) {
        if (mode.value == value) {
          return mode;
        }
      }
      return null;
    }
  }

  // Only fetch our LL values once per periodic cycle
  private Pipeline currentPipelineB, currentPipelineO;
  private LEDMode currentLEDModeB, currentLEDModeO;
  private double txB, txO = 0.0;
  private double tyB, tyO = 0.0;
  private double[] tposeB, tposeO = {0, 0, 0, 0, 0, 0};
  private double[] defaultPose = {0, 0, 0, 0, 0, 0};
  private double latencyB, latencyO = 0.0;
  private boolean hasValidTargetB, hasValidTargetO = false;
  private double distanceToTargetMetersB = 0.0;
  private double distanceToTargetMetersO = 0.0;
  private String allianceColor;
  private String botPoseColor = "botpose_wpi";
  RobotContainer robotContainer;
  private String pipelineAllianceColor;

  public int relocalizeCounter = 0;

  public Vision(RobotContainer robotContainer) {
    // Automatically switch our Limelight to our default pipeline on construction
    switchPipeLine(Pipeline.DEFAULT, LimelightColor.BLUE);
    switchPipeLine(Pipeline.DEFAULT, LimelightColor.ORANGE);
    pipelineAllianceColor = "blue";
    // Systems check
    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      // systemsCheck.addBoolean("Limelight Connected", () -> (latency > 0))
      //   .withPosition(SystemsCheckPositions.LIMELIGHT.x, SystemsCheckPositions.LIMELIGHT.y)
      //   .withSize(3, 3);
    }
    this.robotContainer = robotContainer;
    }

  @Override
  public void periodic() {
    botPoseColor = "botpose_wpi";
    allianceColor = robotContainer.allianceColor;
     
    //SmartDashboard.putBoolean("vision-switch", robotContainer.getBlackSwitchStatus());
    SmartDashboard.putString("vision-color", allianceColor);
    botPoseColor = botPoseColor + allianceColor;
    //SmartDashboard.putString("Bot Pose Color", botPoseColor);
    //SmartDashboard.putString("Pipeline Alliance Color", pipelineAllianceColor);

    // tposeB = NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry(botPoseColor).getDoubleArray(defaultPose);
    // currentPipelineB = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline, LimelightColor.BLUE));
    // currentLEDModeB = LEDMode.withValue(getIntValue(LimelightKey.LEDMode, LimelightColor.BLUE));
    // txB = getDoubleValue(LimelightKey.X, LimelightColor.BLUE);
    // tyB = getDoubleValue(LimelightKey.Y, LimelightColor.BLUE);
    // latencyB = getDoubleValue(LimelightKey.LATENCY, LimelightColor.BLUE);
    // hasValidTargetB = getDoubleValue(LimelightKey.VALID_TARGET, LimelightColor.BLUbotPoseColorE) == 1.0;

    // distanceToTargetMetersB = 0.0;
    // if (hasValidTargetB) {
    //   //distanceToTargetMetersB = calculateDistanceToTargetMeters(LimelightColor.BLUE);
    // }

    // if (currentPipelineB == Pipeline.DEFAULT && pipelineAllianceColor == "blue") {
    //   NetworkTable limelightBlue = NetworkTableInstance.getDefault().getTable("limelight-blue");
    //   limelightBlue.getEntry("pipeline").setNumber(1);
    // }

    tposeO = NetworkTableInstance.getDefault().getTable("limelight-orange").getEntry(botPoseColor).getDoubleArray(defaultPose);
    currentPipelineO = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline, LimelightColor.ORANGE));
    currentLEDModeO = LEDMode.withValue(getIntValue(LimelightKey.LEDMode, LimelightColor.ORANGE));
    txO = getDoubleValue(LimelightKey.X, LimelightColor.ORANGE);
    tyO = getDoubleValue(LimelightKey.Y, LimelightColor.ORANGE);
    latencyO = getDoubleValue(LimelightKey.LATENCY, LimelightColor.ORANGE);
    hasValidTargetO = getDoubleValue(LimelightKey.VALID_TARGET, LimelightColor.ORANGE) == 1.0;

    distanceToTargetMetersO = 0.0;
    if (hasValidTargetO) {
      //distanceToTargetMetersO = calculateDistanceToTargetMeters(LimelightColor.ORANGE);
    }

    


    log();
  }

  private void log() {
    if (Constants.DashboardLogging.VISION) {
      SmartDashboard.putNumber("Vision/# of relocalization", relocalizeCounter);
      // SmartDashboard.putNumber("Vision/Blue Vision Pose X", getVisionPoseX(LimelightColor.BLUE));
      // SmartDashboard.putNumber("Vision/Blue Vision Pose Y", getVisionPoseY(LimelightColor.BLUE));
      // SmartDashboard.putNumber("Vision/latency blue", getLatency(LimelightColor.BLUE));
      // SmartDashboard.putBoolean("Vision/Valid Target blue", hasActiveTarget(LimelightColor.BLUE));
      SmartDashboard.putNumber("Vision/Orange Vision Pose X", getVisionPoseX(LimelightColor.ORANGE));
      SmartDashboard.putNumber("Vision/Orange Vision Pose Y", getVisionPoseY(LimelightColor.ORANGE));
      SmartDashboard.putNumber("Vision/latency orange", getLatency(LimelightColor.ORANGE));
      SmartDashboard.putBoolean("Vision/Valid Target orange", hasActiveTarget(LimelightColor.ORANGE));
      // SmartDashboard.putNumber("Vision/Blue Distance To Target (inches)", Units.metersToInches(distanceToTargetMetersB));
      SmartDashboard.putNumber("Vision/Orange Distance To Target (inches)", Units.metersToInches(distanceToTargetMetersO));
    //  SmartDashboard.putNumber("Vision/Robot Pose X", Units.metersToInches(getVisionPoseX()));
    //SmartDashboard.putNumber("Vision/Robot Pose Y", Units.metersToInches(getVisionPoseY()));
    }
  }

  /** Limelight Network Table Access */

  private static NetworkTableEntry getLimelightEntryOrange(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight-orange").getEntry(key.key);
  }

  private static NetworkTableEntry getLimelightEntryBlue(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight-blue").getEntry(key.key);
  }


  /**
   * Get the double value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the double value for the key in the Limelight NetworkTables. 0.0 if
   *         the key does not exist.
   */
  private static double getDoubleValue(LimelightKey key, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).getDouble(0);
    } else {
      return getLimelightEntryOrange(key).getDouble(0);
    }
  }


  /**
   * Get the int value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the int value for the key in the Limelight NetworkTables. 0 if
   *         the key does not exist.
   */
  private static int getIntValue(LimelightKey key, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).getNumber(0).intValue();
    } else {
      return getLimelightEntryOrange(key).getNumber(0).intValue();
    }
  }

  /**
   * Sets the Limelight entry value.
   *
   * @param value the value to set
   * @return False if the entry exists with a different type
   */
  private static boolean setValue(LimelightKey key, LimelightColor color, Number value) {
    if (color == LimelightColor.BLUE) {
      return getLimelightEntryBlue(key).setNumber(value);
    } else {
      return getLimelightEntryOrange(key).setNumber(value);
    }
  }

  /** Limelight API */

  /**
   * Sets the LED mode to on, off, or blink
   */
  public void setLEDMode(LEDMode mode, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      if (currentLEDModeB != mode) {
        if (setValue(LimelightKey.LEDMode, LimelightColor.BLUE, mode.value)) {
          currentLEDModeB = mode;
        }
      }
    } else {
      if (currentLEDModeO != mode) {
        if (setValue(LimelightKey.LEDMode, LimelightColor.ORANGE, mode.value)) {
          currentLEDModeO = mode;
        }
      }
    }
  }

  /**
   * Gets the Limelight LED mode
   */
  public LEDMode getLEDMode(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return currentLEDModeB;
    } else {
      return currentLEDModeO;
    }
  }

  /**
   * Sets the pipeline of the Limelight
   */
  public void switchPipeLine(Pipeline pipeline, LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      if (currentPipelineB != pipeline) {
        if (setValue(LimelightKey.Pipeline, LimelightColor.BLUE, pipeline.number)) {
          currentPipelineB = pipeline;
        }
      }
    } else {
      if (currentPipelineO != pipeline) {
        if (setValue(LimelightKey.Pipeline, LimelightColor.ORANGE, pipeline.number)) {
          currentPipelineO = pipeline;
        }
      }
    }
  }

  /**
   * Gets the current pipeline on the Limelight
   * @return - Double value Limelight pipeline (0 -> 9)
   */
  public Pipeline getPipeline(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return currentPipelineB;
    } else {
      return currentPipelineO;
    }
  }

  /**
   * Returns the horizontal offset from the crosshair to the target in degree (-27 to 27) 
   * @return - tx value from the limelight plus an offset if desired to allow for fine tuning of vision centering or if we want to shoot to the side of the target
   */
  public double getTx(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return txB + Constants.VISION_OFFSET;
    } else {
      return txO + Constants.VISION_OFFSET;
    }
  }

  /**
   * Returns the Vertical offset from the crosshair to the target in degrees (-20.5 to 20.5)
   * @return ty value from the limelight
   */
  public double getTy(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tyB;
    } else {
      return tyO;
    }
  }

  public double[] getVisionPose(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB;
    } else {
      return tposeO;
    }
  }

  public double getVisionPoseX(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB[0];
    } else {
      return tposeO[0];
    }
  }

  public double getVisionPoseY(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return tposeB[1];
    } else {
      return tposeO[1];
    }
  }

  public double getVisionRotation(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return tposeB[5];
    } else {
      return tposeO[5];
    }
  }

  public double getLatency(LimelightColor color) {
    if (color == LimelightColor.BLUE) {
      return latencyB;
    } else {
      return latencyO;
    }
  }

  public boolean hasActiveTarget(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return hasValidTargetB;
    } else {
      return hasValidTargetO;
    }
  }

  public boolean isOnTarget(LimelightColor color) { 
    if (color == LimelightColor.BLUE) {
      return Math.abs(txB) < Constants.VISION_TOLERANCE;
    } else {
      return Math.abs(txO) < Constants.VISION_TOLERANCE;
    }
  }

  public double getDistanceToCenterHubMeters() {
    return distanceToTargetMetersB + Constants.Vision.VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS;
  }

  public void incrementRelocalizeCounter() {
    relocalizeCounter++;
  }

  // private double calculateDistanceToTargetMeters(LimelightColor color) { 
  //   return LimelightUtilities.calculateDistanceToTargetMeters(
  //     Constants.getInstance().LIMELIGHT_CAMERA_HEIGHT_METERS,
  //     Constants.HUB_HEIGHT_METERS,
  //     Constants.getInstance().LIMEILGHT_CAMERA_ANGLE,
  //     Rotation2d.fromDegrees(getTy(color)),
  //     Rotation2d.fromDegrees(getTx(color))
  //   );
  // }

  public double calculateDistanceToTargetInches() {
    return Units.metersToInches(distanceToTargetMetersB);
  }

}
