package frc.robot.nerdyfiles.utilities;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CTREUtils {

  /**
   * Creates a default limit configuration. This is base the current limit
   * configuration we use on our motors to prevent overheating the motors
   * in catastrophic scenerios.
   */
  public static CurrentLimitsConfigs setDefaultCurrentLimit() {
    //TODO: Valdiate with documentation
    CurrentLimitsConfigs defaultCurrentLimit = new CurrentLimitsConfigs();
    defaultCurrentLimit.SupplyCurrentLimit = 40.0;
    defaultCurrentLimit.SupplyCurrentThreshold = 50.0;
    defaultCurrentLimit.SupplyTimeThreshold = 1.0;
    defaultCurrentLimit.SupplyCurrentLimitEnable = true;

    defaultCurrentLimit.StatorCurrentLimit = 50.0;
    defaultCurrentLimit.StatorCurrentLimitEnable = true;
    return defaultCurrentLimit;
  }


  public static CurrentLimitsConfigs setLowCurrentLimit() {
    //TODO: Valdiate with documentation
    CurrentLimitsConfigs lowCurrentLimit = new CurrentLimitsConfigs();
    lowCurrentLimit.SupplyCurrentLimit = 30.0;
    lowCurrentLimit.SupplyCurrentThreshold = 30.0;
    lowCurrentLimit.SupplyTimeThreshold = 1.0;
    lowCurrentLimit.SupplyCurrentLimitEnable = true;

    lowCurrentLimit.StatorCurrentLimit = 30.0;
    lowCurrentLimit.StatorCurrentLimitEnable = true;
    return lowCurrentLimit;
  }
}
