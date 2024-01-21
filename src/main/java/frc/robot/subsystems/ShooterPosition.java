package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.nerdyfiles.utilities.CTREUtils;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class ShooterPosition extends SubsystemBase {
  private final TalonFX positionMotor = new TalonFX(50);
  
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  private final PositionVoltage voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  /* Start at position 0, no feed forward, use slot 1 */
  private final PositionTorqueCurrentFOC torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut brake = new NeutralOut();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public ShooterPosition() {

    var setPositionMotorToDefault = new TalonFXConfiguration();
    positionMotor.getConfigurator().apply(setPositionMotorToDefault);

    TalonFXConfiguration positionMotorConfig = new TalonFXConfiguration();
        positionMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        positionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        positionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        positionMotorConfig.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
        positionMotorConfig.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
        // Peak output of 8 volts
        positionMotorConfig.Voltage.PeakForwardVoltage = 8;
        positionMotorConfig.Voltage.PeakReverseVoltage = -8;
        
        positionMotorConfig.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
        positionMotorConfig.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
        // Peak output of 130 amps
        positionMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 130;
        positionMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = 130;
        
        positionMotor.getConfigurator().apply(positionMotorConfig);
        /* Retry config apply up to 5 times, report if failure */
    //    StatusCode status = StatusCode.StatusCodeNotInitialized;
    //     for (int i = 0; i < 5; ++i) {
    //         status = m_fx.getConfigurator().apply(configs);
    //   if (status.isOK()) break;
    // }
    // if(!status.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + status.toString());
    // }

    /* Make sure we start at 0 */
    //TODO: validate this value before putting on robot
    positionMotor.setPosition(0);
  }

  public void setShooterPosition(double position) {
    positionMotor.setControl(voltagePosition.withPosition(position));
  }

  public void holdShooterPosition() {
    positionMotor.setControl(brake);
  }

  @Override
  public void periodic() {
    super.periodic();

  }
}
