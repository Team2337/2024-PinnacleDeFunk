package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class ShooterPositionVelocity extends SubsystemBase {
  private final TalonFX positionMotor = new TalonFX(50, "Upper");

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final NeutralOut brake = new NeutralOut();
  private ShuffleboardTab shooterPositionTab = Shuffleboard.getTab("Shooter Position");
  private GenericEntry shooterPosition = shooterPositionTab.add("Shooter Position", 0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public ShooterPositionVelocity() {

    var setPositionMotorToDefault = new TalonFXConfiguration();
    positionMotor.getConfigurator().apply(setPositionMotorToDefault);

    TalonFXConfiguration positionMotorConfig = new TalonFXConfiguration();
        positionMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        positionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        positionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        positionMotorConfig.Slot0.kP = 0.11;
        positionMotorConfig.Slot0.kI = 0.5;
        positionMotorConfig.Slot0.kD = 0.0001;
        positionMotorConfig.Slot0.kV = 0.12;
        positionMotorConfig.Voltage.PeakForwardVoltage = 5;
        positionMotorConfig.Voltage.PeakReverseVoltage = -5;
        
        
        positionMotor.getConfigurator().apply(positionMotorConfig);
  }

  public void setShooterPositionVelocity(double velocity) {
      positionMotor.setControl(velocityVoltage.withVelocity(velocity));
  }
  
  public double getShooterPositionVelocity() {
    return positionMotor.getVelocity().getValueAsDouble();
  }
  
  public double getShooterPositionTemp() {
    return positionMotor.getDeviceTemp().getValueAsDouble();
  }

  public void setBrake() {
    positionMotor.setControl(brake);
  }

  public void log() {
        if (Constants.DashboardLogging.SHOOTER) {
            SmartDashboard.putNumber("Shooter/Shooter Position Motor Temperature", getShooterPositionTemp());
        }
        SmartDashboard.putNumber("Shooter/ShooterPosition Velocity", getShooterPositionVelocity());
    }

  public void initialize() {
  }

  @Override
  public void periodic() {
    super.periodic();
      log();
  }
}
