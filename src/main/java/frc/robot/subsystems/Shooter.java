package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorTopLeft = new TalonFX(40);
    private TalonFX shooterMotorTopRight = new TalonFX(41);
    private TalonFX shooterMotorBottomLeft = new TalonFX(42);
    private TalonFX shooterMotorBottomRight = new TalonFX(43);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    private final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
    private final NeutralOut brake = new NeutralOut();

    public Shooter() {

        var setShooterMotorTopLeftToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorTopLeftToDefault);
        var setShooterMotorTopRightToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorTopRightToDefault);
        var setShooterMotorBottomLeftToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorBottomLeftToDefault);
        var setShooterMotorBottomRightToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorBottomRightToDefault);

        TalonFXConfiguration topLeftMotorConfig = new TalonFXConfiguration();
        topLeftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        topLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        topLeftMotorConfig.Slot0.kP = 0.11;
        topLeftMotorConfig.Slot0.kI = 0.5;
        topLeftMotorConfig.Slot0.kD = 0.0001;
        topLeftMotorConfig.Slot0.kV = 0.12;
        topLeftMotorConfig.Voltage.PeakForwardVoltage = 8;
        topLeftMotorConfig.Voltage.PeakReverseVoltage = -8;
        topLeftMotorConfig.Slot1.kP = 0.5;
        topLeftMotorConfig.Slot1.kI = 0.1;
        topLeftMotorConfig.Slot1.kD = 0.001;
        topLeftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        topLeftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        shooterMotorTopLeft.getConfigurator().apply(topLeftMotorConfig);
        
        TalonFXConfiguration topRightMotorConfig = new TalonFXConfiguration();
        topRightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        topRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topRightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topRightMotorConfig.Slot0.kP = 0.11;
        topRightMotorConfig.Slot0.kI = 0.5;
        topRightMotorConfig.Slot0.kD = 0.0001;
        topRightMotorConfig.Slot0.kV = 0.12;
        topRightMotorConfig.Voltage.PeakForwardVoltage = 8;
        topRightMotorConfig.Voltage.PeakReverseVoltage = -8;
        topRightMotorConfig.Slot1.kP = 0.5;
        topRightMotorConfig.Slot1.kI = 0.1;
        topRightMotorConfig.Slot1.kD = 0.001;
        topRightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        topRightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        shooterMotorTopRight.getConfigurator().apply(topRightMotorConfig);

        TalonFXConfiguration bottomLeftMotorConfig = new TalonFXConfiguration();
        bottomLeftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        bottomLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomLeftMotorConfig.Slot0.kP = 0.11;
        bottomLeftMotorConfig.Slot0.kI = 0.5;
        bottomLeftMotorConfig.Slot0.kD = 0.0001;
        bottomLeftMotorConfig.Slot0.kV = 0.12;
        bottomLeftMotorConfig.Voltage.PeakForwardVoltage = 8;
        bottomLeftMotorConfig.Voltage.PeakReverseVoltage = -8;
        bottomLeftMotorConfig.Slot1.kP = 0.5;
        bottomLeftMotorConfig.Slot1.kI = 0.1;
        bottomLeftMotorConfig.Slot1.kD = 0.001;
        bottomLeftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        bottomLeftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        shooterMotorBottomLeft.getConfigurator().apply(bottomLeftMotorConfig);

       TalonFXConfiguration bottomRightMotorConfig = new TalonFXConfiguration();
        bottomRightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        bottomRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomRightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        bottomRightMotorConfig.Slot0.kP = 0.11;
        bottomRightMotorConfig.Slot0.kI = 0.5;
        bottomRightMotorConfig.Slot0.kD = 0.0001;
        bottomRightMotorConfig.Slot0.kV = 0.12;
        bottomRightMotorConfig.Voltage.PeakForwardVoltage = 8;
        bottomRightMotorConfig.Voltage.PeakReverseVoltage = -8;
        bottomRightMotorConfig.Slot1.kP = 0.5;
        bottomRightMotorConfig.Slot1.kI = 0.1;
        bottomRightMotorConfig.Slot1.kD = 0.001;
        bottomRightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        bottomRightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        shooterMotorBottomRight.getConfigurator().apply(bottomRightMotorConfig);
    }

    public void setShooterVelocity(double velocity) {
        shooterMotorTopLeft.setControl(velocityVoltage.withVelocity(velocity));
        shooterMotorBottomLeft.setControl(velocityVoltage.withVelocity(velocity));
        shooterMotorTopRight.setControl(velocityVoltage.withVelocity(velocity));
        shooterMotorBottomRight.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setLeftShooterVelocity(double velocity) {
        shooterMotorTopLeft.setControl(velocityVoltage.withVelocity(velocity));
        shooterMotorBottomLeft.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setRightShooterVelocity(double velocity) {
        shooterMotorTopRight.setControl(velocityVoltage.withVelocity(velocity));
        shooterMotorBottomRight.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setTopShooterTorqueVelocity(double velocity) {
        shooterMotorTopLeft.setControl(torqueVelocity.withVelocity(velocity));
    }

    public void setBottomShooterTorqueVelocity(double velocity) {
        shooterMotorBottomLeft.setControl(torqueVelocity.withVelocity(velocity));
    }

    public double getTopLeftMotorTemp() {
        return shooterMotorTopLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getTopRightMotorTemp() {
        return shooterMotorTopRight.getDeviceTemp().getValueAsDouble();
    }

    public double getBottomLeftMotorTemp() {
        return shooterMotorBottomLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getBottomRightMotorTemp() {
        return shooterMotorBottomRight.getDeviceTemp().getValueAsDouble();
    }

    public void setBrake() {
        shooterMotorTopLeft.setControl(brake);
        shooterMotorBottomLeft.setControl(brake);
        shooterMotorTopRight.setControl(brake);
        shooterMotorBottomRight.setControl(brake);
    }
    public void log() {
        if (Constants.DashboardLogging.SHOOTER) {
            SmartDashboard.putNumber("Shooter/Top Left Motor Temperature", getTopLeftMotorTemp());
            SmartDashboard.putNumber("Shooter/Top Right Motor Temperature", getTopRightMotorTemp());
            SmartDashboard.putNumber("Shooter/Bottom Left Motor Temperature", getBottomLeftMotorTemp());
            SmartDashboard.putNumber("Shooter/Bottom Right Motor Temperature", getBottomRightMotorTemp());
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
        // shuffleBoardSpeed = SmartDashboard.getNumber("ShooterSpeed", 0);
        // setShooterSpeed(shuffleBoardSpeed);
    }
}
