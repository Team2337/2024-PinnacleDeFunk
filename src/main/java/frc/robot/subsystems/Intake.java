package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Intake extends SubsystemBase {

    private TalonFX intakeMotorLeft = new TalonFX(20,"Upper");
    private TalonFX intakeMotorRight = new TalonFX(21, "Upper");
    private DigitalInput intakeSensor = new DigitalInput(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private double logDelayCounter = 0;

    public Intake() { 
        
        var setIntakeMotorLeftToDefault = new TalonFXConfiguration();
        intakeMotorLeft.getConfigurator().apply(setIntakeMotorLeftToDefault);

        var setIntakeMotorRightToDefault = new TalonFXConfiguration();
        intakeMotorRight.getConfigurator().apply(setIntakeMotorRightToDefault);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Slot0.kP = 0.21;
        leftMotorConfig.Slot0.kI = 0.5;
        leftMotorConfig.Slot0.kD = 0.0001;
        leftMotorConfig.Slot0.kV = 0.12;
        leftMotorConfig.Voltage.PeakForwardVoltage = 12;
        leftMotorConfig.Voltage.PeakReverseVoltage = -12;
        intakeMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotorConfig.Slot0.kP = 0.21;
        rightMotorConfig.Slot0.kI = 0.5;
        rightMotorConfig.Slot0.kD = 0.0001;
        rightMotorConfig.Slot0.kV = 0.12;
        rightMotorConfig.Voltage.PeakForwardVoltage = 12;
        rightMotorConfig.Voltage.PeakReverseVoltage = -12;
        intakeMotorRight.getConfigurator().apply(rightMotorConfig);

        setupShuffleboard();
    }

    public void setIntakeSpeed(double speed) {
        intakeMotorLeft.set(speed);
        intakeMotorRight.set(speed);
    }

    public void setIntakeVelocity(double velocity) {
        intakeMotorLeft.setControl(velocityVoltage.withVelocity(velocity));
        intakeMotorRight.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setDriveOver(double velocity) {
        intakeMotorLeft.setControl(velocityVoltage.withVelocity(velocity));
        intakeMotorRight.setControl(velocityVoltage.withVelocity(-velocity));
        // intakeMotorLeft.setControl(velocityVoltage.withVelocity(100));
        // intakeMotorRight.setControl(velocityVoltage.withVelocity(-100));
    }

    public void setLeftIntakeSpeed(double speed) {
        intakeMotorLeft.set(speed);
    }

    public void setRightIntakeSpeed(double speed) {
        intakeMotorRight.set(speed);
    }

    public void stopMotors() {
        intakeMotorLeft.stopMotor();
        intakeMotorRight.stopMotor();
    }

    public double getIntakeLeftTemp() {
        return intakeMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getIntakeRightTemp() {
        return intakeMotorRight.getDeviceTemp().getValueAsDouble();
    }

    public double getIntakeAverageTemp() {
        return (intakeMotorLeft.getDeviceTemp().getValueAsDouble() +  intakeMotorRight.getDeviceTemp().getValueAsDouble()) / 2;
    }

    public boolean getIntakeSensor() {
        return !intakeSensor.get();
    }


    private boolean isOverheated() {
        return isMotorOverheated(intakeMotorLeft) || isMotorOverheated(intakeMotorRight);
    }

    private boolean isMotorOverheated(TalonFX motor) {
        return motor.getDeviceTemp().getValueAsDouble() >= Constants.Global.motorShutDownTempCelcius;
    }

    public void log() {
        if (Constants.DashboardLogging.INTAKE) {
            SmartDashboard.putNumber("Intake/Intake Left Velo", intakeMotorLeft.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Right Velo", intakeMotorRight.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Left Current", intakeMotorLeft.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Right Current", intakeMotorRight.getStatorCurrent().getValueAsDouble());
        }
        if (Constants.DashboardLogging.TEMP) {
            if (logDelayCounter >= Constants.Global.logDelay) {
                SmartDashboard.putNumber("Temps/Intake Left Motor Temperature", getIntakeLeftTemp());
                SmartDashboard.putNumber("Temps/Intake Right Motor Temperature", getIntakeRightTemp());
                SmartDashboard.putBoolean("Temps/Intake Overheating?", isOverheated());
                logDelayCounter = 0;
            }
        }
        logDelayCounter++;
    }

    public void setupShuffleboard() {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
      
      systemsCheck.addBoolean("Intake Sensor", () -> getIntakeSensor())
        .withPosition(SystemsCheckPositions.INTAKE_SENSOR.x, SystemsCheckPositions.INTAKE_SENSOR.y)
        .withSize(2, 2);
    
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }

    public Command setIntakeSpeedLocal() {
        return this.startEnd(
            () -> setIntakeSpeed(0.1), 
            () -> setIntakeSpeed(0.0)
        );
    }
}
