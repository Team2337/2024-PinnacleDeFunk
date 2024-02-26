package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Intake extends SubsystemBase {

    private TalonFX intakeMotorLeft = new TalonFX(20,"Upper");
    private TalonFX intakeMotorRight = new TalonFX(21, "Upper");
    private DigitalInput intakeSensor = new DigitalInput(0);
    
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    private GenericEntry shuffleboardSpeed = intakeTab 
        .add("IntakeSpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    private GenericEntry addDashNum = intakeTab.add("Intake Speed from dash", 0).getEntry();
    private double speedFromDash = 0; 
    private static double motorShutDownTempCelcius = 70; 

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
        setupShuffleboard(true);

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
        return motor.getDeviceTemp().getValueAsDouble() >= motorShutDownTempCelcius;
    }

    public double intakeSpeedFromDash() {
        return speedFromDash;
    }
    public void log() {
        if (Constants.DashboardLogging.INTAKE) {
            SmartDashboard.putNumber("Intake/Average Motor Temperature", getIntakeAverageTemp());
            SmartDashboard.putNumber("Intake/Left Motor Temperature", getIntakeLeftTemp());
            SmartDashboard.putNumber("Intake/Right Motor Temperature", getIntakeRightTemp());
            SmartDashboard.putNumber("Intake/Intake Left Velo", intakeMotorLeft.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Right Velo", intakeMotorRight.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Left Current", intakeMotorLeft.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Intake Right Current", intakeMotorRight.getStatorCurrent().getValueAsDouble());
        }
        SmartDashboard.putBoolean("Intake/Intake Sensor", getIntakeSensor());
        addDashNum.setDouble(intakeSpeedFromDash());
    }

    private void setupShuffleboard(boolean logEnable) {
        if (logEnable) {
            ShuffleboardLayout widget = intakeTab.getLayout("Diagnositics", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(4, 0);
            widget.addNumber("Left Motor Temp", this::getIntakeLeftTemp);
            widget.addNumber("Right Motor Temp", this::getIntakeRightTemp);
            widget.addBoolean("Motors Overheating?", () -> isOverheated());
        }
    }

    public void initialize() {
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
        speedFromDash = shuffleboardSpeed.getDouble(0);
        //setIntakeSpeed(speedFromDash);

    }

    public Command setIntakeSpeedLocal() {
        return this.startEnd(
            () -> setIntakeSpeed(0.1), 
            () -> setIntakeSpeed(0.0)
        );
    }
}
