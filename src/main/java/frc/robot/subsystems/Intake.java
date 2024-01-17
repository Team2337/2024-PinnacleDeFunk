package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Intake extends SubsystemBase {
    
    private TalonFX intakeMotorLeft = new TalonFX(20);
    private TalonFX intakeMotorRight = new TalonFX(21);
    private DigitalInput intakeSensorTop = new DigitalInput(0);
    private DigitalInput intakeSensorBottom = new DigitalInput(1);
    private double shuffleBoardSpeed = 0;
    private double intakeMotorSpeed = 0;

    public Intake() { 

        Shuffleboard.getTab("Intake")
        .add("IntakeSpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

        var setIntakeMotorLeftToDefault = new TalonFXConfiguration();
        intakeMotorLeft.getConfigurator().apply(setIntakeMotorLeftToDefault);

        var setIntakeMotorRightToDefault = new TalonFXConfiguration();
        intakeMotorRight.getConfigurator().apply(setIntakeMotorRightToDefault);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorRight.getConfigurator().apply(rightMotorConfig);
        
    }

    public void setIntakeSpeed(double speed) {
        intakeMotorLeft.set(speed);
        intakeMotorRight.set(speed);
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

    public boolean getIntakeTopSensor() {
        return intakeSensorTop.get();
    }

    public boolean getIntakeBottomSensor() {
        return intakeSensorBottom.get();
    }

    public void log() {
        if (Constants.DashboardLogging.INTAKE) {
            SmartDashboard.putNumber("Intake/Average Motor Temperature", getIntakeAverageTemp());
            SmartDashboard.putNumber("Intake/Left Motor Temperature", getIntakeLeftTemp());
            SmartDashboard.putNumber("Intake/Right Motor Temperature", getIntakeRightTemp());
        }
        SmartDashboard.putBoolean("Intake/Top Sensor", getIntakeTopSensor());
        SmartDashboard.putBoolean("Intake/Bottom Sensor", getIntakeBottomSensor());
//        SmartDashboard.putNumber("Set Intake Speed", getIntakeSpeed())
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
        shuffleBoardSpeed = SmartDashboard.getNumber("IntakeSpeed", 0);
        setIntakeSpeed(shuffleBoardSpeed);
    }
}
