package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class Delivery extends SubsystemBase {
    
    private TalonFX deliveryMotorLeft = new TalonFX(30);
    private TalonFX deliveryMotorRight = new TalonFX(31);
    private DigitalInput deliverySensor = new DigitalInput(2);
    private ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");
    private GenericEntry shuffleboardSpeed = deliveryTab 
        .add("DeliverySpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    private GenericEntry addDashNum = deliveryTab.add("Delivery Speed from dash", 0).getEntry();
    private double speedFromDash = 0; 
    private static double motorShutDownTempCelcius = 70; 

    public Delivery() { 
        
        var setDeliveryMotorLeftToDefault = new TalonFXConfiguration();
        deliveryMotorLeft.getConfigurator().apply(setDeliveryMotorLeftToDefault);

        var setDeliveryMotorRightToDefault = new TalonFXConfiguration();
        deliveryMotorRight.getConfigurator().apply(setDeliveryMotorRightToDefault);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deliveryMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deliveryMotorRight.getConfigurator().apply(rightMotorConfig);
        
        deliveryMotorLeft.setSafetyEnabled(true);
        deliveryMotorRight.setSafetyEnabled(true);
        setupShuffleboard(true);

    }

    public void setDeliverySpeed(double speed) {
        deliveryMotorLeft.set(speed);
        deliveryMotorRight.set(speed);
    }

    public void setLeftDeliverySpeed(double speed) {
        deliveryMotorLeft.set(speed);
    }

    public void setRightDeliverySpeed(double speed) {
        deliveryMotorRight.set(speed);
    }

    public void stopMotors() {
        deliveryMotorLeft.stopMotor();
        deliveryMotorRight.stopMotor();
    }

    public double getDeliveryLeftTemp() {
        return deliveryMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getDeliveryRightTemp() {
        return deliveryMotorRight.getDeviceTemp().getValueAsDouble();
    }

    public double getDeliveryAverageTemp() {
        return (deliveryMotorLeft.getDeviceTemp().getValueAsDouble() +  deliveryMotorRight.getDeviceTemp().getValueAsDouble()) / 2;
    }

    public boolean getDeliverySensor() {
        return deliverySensor.get();
    }

    private boolean isOverheated() {
        return isMotorOverheated(deliveryMotorLeft) || isMotorOverheated(deliveryMotorRight);
    }

    private boolean isMotorOverheated(TalonFX motor) {
        return motor.getDeviceTemp().getValueAsDouble() >= motorShutDownTempCelcius;
    }

    public double deliverySpeedFromDash() {
        return speedFromDash;
    }
    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
            SmartDashboard.putNumber("Delivery/Average Motor Temperature", getDeliveryAverageTemp());
            SmartDashboard.putNumber("Delivery/Left Motor Temperature", getDeliveryLeftTemp());
            SmartDashboard.putNumber("Delivery/Right Motor Temperature", getDeliveryRightTemp());
        }
        SmartDashboard.putBoolean("Delivery/Top Sensor", getDeliverySensor());
        addDashNum.setDouble(deliverySpeedFromDash());
    }

    private void setupShuffleboard(boolean logEnable) {
        if (logEnable) {
            ShuffleboardLayout widget = deliveryTab.getLayout("Diagnositics", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(4, 0);
            widget.addNumber("Left Motor Temp", this::getDeliveryLeftTemp);
            widget.addNumber("Right Motor Temp", this::getDeliveryRightTemp);
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
        setDeliverySpeed(speedFromDash);

    }

    public Command setDeliverySpeedLocal() {
        return this.startEnd(
            () -> setDeliverySpeed(0.1), 
            () -> setDeliverySpeed(0.0)
        );
    }
}
