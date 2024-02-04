package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Delivery extends SubsystemBase {
    
    private TalonFX deliveryMotor = new TalonFX(30);
    private DigitalInput deliveryTopSensor = new DigitalInput(1);
    private DigitalInput deliveryBottomSensor = new DigitalInput(2);
    private DigitalInput trapSensor = new DigitalInput(3);
    private ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");
    private GenericEntry shuffleboardSpeed = deliveryTab 
        .add("DeliverySpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    private GenericEntry addDashNum = deliveryTab.add("Delivery Speed from dash", 0).getEntry();
    private double speedFromDash = 0; 

    public Delivery() { 
        
        var setDeliveryMotorLeftToDefault = new TalonFXConfiguration();
        deliveryMotor.getConfigurator().apply(setDeliveryMotorLeftToDefault);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deliveryMotor.getConfigurator().apply(leftMotorConfig);

        deliveryMotor.setSafetyEnabled(true);
    }

    public void setDeliverySpeed(double speed) {
        deliveryMotor.set(speed);
    }

    public void stopMotors() {
        deliveryMotor.stopMotor();
    }

    public boolean getDeliveryTopSensor() {
        return !deliveryTopSensor.get();
    }

    public boolean getDeliveryBottomSensor() {
        return !deliveryBottomSensor.get();
    }

    public boolean getTrapSensor() {
        return !trapSensor.get();
    }

    public double deliverySpeedFromDash() {
        return speedFromDash;
    }
    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        SmartDashboard.putBoolean("Delivery/Top Sensor", getDeliveryTopSensor());
        SmartDashboard.putBoolean("Delivery/Bottom Sensor", getDeliveryBottomSensor());
        SmartDashboard.putBoolean("Delivery/Trap Sensor", getTrapSensor());
        addDashNum.setDouble(deliverySpeedFromDash());
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
}

