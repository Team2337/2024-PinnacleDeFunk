package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Delivery extends SubsystemBase {
    
    private VictorSPX deliveryMotor = new VictorSPX(30);
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
        deliveryMotor.setInverted(true);
        deliveryMotor.setNeutralMode(NeutralMode.Brake);
        
        
    }

    public void setDeliverySpeed(double speed) {
        deliveryMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void stopMotors() {
        deliveryMotor.set(VictorSPXControlMode.PercentOutput, 0);
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

