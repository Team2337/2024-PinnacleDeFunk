package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
    private DigitalInput deliverySensor = new DigitalInput(2);
    private ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");
    private GenericEntry shuffleboardSpeed = deliveryTab 
        .add("DeliverySpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
    private GenericEntry addDashNum = deliveryTab.add("Delivery Speed from dash", 0).getEntry();
    private double speedFromDash = 0; 

    public Delivery() { 
        deliveryMotor.setInverted(false);
        deliveryMotor.setNeutralMode(NeutralMode.Brake);
        
    }

    public void setDeliverySpeed(double speed) {
        deliveryMotor.set(ControlMode.PercentOutput, speed);
        
    }

    public void stopMotors() {
        deliveryMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean getDeliverySensor() {
        return deliverySensor.get();
    }

    public double deliverySpeedFromDash() {
        return speedFromDash;
    }
    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        SmartDashboard.putBoolean("Delivery/Top Sensor", getDeliverySensor());
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
