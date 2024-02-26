package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Delivery extends SubsystemBase {
    
    private VictorSPX deliveryMotor = new VictorSPX(30);
    private DigitalInput deliveryTopSensor = new DigitalInput(1);
    private DigitalInput deliveryBottomSensor = new DigitalInput(2);
    private DigitalInput trapSensor = new DigitalInput(3);

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

    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        
        SmartDashboard.putBoolean("Delivery/Top Sensor", getDeliveryTopSensor());
        SmartDashboard.putBoolean("Delivery/Bottom Sensor", getDeliveryBottomSensor());
        SmartDashboard.putBoolean("Delivery/Trap Sensor", getTrapSensor());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }
}

