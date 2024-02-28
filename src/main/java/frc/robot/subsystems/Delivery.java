package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;

public class Delivery extends SubsystemBase {
    
    private VictorSPX deliveryMotor = new VictorSPX(30);
    private DigitalInput deliveryTopSensor = new DigitalInput(1);
    private DigitalInput deliveryBottomSensor = new DigitalInput(2);

    public Delivery() { 
        deliveryMotor.setInverted(true);
        deliveryMotor.setNeutralMode(NeutralMode.Brake);
        setupShuffleboard();
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

    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        
    }

    public void setupShuffleboard() {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
      
      systemsCheck.addBoolean("Top Sensor", () -> getDeliveryTopSensor())
        .withPosition(SystemsCheckPositions.TOP_SENSOR.x, SystemsCheckPositions.TOP_SENSOR.y)
        .withSize(2, 2);
      systemsCheck.addBoolean("Bottom Sensor", () -> getDeliveryBottomSensor())
        .withPosition(SystemsCheckPositions.BOTTOM_SENSOR.x, SystemsCheckPositions.BOTTOM_SENSOR.y)
        .withSize(2, 2);
    
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }
}

