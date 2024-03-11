package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class Delivery extends SubsystemBase {
    
    private VictorSPX deliveryMotor = new VictorSPX(30);
    private DigitalInput deliveryTopSensor = new DigitalInput(1);
    private DigitalInput deliveryBottomSensor = new DigitalInput(2);
    private LaserCan laserCan = new LaserCan(0);

    public Delivery() { 
        deliveryMotor.setInverted(true);
        deliveryMotor.setNeutralMode(NeutralMode.Brake);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            //laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8));
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
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

    public double getLaserCan() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm; 
        } else {
            return 0;
        }
    }

    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        SmartDashboard.putNumber("LaserCAN", getLaserCan());
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

