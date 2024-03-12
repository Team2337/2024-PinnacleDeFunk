package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliveryServo extends SubsystemBase {
    
    private Servo noteStopperServo = new Servo(2);
    private LaserCan laserCan = new LaserCan(0);

    public DeliveryServo() { 

            // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
            try {
                laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
                //laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8));
                laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
                laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed! " + e);
            }
    }

    public double getLaserCan() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return 0;
        }
    }

    public void servoSet(double pos) {
        noteStopperServo.set(pos);
    }

    public void servoSetPosition(double pos) {
        noteStopperServo.setPosition(pos);
    }
    /**
     * Sets the angle of the servo
     * @param pos - 0-360 degrees
     */
    public void servoSetAngle(double pos) {
        noteStopperServo.setAngle(pos);
    }

    public double getServo() {
        return noteStopperServo.get();
    }

    public void engageNoteStop() {
        servoSet(0.7);
    }

    public void disengageNoteStop() {
        servoSet(0.1);
    }

    public void log() {
        if (Constants.DashboardLogging.DELIVERY) {
        }
        SmartDashboard.putNumber("Servo Pos", getServo());
        SmartDashboard.putNumber("LaserCAN", getLaserCan());
    }


    @Override
    public void periodic() {
        super.periodic();
        log();
    }
}
