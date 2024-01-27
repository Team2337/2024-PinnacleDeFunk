package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
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

public class Climber extends SubsystemBase{
    private TalonFX climbMotorLeft = new TalonFX(60);
    private TalonFX climbMotorRight = new TalonFX(61); 

    private ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    private GenericEntry leftClimbVelocityFromDash = climberTab
        .add("Left Climber Velocity", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -1, "Max", 1))
        .getEntry();

    private GenericEntry rightClimbVelocityFromDash = climberTab
        .add("Right Climber Velocity", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -1, "Max", 1))
        .getEntry();

    private GenericEntry climbVelocityFromDash = climberTab
        .add("Climber Velocity", 0) 
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -1, "Max", 1))
        .getEntry();

    public Climber() {
        var setClimbMotorLeftToDefault = new TalonFXConfiguration();
        climbMotorLeft.getConfigurator().apply(setClimbMotorLeftToDefault);
        
        var setClimbMotorRightToDefault = new TalonFXConfiguration();
        climbMotorLeft.getConfigurator().apply(setClimbMotorRightToDefault);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Slot0.kP = 0.11;
        leftMotorConfig.Slot0.kI = 0.5;
        leftMotorConfig.Slot0.kD = 0.0001;
        leftMotorConfig.Slot0.kV = 0.12;
        leftMotorConfig.Voltage.PeakForwardVoltage = 12;
        leftMotorConfig.Voltage.PeakReverseVoltage = -12;
        leftMotorConfig.Slot1.kP = 0.5;
        leftMotorConfig.Slot1.kI = 0.1;
        leftMotorConfig.Slot1.kD = 0.001;
        leftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        leftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        climbMotorLeft.setSafetyEnabled(false); 
        climbMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig= new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Slot0.kP = 0.11;
        rightMotorConfig.Slot0.kI = 0.5;
        rightMotorConfig.Slot0.kD = 0.0001;
        rightMotorConfig.Slot0.kV = 0.12;
        rightMotorConfig.Voltage.PeakForwardVoltage = 12;
        rightMotorConfig.Voltage.PeakReverseVoltage = -12;
        rightMotorConfig.Slot1.kP = 0.5;
        rightMotorConfig.Slot1.kI = 0.1;
        rightMotorConfig.Slot1.kD = 0.001;
        rightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        climbMotorRight.setSafetyEnabled(false); 
        climbMotorRight.getConfigurator().apply(rightMotorConfig);
        setupShuffleboard(true);

    }

    // Set Climber Position
    public void setClimberPosition(double Position) {
        
    }

    public void setClimbSpeed(double speed) {
        climbMotorLeft.set(speed);
        climbMotorRight.set(speed);
    }

    public void setLeftClimbSpeed(double speed) {
        climbMotorLeft.set(speed);
    }

    public void setRightClimbSpeed(double speed) {
        climbMotorRight.set(speed);
    }

    public void stopMotors() {
        climbMotorLeft.stopMotor();
        climbMotorRight.stopMotor();
    }

    public double getClimbLeftTemp() {
        return climbMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getClimbRightTemp() {
        return climbMotorRight.getDeviceTemp().getValueAsDouble();
    }

    public double getClimbAverageTemp() {
        return (climbMotorLeft.getDeviceTemp().getValueAsDouble() +  climbMotorRight.getDeviceTemp().getValueAsDouble()) / 2;
    }


    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Climb/Average Motor Temperature", getClimbAverageTemp());
            SmartDashboard.putNumber("Climb/Left Motor Temperature", getClimbLeftTemp());
            SmartDashboard.putNumber("Climb/Right Motor Temperature", getClimbRightTemp());
        }
    }

    private void setupShuffleboard(boolean logEnable) {
        if (logEnable) {
            ShuffleboardLayout widget =  climberTab.getLayout("Climber", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(4, 0);
            widget.addNumber("Left Motor Temp", this::getClimbLeftTemp);
            widget.addNumber("Right Motor Temp", this::getClimbRightTemp);
        }
    }

    public void initialize() {
    }    

    @Override
    public void periodic() {
        super.periodic();
        log();        
        
        // setRightClimbSpeed(rightClimbVelocityFromDash.getDouble(0));
        // setLeftClimbSpeed(leftClimbVelocityFromDash.getDouble(0));
        setClimbSpeed(climbVelocityFromDash.getDouble(0));
    }

   /*  public Command setClimbSpeedLocal() {
        return this.startEnd(
            () -> setClimbSpeed(0.1), 
            () -> setClimbSpeed(0.0)
        );
    }*/
}
