package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Climber extends PIDSubsystem {
    private TalonFX climbMotorLeft = new TalonFX(54); 
    private TalonFX climbMotorRight = new TalonFX(55); 
    private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          0.1, 0.1);
    
    AnalogInput input = new AnalogInput(0);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 180, 30);

    private double position;
    private PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

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
    
    private GenericEntry climbPositionFromDash = climberTab
            .add("Climber Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            // .withProperties(Map.of("Min", -1, "Max", 1))
            .getEntry();

    public Climber() {
        super(new PIDController(0.1, 0.0, 0.0001));
        getController().setTolerance(2.0);
        setSetpoint(pot.get());
        enable();

        // input.setAverageBits(2);

        var setClimbMotorLeftToDefault = new TalonFXConfiguration();
        climbMotorLeft.getConfigurator().apply(setClimbMotorLeftToDefault);

        var setClimbMotorRightToDefault = new TalonFXConfiguration();
        climbMotorRight.getConfigurator().apply(setClimbMotorRightToDefault);
        
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Slot0.kP = 10;
        leftMotorConfig.Slot0.kI = 0.0;
        leftMotorConfig.Slot0.kD = 0.0001;
        // leftMotorConfig.Slot0.kV = 0.12;
        leftMotorConfig.Voltage.PeakForwardVoltage = 8;
        leftMotorConfig.Voltage.PeakReverseVoltage = -8;
        leftMotorConfig.Slot1.kP = 0.5;
        leftMotorConfig.Slot1.kI = 0.1;
        leftMotorConfig.Slot1.kD = 0.001;
        leftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        leftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // leftMotorConfig.DifferentialSensors.DifferentialRemoteSensorID = 0;
        // leftMotorConfig.DifferentialSensors.DifferentialSensorSource.RemoteCANcoder
        climbMotorLeft.setSafetyEnabled(false);
        climbMotorLeft.getConfigurator().apply(leftMotorConfig);
/* 
         TalonFXConfiguration config = new TalonFXConfiguration();
        config.sum0Term = FeedbackDevice.RemoteSensor0;
        config.diff0Term = FeedbackDevice.RemoteSensor0;
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        config.remoteFilter0.remoteSensorDeviceID = cancoder.getDeviceID();

        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
         */
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Slot0.kP = 10;
        rightMotorConfig.Slot0.kI = 0.0;
        rightMotorConfig.Slot0.kD = 0.0001;
        // rightMotorConfig.Slot0.kV = 0.12;
        rightMotorConfig.Voltage.PeakForwardVoltage = 8;
        rightMotorConfig.Voltage.PeakReverseVoltage = -8;
        rightMotorConfig.Slot1.kP = 0.5;
        rightMotorConfig.Slot1.kI = 0.1;
        rightMotorConfig.Slot1.kD = 0.001;
        rightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        climbMotorRight.setSafetyEnabled(false);
        climbMotorRight.getConfigurator().apply(rightMotorConfig);
        setupShuffleboard(false);
        
    }

    public void setClimberSetpoint(double position) {
        this.setSetpoint(position);
    }

    // Set Climber Position
    public void setClimberPosition(double position) {
        this.position = position;
        climbMotorLeft.setControl(positionRequest.withPosition(position));
        climbMotorRight.setControl(positionRequest.withPosition(position));
    }

    public double getClimberLeftPosition() {
        return climbMotorLeft.getPosition().getValueAsDouble();

    }

    public double getClimberRightPosition() {
        return climbMotorRight.getPosition().getValueAsDouble();
    }

    public double getClimberPostition() {
        return pot.get();
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
        return (climbMotorLeft.getDeviceTemp().getValueAsDouble() + climbMotorRight.getDeviceTemp().getValueAsDouble())
                / 2;
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Climb/Average Motor Temperature", getClimbAverageTemp());
            SmartDashboard.putNumber("Climb/Left Motor Temperature", getClimbLeftTemp());
            SmartDashboard.putNumber("Climb/Right Motor Temperature", getClimbRightTemp());
            SmartDashboard.putNumber("Climb Position", pot.get());
            SmartDashboard.putNumber("Climb Set Point", getSetpoint());
            SmartDashboard.putBoolean("Climb at Set Point", getController().atSetpoint());
            
        }
    }

    private void setupShuffleboard(boolean logEnable) {
        if (logEnable) {
            ShuffleboardLayout widget = climberTab.getLayout("Climber", BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withPosition(4, 0);
            widget.addNumber("Left Motor Temp", this::getClimbLeftTemp);
            widget.addNumber("Right Motor Temp", this::getClimbRightTemp);
        }
    }

    public void initialize() {
        setSetpoint(pot.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (output > 0.75) {
            output  = 0.75;
        } else if (output < -0.75) {
            output = -0.75;
        }
        // climbMotorLeft.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
        climbMotorLeft.set(output);
        SmartDashboard.putNumber("Climb Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}
