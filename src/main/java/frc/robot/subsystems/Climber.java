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
        .withProperties(Map.of("Min", -5, "Max", 5))
        .getEntry();

    private GenericEntry rightClimbVelocityFromDash = climberTab
        .add("Right Climber Velocity", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -5, "Max", 5))
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
        climbMotorLeft.getConfigurator().apply(leftMotorConfig);
        climbMotorRight.setSafetyEnabled(false);

        TalonFXConfiguration rightMotorConfig= new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorRight.getConfigurator().apply(rightMotorConfig);
        climbMotorRight.setSafetyEnabled(false);
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

    // private void setupShuffleboard(boolean logEnable) {
    //     if (logEnable) {
    //         ShuffleboardLayout widClimbTab.getLayout("Diagnositics", BuiltInLayouts.kList)
    //         .withSize(2, 2)
    //         .withPosition(4, 0);
    //         widget.addNumber("Left Motor Temp", this::getIntakeLeftTemp);
    //         widget.addNumber("Right Motor Temp", this::getIntakeRightTemp);
    //     }
    // }

    public void initialize() {
    }    

    @Override
    public void periodic() {
        super.periodic();
        log();        
        setRightClimbSpeed(rightClimbVelocityFromDash.getDouble(0));
        setLeftClimbSpeed(leftClimbVelocityFromDash.getDouble(0));
    }

   /*  public Command setClimbSpeedLocal() {
        return this.startEnd(
            () -> setClimbSpeed(0.1), 
            () -> setClimbSpeed(0.0)
        );
    }*/
}
