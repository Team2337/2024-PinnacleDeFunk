package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class ClimberPosition extends SubsystemBase {
    private TalonFX climbMotorLeft = new TalonFX(54, "Upper"); 
    private TalonFX climbMotorRight = new TalonFX(44, "Upper"); 
    private final PositionVoltage voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    CommandXboxController operatorJoystick;

    double climberMaxSetPoint = -70;
    double climberMinSetPoint = 0;

    public ClimberPosition(CommandXboxController operatorJoystick) {
        this.operatorJoystick = operatorJoystick;
        
        var setClimbMotorToDefault = new TalonFXConfiguration();
        climbMotorLeft.getConfigurator().apply(setClimbMotorToDefault);
        climbMotorRight.getConfigurator().apply(setClimbMotorToDefault);
        
        TalonFXConfiguration climbMotorLeftConfig = new TalonFXConfiguration();
        climbMotorLeftConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        climbMotorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotorLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorLeftConfig.Slot0.kP = 1;
        climbMotorLeftConfig.Slot0.kD = 0;
        climbMotorLeftConfig.Voltage.PeakForwardVoltage = 12;
        climbMotorLeftConfig.Voltage.PeakReverseVoltage = -12;
        

        climbMotorLeft.setSafetyEnabled(false);
        climbMotorLeft.getConfigurator().apply(climbMotorLeftConfig);
        climbMotorLeft.setPosition(0);

        TalonFXConfiguration climbMotorRightConfig = new TalonFXConfiguration();
        climbMotorRightConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        climbMotorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        climbMotorRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climbMotorRightConfig.Slot0.kP = 1;
        climbMotorRightConfig.Slot0.kD = 0;
        climbMotorRightConfig.Voltage.PeakForwardVoltage = 12;
        climbMotorRightConfig.Voltage.PeakReverseVoltage = -12;

        
        climbMotorRight.getConfigurator().apply(climbMotorRightConfig);
        
        climbMotorLeft.setPosition(0);
        //climbMotorRight.setControl(new Follower(climbMotorLeft.getDeviceID(), true));
    }

    public void setClimberSetpoint(double setPoint) {
            if (setPoint > climberMinSetPoint) {
                setPoint = climberMinSetPoint;
            } else if (setPoint < climberMaxSetPoint) {
                setPoint = climberMaxSetPoint;
            }
            //this.setSetpoint(setPoint);
    }
    
    public void setClimbSpeed(double speed) {
        climbMotorLeft.set(speed);
        //climbMotorRight.set(speed);
    }

    public void stopMotors() {
        climbMotorLeft.stopMotor();
        climbMotorRight.stopMotor();
    }

    public double getClimbTemp() {
        return climbMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getClimberPosition() {
        return climbMotorLeft.getPosition().getValueAsDouble();
    }

    public void setClimberPosition(double rotations) {
        climbMotorLeft.setControl(voltagePosition.withPosition(rotations));
        climbMotorRight.setControl(voltagePosition.withPosition(rotations));
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
        }
        if (Constants.DashboardLogging.TEMP) {
            SmartDashboard.putNumber("Temps/Climber Temp", getClimbTemp() );
        }
        SmartDashboard.putNumber("Climber/Get Climber Position", getClimberPosition());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }
}
