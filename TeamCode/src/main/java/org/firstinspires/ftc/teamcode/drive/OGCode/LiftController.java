package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.Current;
@Config
public class LiftController {

    public enum liftStatus
    {
        INIT,
        GROUND,
        POLE
    }
    public static liftStatus CurrentStatus = liftStatus.INIT,  PreviousStatus = liftStatus.INIT;
    int pozHigh = 1700, pozMid = 900, pozLow = 500;
    public static int pozSecondCone = 150, pozThirdCone = 250 , pozFourthCone = 350 ,pozFifthCone = 450;
    double Kp4Bar = 0.008;
    double Ki4Bar = 0.0001;
    double Kd4Bar = 0.0001;
    double basePosition = 0;
    SimplePIDController LiftPID;
    public double CurrentPosition = 0;
    public LiftController()
    {
        LiftPID = new SimplePIDController(Kp4Bar,Ki4Bar,Kd4Bar);
        LiftPID.targetValue=basePosition;
        LiftPID.maxOutput = 1;
    }

    public void update(RobotMap Robotel, int junctionHeight, int currentPose, double CurrentVoltage)
    {
        CurrentPosition=currentPose;
        double powerLift = LiftPID.update(currentPose);
        powerLift = Math.max(-1,Math.min(powerLift* 14 / CurrentVoltage,1));
        Robotel.extensieOuttake.setPower(powerLift);
        switch (CurrentStatus)
        {
                case GROUND:
                {
                   LiftPID.targetValue=basePosition;
                    break;
                }
                case POLE:
                {
                    if (junctionHeight == 0) LiftPID.targetValue=pozHigh;
                    else if (junctionHeight == 1) LiftPID.targetValue=pozMid;
                    else if (junctionHeight == 2) LiftPID.targetValue=0;


                    if (junctionHeight == 4)
                    {
                        LiftPID.targetValue=0;
                    }
                    if (junctionHeight == 5)
                    {
                        LiftPID.targetValue = pozSecondCone;
                    }
                    if (junctionHeight == 6)
                    {
                        LiftPID.targetValue = pozThirdCone;
                    }
                    if (junctionHeight == 7)
                    {
                        LiftPID.targetValue = pozFourthCone;
                    }
                    if (junctionHeight == 8)
                    {
                        LiftPID.targetValue = pozFifthCone;
                    }
                    break;

                }
        }
    }
}
