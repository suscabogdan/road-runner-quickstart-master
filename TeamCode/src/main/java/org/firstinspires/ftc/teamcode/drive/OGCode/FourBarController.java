package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class FourBarController {

    public enum fourBarStatus
    {
        INIT,
        PLACE_POSITION,
        COLLECT_DRIVE,
        COLLECT_STACK,
        INTER_POSITION
    }
    public static fourBarStatus CurrentStatus = fourBarStatus.INIT,  PreviousStatus = fourBarStatus.INIT;
    double pozPlace = 1200, pozCollect = 0, pozSecondCone = 55, pozThirdCone=75, pozFourthCone = 95, pozFifthCone=115, pozInter=900;
    double Kp4Bar = 0.0018;
    double Ki4Bar = 0;
    double Kd4Bar = 0.0001;
   double basePosition = 0;
    SimplePIDController FourBarPID;
    public double CurrentPosition = 0;
    public FourBarController()
    {
        FourBarPID = new SimplePIDController(Kp4Bar,Ki4Bar,Kd4Bar);
        FourBarPID.targetValue=basePosition;
        FourBarPID.maxOutput = 1;
    }
    public void update(RobotMap Robotel, double currentPose, double CurrentVoltage)
    {
        CurrentPosition=currentPose;
        double powerLift = FourBarPID.update(currentPose);
        powerLift = Math.max(-1,Math.min(powerLift* 14 / CurrentVoltage,1));
        Robotel.motor4Bar.setPower(powerLift);
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case INTER_POSITION:
                {
                    FourBarPID.targetValue = pozInter;

                    break;
                }
                case COLLECT_DRIVE:
                {
                    FourBarPID.targetValue=pozCollect;
                    break;
                }
                case PLACE_POSITION:
                {
                    FourBarPID.targetValue=pozPlace;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
