package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class GhidajController {

    public enum ghidajStatus
    {
        INIT,
        OUTTAKE,
        INTAKE,
    }
    public static ghidajStatus CurrentStatus = ghidajStatus.INIT,  PreviousStatus = ghidajStatus.INIT;
    double pozIntake = 1, pozOuttake = 0;

    public void update(RobotMap Robotel)
    {
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case INTAKE:
                {
                    Robotel.servoGhidaj.setPosition(pozIntake);
                    break;
                }
                case OUTTAKE:
                {
                    Robotel.servoGhidaj.setPosition(pozOuttake);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
