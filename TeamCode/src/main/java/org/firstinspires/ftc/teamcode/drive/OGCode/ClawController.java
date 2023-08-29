package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class ClawController {

    public enum closeClawStatus
    {
        INIT,
        CLOSED,
        OPEN,
        OPEN_INTERMEDIARY
    }
    public static closeClawStatus CurrentStatus = closeClawStatus.INIT,  PreviousStatus = closeClawStatus.INIT;
    double pozOpenClaw = 0.8, pozIntermediary=0.75, pozCloseClaw = 0.5;

    public void update(RobotMap Robotel)
    {
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case CLOSED:
                {
                    Robotel.servoGheara.setPosition(pozCloseClaw);
                    break;
                }
                case OPEN_INTERMEDIARY:
                {
                    Robotel.servoGheara.setPosition(pozIntermediary);
                    break;
                }
                case OPEN:
                {
                    Robotel.servoGheara.setPosition(pozOpenClaw);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
