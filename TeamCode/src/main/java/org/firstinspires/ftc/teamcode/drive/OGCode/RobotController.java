package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.DRIVE_POSITION;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_COLLECT;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE_FIRST_CONE_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE_STACK;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_PICK_UP_STACK;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_PLACE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.PLACE_INTER;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.START;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Current;

public class RobotController {

    public enum RobotControllerStatus
    {
        START,
        PICK_UP_CONE,
        INTER_COLLECT,
        PLACE,
        INTER_PLACE,
        DRIVE_POSITION,
        PLACE_INTER,
        INTER_PICK_UP_STACK,
        PICK_UP_STACK,
        GO_PLACE_STACK,
        INTER_GO_PLACE_STACK,
        GO_PLACE_FIRST_CONE_AUTO,
        INTER_GO_PLACE_FIRST_CONE_AUTO,
    }

    public double timerTransferColectare = 0.25, timerTransferOuttake=1, timerPlaceCone=1.4, timerGhidaj=3, timerPlace=0.4;
    public static RobotControllerStatus CurrentStatus = START, PreviousStatus = START;
    ElapsedTime timerPickUpCone = new ElapsedTime() , timerGO_PLACE = new ElapsedTime() ,TimerGO_PLACE2 = new ElapsedTime();
    public void update(RobotMap Robot, FourBarController fourBarController, GhidajController ghidajController, LiftController liftController, ClawController clawController, int junctionHeight, int fourBarPosition)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus == INTER_COLLECT || CurrentStatus == DRIVE_POSITION || CurrentStatus == PLACE_INTER || CurrentStatus == INTER_PICK_UP_STACK)
        {
            switch (CurrentStatus)
            {

                case PICK_UP_CONE:
                {
                    timerPickUpCone.reset();
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    CurrentStatus = INTER_COLLECT;
                    break;
                }

                case INTER_COLLECT:
                {
                    if (timerPickUpCone.seconds()>timerTransferColectare)
                    {
                        liftController.CurrentStatus = LiftController.liftStatus.POLE;
                        fourBarController.CurrentStatus = FourBarController.fourBarStatus.INTER_POSITION;
                        ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                        CurrentStatus = START;
                    }
                    break;
                }
                case PICK_UP_STACK:
                {
                    timerPickUpCone.reset();
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    CurrentStatus = INTER_PICK_UP_STACK;
                    break;
                }

                case INTER_PICK_UP_STACK:
                {
                    if (timerPickUpCone.seconds()>timerTransferColectare)
                    {
                        fourBarController.CurrentStatus = FourBarController.fourBarStatus.INTER_POSITION;
                        CurrentStatus = START;
                    }
                    break;
                }
                case PLACE:
                {
                    timerGO_PLACE.reset();
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.PLACE_POSITION;
                    CurrentStatus = PLACE_INTER;
                    break;
                }
                case PLACE_INTER:
                {

                    if (fourBarPosition>1150)
                    {
                        clawController.CurrentStatus = ClawController.closeClawStatus.OPEN_INTERMEDIARY;
                        ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                        timerGO_PLACE.reset();
                        CurrentStatus = DRIVE_POSITION;
                    }

                    break;
                }
                case DRIVE_POSITION:
                {
                    if (timerGO_PLACE.seconds()>0.1)
                    {
                        fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    }
                    if (timerGO_PLACE.seconds()>0.5)
                    {
                        liftController.CurrentStatus = LiftController.liftStatus.GROUND;
                        clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                        CurrentStatus = START;
                    }
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
