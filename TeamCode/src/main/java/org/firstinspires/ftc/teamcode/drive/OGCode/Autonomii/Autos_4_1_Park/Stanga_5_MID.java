package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.Autos_4_1_Park;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.PICK_UP_CONE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.ClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.FourBarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.GhidajController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


@Config
//@Autonomous(group = "b")

public class Stanga_5_MID extends LinearOpMode {
    enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICK_UP_CONE,
        GO_TO_STACK,
        COLLECT,
        GO_PLACE_FROM_STACK,
        STOP_JOC,
        PLACE_STACK_CONE,
        PARK
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166; //m

    // Tag IDs of sleeve
    int left = 5;
    int middle = 8;
    int right = 19;

    public static double x_PLACE_PRELOAD = -27.5, y_PLACE_PRELOAD = -5, Angle_PLACE_PRELOAD = 240, backPreload=43;
    public static double x_GTS_FIRST_LT1 = -31, y_GTS_FIRST_LT1 = -7.5,
            x_GTS_FIRST_STS=-40, y_GTS_FIRST_STS=-14, Angle_GTS_FIRST=180,
            x_GTS_FIRST_LT2 = -64.5, y_GTS_FIRST_LT2 = -14;
    public static double x_GTS_FIRST_SECOND_LT1 = -31, y_GTS_FIRST_SECOND_LT1 = -18;
    public static double x_PLACE_FIRST_LT1 = -40, y_PLACE_FIRST_LT1 =-14,
            x_PLACE_FIRST_STS = -29, y_PLACE_FIRST_STS = -19, angle_PLACE_FIRST_STS = 155;
    int junctionHeight = 0;
    ElapsedTime TIMERGLOBAL = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime() , timeCollect = new ElapsedTime();

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ClawController clawController = new ClawController();
        FourBarController fourBarController = new FourBarController();
        LiftController liftController = new LiftController();
        GhidajController ghidajController = new GhidajController();
        RobotController robotController = new RobotController();
        fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
        clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
        liftController.CurrentStatus = LiftController.liftStatus.GROUND;
        ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
        robot.servoGheara.setPosition(0.5);
        int nr=0,NRCON = 5, CAZ = 1;
        ElapsedTime timePLACE_PRELOAD = new ElapsedTime();
        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(backPreload)
                .addTemporalMarker(1.5, ()->{
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                    robotController.CurrentStatus = PICK_UP_CONE;
                })
                .splineToSplineHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)),Math.toRadians(45))
                .build();
        TrajectorySequence GTS_FIRST = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineTo(new Vector2d(x_GTS_FIRST_LT1, y_GTS_FIRST_LT1))
                .splineToSplineHeading(new Pose2d(x_GTS_FIRST_STS, y_GTS_FIRST_STS, Math.toRadians(Angle_GTS_FIRST)), Math.toRadians(Angle_GTS_FIRST))
                .lineTo(new Vector2d(x_GTS_FIRST_LT2,y_GTS_FIRST_LT2))
                .build();
        TrajectorySequence PLACE_FIRST = drive.trajectorySequenceBuilder(GTS_FIRST.end())
                .lineTo(new Vector2d(x_PLACE_FIRST_LT1, y_PLACE_FIRST_LT1))
                .splineToSplineHeading(new Pose2d(x_PLACE_FIRST_STS, y_PLACE_FIRST_STS, Math.toRadians(angle_PLACE_FIRST_STS)), Math.toRadians(345))
                .build();
        TrajectorySequence GTS_FIRST_SECOND = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .lineTo(new Vector2d(x_GTS_FIRST_SECOND_LT1, y_GTS_FIRST_SECOND_LT1))
                .splineToSplineHeading(new Pose2d(x_GTS_FIRST_STS, y_GTS_FIRST_STS, Math.toRadians(Angle_GTS_FIRST)), Math.toRadians(Angle_GTS_FIRST))
                .lineTo(new Vector2d(x_GTS_FIRST_LT2,y_GTS_FIRST_LT2))
                .build();
        TrajectorySequence PARK_1 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .lineTo(new Vector2d(x_GTS_FIRST_LT1, y_GTS_FIRST_LT1))
                .splineToSplineHeading(new Pose2d(x_GTS_FIRST_STS, y_GTS_FIRST_STS, Math.toRadians(Angle_GTS_FIRST)), Math.toRadians(Angle_GTS_FIRST))
                .lineTo(new Vector2d(x_GTS_FIRST_LT2,y_GTS_FIRST_LT2))
                .build();
        TrajectorySequence PARK_2 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .lineToLinearHeading(new Pose2d(-38.5,-10,Math.toRadians(90)))
                .build();
        TrajectorySequence PARK_3 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .lineToLinearHeading(new Pose2d(-15,-10,Math.toRadians(90)))
                .build();
        while (!isStarted()&&!isStopRequested())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    if(tagOfInterest.id == left)
                    {
                        telemetry.addLine("Sleeve detected! Case: 1");
                    }
                    if(tagOfInterest.id == middle)
                    {
                        telemetry.addLine("Sleeve detected! Case: 2");
                    }
                    if(tagOfInterest.id == right)
                    {
                        telemetry.addLine("Sleeve detected! Case: 3");
                    }
                }
                else
                {
                    telemetry.addLine("Sleeve not in sight!");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("Sleeve never detected. Default case is: 2");
                    }
                    else
                    {
                        if(tagOfInterest.id == left)
                        {
                            telemetry.addLine("Last detection case was: 1");
                        }
                        if(tagOfInterest.id == middle)
                        {
                            telemetry.addLine("Last detection case was: 2");
                        }
                        if(tagOfInterest.id == right)
                        {
                            telemetry.addLine("Last detection case was: 3");
                        }
                    }
                }

            }
            else
            {
                telemetry.addLine("Sleeve not in sight!");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("Sleeve never detected. Default case is: 2");
                }
                else
                {
                    if(tagOfInterest.id == left)
                    {
                        telemetry.addLine("Last detection case was: 1");
                    }
                    if(tagOfInterest.id == middle)
                    {
                        telemetry.addLine("Last detection case was: 2");
                    }
                    if(tagOfInterest.id == right)
                    {
                        telemetry.addLine("Last detection case was: 3");
                    }
                }

            }

            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            if (status == STROBOT.START)
            {
                drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                status = STROBOT.PLACE_PRELOAD;
            }
            else
            if (status == STROBOT.PLACE_PRELOAD)
            {
                if (!drive.isBusy())
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    TIMERGLOBAL.reset();
                    status = STROBOT.GO_TO_STACK;
                }
            }
            else
            if (status == STROBOT.GO_TO_STACK)
            {
                if (NRCON==1)
                {
                    status = STROBOT.PARK;
                }
                else
                {
                    if (TIMERGLOBAL.seconds()>0.75)
                    {
                        TIMERGLOBAL.reset();
                        if (NRCON==5)
                        {
                            drive.followTrajectorySequenceAsync(GTS_FIRST);
                        }
                        else
                        {
                            drive.followTrajectorySequenceAsync(GTS_FIRST_SECOND);
                        }
                        status = STROBOT.COLLECT;
                    }
                }
            }
            else
            if (status == STROBOT.COLLECT)
            {
                if (TIMERGLOBAL.seconds()>0.75)
                {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = NRCON+3;
                }
                if (!drive.isBusy())
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PICK_UP_STACK;
                    status = STROBOT.GO_PLACE_FROM_STACK;
                }
            }
            else
            if (status == STROBOT.GO_PLACE_FROM_STACK)
            {
                if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START)
                {
                    TIMERGLOBAL.reset();
                    drive.followTrajectorySequenceAsync(PLACE_FIRST);
                    status = STROBOT.PLACE_STACK_CONE;
                }
            }
            else
            if (status == STROBOT.PLACE_STACK_CONE)
            {
                if (TIMERGLOBAL.seconds()>1.2)
                {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 1;
                }
                if (TIMERGLOBAL.seconds()>1.75)
                {
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                }
                if (!drive.isBusy() && TIMERGLOBAL.seconds() > 2.25)
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    TIMERGLOBAL.reset();
                    NRCON--;
                    status = STROBOT.GO_TO_STACK;
                }
            }
            else
            if (status == STROBOT.PARK)
            {
                if(tagOfInterest == null)
                {
                    CAZ = 2;
                } else {
                    if(tagOfInterest.id == left)
                    {
                        CAZ = 1;
                    } else
                    if(tagOfInterest.id == middle)
                    {
                        CAZ = 2;
                    }
                    if(tagOfInterest.id == right)
                    {
                        CAZ = 3;
                    }
                }
                if (CAZ == 1)
                {
                    drive.followTrajectorySequenceAsync(PARK_1);
                }
                else
                if (CAZ == 2)
                {
                    drive.followTrajectorySequenceAsync(PARK_2);
                }
                else
                if (CAZ == 3)
                {
                    drive.followTrajectorySequenceAsync(PARK_3);
                }
                status = STROBOT.STOP_JOC;
            }
            int fourBarPosition = robot.motor4Bar.getCurrentPosition();
            int liftPosition = robot.extensieOuttake.getCurrentPosition();
            fourBarController.update(robot , fourBarPosition,14 );
            clawController.update(robot);
            liftController.update(robot,junctionHeight, liftPosition, currentVoltage);
            ghidajController.update(robot);
            robotController.update(robot, fourBarController, ghidajController, liftController, clawController, junctionHeight, fourBarPosition);
            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}