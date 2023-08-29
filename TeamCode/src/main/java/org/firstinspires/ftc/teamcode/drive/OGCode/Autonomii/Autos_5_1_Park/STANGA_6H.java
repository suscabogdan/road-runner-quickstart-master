package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.Autos_5_1_Park;

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
import java.util.concurrent.atomic.AtomicReference;


@Config
@Autonomous(group = "a")

public class STANGA_6H extends LinearOpMode {
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

    public static double x_PLACE_PRELOAD = -30, y_PLACE_PRELOAD = -5, backPreload = 45;
    public static double x_GTS_FIRST_LT2 = -65, y_GTS_FIRST_LT2 = -11;
    public static double x_PLACE_FIRST_STS = -30, y_PLACE_FIRST_STS = -5;

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
        int nr = 0, NRCON = 6, CAZ = 1;
        ElapsedTime timePLACE_PRELOAD = new ElapsedTime();
        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        AtomicReference<STROBOT> status = new AtomicReference<>(STROBOT.START);
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(backPreload)
                .splineTo(new Vector2d(x_PLACE_PRELOAD, y_PLACE_PRELOAD), Math.toRadians(40))
                .addTemporalMarker(1.25, ()->{
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                    robotController.CurrentStatus = PICK_UP_CONE;
                })
                .build(); // merge de la inceput sa puna preload-u

        TrajectorySequence GTS_FIRST = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .addSpatialMarker(new Vector2d(-60, -11), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .forward(0.1)
                .splineTo(new Vector2d(x_GTS_FIRST_LT2, y_GTS_FIRST_LT2), Math.toRadians(180))
                .build(); // merge de la junction la stack
        TrajectorySequence PLACE_FIRST = drive.trajectorySequenceBuilder(GTS_FIRST.end())
                .back(5)
                .splineTo(new Vector2d(x_PLACE_FIRST_STS, y_PLACE_FIRST_STS), Math.toRadians(45))
                .build();
        TrajectorySequence GTS_FIRST_SECOND = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .addSpatialMarker(new Vector2d(-60, -11), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .forward(1)
                .splineTo(new Vector2d(x_GTS_FIRST_LT2, y_GTS_FIRST_LT2), Math.toRadians(180))
                .build(); // merge de la junction la stack

        TrajectorySequence PARK_1 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .waitSeconds(1)
                .forward(1)
                .splineTo(new Vector2d(-60, y_GTS_FIRST_LT2), Math.toRadians(180))
                .build();
        TrajectorySequence PARK_2 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .waitSeconds(1)
                .back(0.5)
                .lineToLinearHeading(new Pose2d(-35,-15,Math.toRadians(270)))
                .build();
        TrajectorySequence PARK_3 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
                .waitSeconds(1)
                .back(0.5)
                .lineToLinearHeading(new Pose2d(-10,-15,Math.toRadians(270)))
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
            if (status.get() == STROBOT.START)
            {
                drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                status.set(STROBOT.PLACE_PRELOAD);
            }
            else
            if (status.get() == STROBOT.PLACE_PRELOAD)
            {
                if (!drive.isBusy())
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    TIMERGLOBAL.reset();
                    status.set(STROBOT.GO_TO_STACK);
                }
            }
            else
            if (status.get() == STROBOT.GO_TO_STACK)
            {
                if (NRCON==1)
                {
                    status.set(STROBOT.PARK);
                }
                else
                {
                    if (TIMERGLOBAL.seconds()>0.75)
                    {
                        TIMERGLOBAL.reset();
                        if (NRCON==6)
                        {
                            drive.followTrajectorySequenceAsync(GTS_FIRST);
                        }
                        else
                        {
                            drive.followTrajectorySequenceAsync(GTS_FIRST_SECOND);
                        }
                        status.set(STROBOT.COLLECT);
                    }
                }
            }
            else
            if (status.get() == STROBOT.COLLECT)
            {
                if (TIMERGLOBAL.seconds()>0.75)
                {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = NRCON+2;
                }
                if (!drive.isBusy())
                {
                    status.set(STROBOT.GO_PLACE_FROM_STACK);
                    if(TIMERGLOBAL.seconds() > 2.25)
                    {
                        robotController.CurrentStatus = RobotController.RobotControllerStatus.INTER_PICK_UP_STACK;
                    }
                }
            }
            else
            if (status.get() == STROBOT.GO_PLACE_FROM_STACK)
            {
                if (robotController.CurrentStatus == RobotController.RobotControllerStatus.START)
                {
                    TIMERGLOBAL.reset();
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                    drive.followTrajectorySequenceAsync(PLACE_FIRST);
                    status.set(STROBOT.PLACE_STACK_CONE);
                }
            }
            else
            if (status.get() == STROBOT.PLACE_STACK_CONE)
            {
                if (TIMERGLOBAL.seconds()>1)
                {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                }
                if (TIMERGLOBAL.seconds()>0.75)
                {
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                }
                if (!drive.isBusy() && TIMERGLOBAL.seconds() > 1)
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    TIMERGLOBAL.reset();
                    NRCON--;
                    status.set(STROBOT.GO_TO_STACK);
                }
            }
            else
            if (status.get() == STROBOT.PARK)
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
                status.set(STROBOT.STOP_JOC);
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