package org.firstinspires.ftc.teamcode.drive.OGCode;
//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="Hulk_Smash", group="Linear Opmode")

public class Hulk_Smash extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime() , timeGetVoltage = new ElapsedTime();

    double  PrecisionDenominator=1, PrecisionDenominator2=1.75;
    double Kp4Bar = 0.008;
    double Ki4Bar = 0;
    double Kd4Bar = 0.001;
    int junctionHeight = 0;
    double outtakeStatus=0, fourBarStatus = 0, ghidajStatus = 0;

    double pozCollectGround = 35, pozCollectFirst=55, pozCollectSecond=75, pozCollectThird=95, pozCollectFourth=115,
            pozCollectFifth = 135, pozPlace=300;

    int status = 0;
    int pozStack = 1;
    public void robotCentricDrive(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim, boolean StrafesOn , double LeftTrigger,  double RightTrigger)
    {
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x*1 - LeftTrigger + RightTrigger;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
//    public void fieldCentricDrive(BNO055IMU imu,DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack , double LeftTrigger , double RightTrigger)
//    {
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x - LeftTrigger + RightTrigger;
//
//        // Read inverse IMU heading, as the IMU heading is CW positive
//        double botHeading = -imu.getAngularOrientation().firstAngle;
//
//        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
//        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        leftFront.setPower(frontLeftPower);
//        leftBack.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightBack.setPower(backRightPower);
//    }

    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();
        double loopTime = 0;

        ClawController clawController = new ClawController();
        FourBarController fourBarController = new FourBarController();
        LiftController liftController = new LiftController();
        GhidajController ghidajController = new GhidajController();
        RobotController robotController = new RobotController();
        fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
        clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
        liftController.CurrentStatus = LiftController.liftStatus.GROUND;
        ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
        //clawController.update(robot);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        DcMotor rightFront = null;
        DcMotor rightBack = null;
        DcMotor leftFront = null;
        DcMotor leftBack = null;
        SimplePIDController motor4BarPID = new SimplePIDController(0.02,0,0);
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 1 ; /// limita vitezei la sasiu
        boolean StrafesOn = false;
        int pozHigh = 1700;
        String typeOfDrive = "RobotCentric";
        SimplePIDController hello = new SimplePIDController(Kp4Bar,Ki4Bar,Kd4Bar);
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            int Motor4BarPosition = robot.motor4Bar.getCurrentPosition();
            int fourBarPosition = robot.motor4Bar.getCurrentPosition();
            int liftPosition = robot.extensieOuttake.getCurrentPosition();
            robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim,StrafesOn , 0,0);

            if (currentGamepad1.right_trigger>0)
            {
                PrecisionDenominator=2.25;
                PrecisionDenominator2=2.75;
            }
            else
            {
                PrecisionDenominator=1;
                PrecisionDenominator2=1.75;
            }

            /*if (!previousGamepad1.circle && currentGamepad1.circle)
            {
               if (outtakeStatus==0)
               {
                   liftController.CurrentStatus = LiftController.liftStatus.POLE;
                   outtakeStatus=1;
               }
               else
               {
                   liftController.CurrentStatus = LiftController.liftStatus.GROUND;
                   outtakeStatus=0;
               }

            }*/

            if (!previousGamepad1.square && currentGamepad1.square)
            {
                if (clawController.CurrentStatus == ClawController.closeClawStatus.CLOSED)
                {
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                }
                else
                {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                }

            }
            if (!previousGamepad1.cross && currentGamepad1.cross)
            {
                if (fourBarStatus==0)
                {
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.PLACE_POSITION;
                    fourBarStatus=1;
                }
                else
                {
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    fourBarStatus=0;
                }

            }
            if (!previousGamepad1.triangle && currentGamepad1.triangle)
            {
                if (ghidajStatus==0)
                {
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                    ghidajStatus=1;
                }
                else
                {
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                    ghidajStatus=0;
                }
            }

            if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
            {
                if (status==0)
                {
                    junctionHeight = 0;
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PICK_UP_CONE;
                    status=1;
                }
                else if (status==1)
                {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    status=0;
                }
            }
            if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
            {
                if (junctionHeight==0) junctionHeight=1;
                else if (junctionHeight==1) junctionHeight=2;
                else if (junctionHeight==2) junctionHeight=0;
            }
            if (!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
            {
                liftController.CurrentStatus = LiftController.liftStatus.POLE;
                pozStack = Math.min(pozStack+1,5);
                junctionHeight = pozStack + 3;
            }
            if (!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
            {
                liftController.CurrentStatus = LiftController.liftStatus.POLE;
                pozStack = Math.max(pozStack-1,1);
                junctionHeight = pozStack + 3;
            }
            fourBarController.update(robot , fourBarPosition,14 );
            clawController.update(robot);
            liftController.update(robot,junctionHeight, liftPosition, currentVoltage);
            ghidajController.update(robot);
            robotController.update(robot, fourBarController, ghidajController, liftController, clawController, junctionHeight, fourBarPosition);
            telemetry.addData("CurrentPositionOuttake", liftPosition);
            telemetry.addData("liftPower", robot.extensieOuttake.getPower());
            telemetry.addData("CurrentPosition4Bar",robot.motor4Bar.getCurrentPosition());
            telemetry.addData("Claw",robot.servoGheara.getPosition());
            telemetry.addData("clawController", clawController.CurrentStatus);
            telemetry.addData("4BarStatus" , fourBarController.CurrentStatus);
            telemetry.addData("junctionHeight", junctionHeight);


            telemetry.update();
        }
    }
}
