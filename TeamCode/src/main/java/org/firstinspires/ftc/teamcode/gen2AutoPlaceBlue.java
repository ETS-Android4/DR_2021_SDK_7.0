package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="gen2AutoPlaceBlue")
@Disabled


public class gen2AutoPlaceBlue extends LinearOpMode
{
    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public  DcMotor duckSpinnerRight = null;
    public Servo baseRight = null;
    public Servo  armRight = null;
    public Servo  bucketRight = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        ElapsedTime servoTime = new ElapsedTime();

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
        duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
        baseRight = hardwareMap.servo.get("baseRight");
        armRight = hardwareMap.servo.get("armRight");
        bucketRight = hardwareMap.servo.get("bucketRight");

        DemoBotDriveMecanum drive = new DemoBotDriveMecanum();
        DcMotor[] motors = new DcMotor[4];
        {
            motors[0] = robot.motorLF;
            motors[1] = robot.motorLB;
            motors[2] = robot.motorRF;
            motors[3] = robot.motorRB;

        }

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        //freeze the bucket
        bucketRight.setPosition(.17);

        armRight.setPosition(.1);

        //move to hub
        drive.encoderDrive(300, driveStyle.STRAFE_RIGHT, 1, motors);

        //raise arm and place
        baseRight.setPosition(.6);

        sleep(1000);

        armRight.setPosition(.55);

        sleep(1000);

        //bring arm down
        armRight.setPosition(.1);

        sleep(1000);

        baseRight.setPosition(.23);

        sleep(1000);

        armRight.setPosition(0);

        //drive to wall
        drive.encoderDrive(300, driveStyle.STRAFE_LEFT, 1, motors);

        //park
        drive.encoderDrive(3750, driveStyle.FORWARD, 1, motors);

        //drive.timeDrive(100,1,driveStyle.FORWARD,motors);

        //sleep(10000);


    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
