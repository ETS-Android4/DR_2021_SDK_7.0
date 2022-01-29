package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="gen2AutoDuckRed")
@Disabled


public class gen2AutoDuckRed extends LinearOpMode
{
    BNO055IMU imu;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public  DcMotor duckSpinnerRight = null;
    public Servo baseRight = null;
    public Servo  armRight = null;
    public Servo  bucketRight = null;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
        duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
        baseRight = hardwareMap.servo.get("baseRight");
        armRight = hardwareMap.servo.get("armRight");
        bucketRight = hardwareMap.servo.get("bucketRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DemoBotDriveMecanum drive = new DemoBotDriveMecanum();
        //meetOneRedRightWithIMU turn = new meetOneRedRightWithIMU();

        DcMotor[] motors = new DcMotor[4];
        {
            motors[0] = robot.motorLF;
            motors[1] = robot.motorLB;
            motors[2] = robot.motorRF;
            motors[3] = robot.motorRB;

        }

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        bucketRight.setPosition(.17);

        robot.motorLF.setPower(.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(.75);

        sleep(500);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(750);

        betterPivot(90);

        sleep(750);

        robot.motorLF.setPower(.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(.75);

        sleep(1500);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(750);

        duckSpinnerLeft.setPower(.5);
        duckSpinnerRight.setPower(.5);

        robot.motorLF.setPower(.4);
        robot.motorLB.setPower(.4);
        robot.motorRF.setPower(.4);
        robot.motorRB.setPower(.4);

        sleep(750);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(4500);

        duckSpinnerLeft.setPower(0);
        duckSpinnerRight.setPower(0);

        sleep(750);

        //robot.motorLF.setPower(.75);
        //robot.motorLB.setPower(-.75);
        //robot.motorRF.setPower(-.75);
        //robot.motorRB.setPower(.75);
//
        //sleep(200);
//
        //robot.motorLF.setPower(0);
        //robot.motorLB.setPower(0);
        //robot.motorRF.setPower(0);
        //robot.motorRB.setPower(0);

        drive.encoderDrive(2150, driveStyle.BACKWARD, 1, motors);

        sleep(750);

        robot.motorLF.setPower(-.75);
        robot.motorLB.setPower(.75);
        robot.motorRF.setPower(.75);
        robot.motorRB.setPower(-.75);

        sleep(400);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(750);

        betterPivot(180);

        sleep(750);

        armRight.setPosition(.1);

        sleep(750);

        robot.motorLF.setPower(-.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(-.75);

        sleep(1000);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(750);

        baseRight.setPosition(.6);

        sleep(750);

        drive.encoderDrive(2500, driveStyle.FORWARD, 1, motors);

        sleep(750);

        armRight.setPosition(.55);

        sleep(1250);

        armRight.setPosition(.1);

        robot.motorLF.setPower(-.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(-.75);

        sleep(500);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);

        sleep(750);

        baseRight.setPosition(.23);

        sleep(750);

        robot.motorLF.setPower(-.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(-.75);

        sleep(2000);

        robot.motorLF.setPower(.75);
        robot.motorLB.setPower(-.75);
        robot.motorRF.setPower(-.75);
        robot.motorRB.setPower(.75);

        sleep(1200);

        robot.motorLF.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);
    }

    public void betterPivot(int angle)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);


        double I = 0;
        double turnPower;


        while (angle > 180)
        {
            angle -= 360;
        }
        while (angle < -180)
        {
            angle += 360;
        }

        while (I == 0)
        {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while ( angles.firstAngle < angle + 10 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((angle - angles.firstAngle) /angle)+ .2;

                robot.motorRF.setPower(turnPower);
                robot.motorRB.setPower(turnPower);
                robot.motorLB.setPower(-turnPower);
                robot.motorLF.setPower(-turnPower);

                telemetry.addData("Left", I);
                telemetry.addData("current angle" , angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();

                if(angles.firstAngle > angle - 2 && I == 0)
                {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    robot.motorRF.setPower(0);
                    robot.motorRB.setPower(0);
                    robot.motorLB.setPower(0);
                    robot.motorLF.setPower(0);

                    I = 3;

                    telemetry.addData("Dead", I);
                    telemetry.addData("current angle" ,angles.firstAngle);
                    telemetry.addData("target angle" , angle);
                    telemetry.update();
                }
            }

            while(angles.firstAngle > angle -  10 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((angle - angles.firstAngle) /angle)+ .2;

                robot.motorRF.setPower(-turnPower);
                robot.motorRB.setPower(-turnPower);
                robot.motorLB.setPower(turnPower);
                robot.motorLF.setPower(turnPower);

                telemetry.addData("right", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();

                if(angles.firstAngle < angle + 2 && I == 0)
                {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    robot.motorRF.setPower(0);
                    robot.motorRB.setPower(0);
                    robot.motorLB.setPower(0);
                    robot.motorLF.setPower(0);

                    I = 3;

                    telemetry.addData("Dead", I);
                    telemetry.addData("current angle" ,angles.firstAngle);
                    telemetry.addData("target angle" , angle);
                    telemetry.update();
                }
            }


        }


    }

}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
