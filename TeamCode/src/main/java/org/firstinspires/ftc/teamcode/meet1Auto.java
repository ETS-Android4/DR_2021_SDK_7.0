package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="meet1Auto")


public class meet1Auto extends LinearOpMode
{

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();


        //Imu startUp

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //creates two variables for ramp motor speed

        double turnPowerM = 0;
        double turnPowerO = 0;


        waitForStart();

        //more imu stuff

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //drives the robot backwards at half speed

        encoderDrive(-750, -0.5);

        //turn right

        while (angles.firstAngle > -85) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPowerM = ((90 + angles.firstAngle) /90) + .1 ;

            turnPowerO = ((90 + angles.firstAngle) /90)*.75 + .1 ;

            robot.motorRB.setPower(-turnPowerO);
            robot.motorRM.setPower(-turnPowerM);
            robot.motorRF.setPower(-turnPowerO);
            robot.motorLB.setPower(turnPowerO);
            robot.motorLM.setPower(turnPowerM);
            robot.motorLF.setPower(turnPowerO);

            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }

        //move backward

        encoderDrive(-2000, -.75);

        //put code to spin ducks HERE ^^^

        //drive forward alot

        encoderDrive(3000, .75);

        while (angles.firstAngle > -135) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPowerM = ((135 + angles.firstAngle) /135) + .1 ;

            turnPowerO = ((135 + angles.firstAngle) /135)*.75 + .1 ;

            robot.motorRB.setPower(-turnPowerO);
            robot.motorRM.setPower(-turnPowerM);
            robot.motorRF.setPower(-turnPowerO);
            robot.motorLB.setPower(turnPowerO);
            robot.motorLM.setPower(turnPowerM);
            robot.motorLF.setPower(turnPowerO);

            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }

        encoderDrive(1250, .75);

        while (angles.firstAngle < -95) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPowerM = -((90 + angles.firstAngle) /90) + .1 ;

            turnPowerO = -((90 + angles.firstAngle) /90)*.75 + .1 ;

            robot.motorRB.setPower(turnPowerO);
            robot.motorRM.setPower(turnPowerM);
            robot.motorRF.setPower(turnPowerO);
            robot.motorLB.setPower(-turnPowerO);
            robot.motorLM.setPower(-turnPowerM);
            robot.motorLF.setPower(-turnPowerO);

            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }

        //place box

        encoderDrive(6000, 1);


            //robot.motorRF.setPower(gamepad1.right_stick_y * 3/4);
            //robot.motorRM.setPower(gamepad1.right_stick_y);
            //robot.motorRB.setPower(gamepad1.right_stick_y * 3/4);
            //robot.motorLB.setPower(gamepad1.left_stick_y * 3/4);
            //robot.motorLM.setPower(gamepad1.left_stick_y);
            //robot.motorLF.setPower(gamepad1.left_stick_y * 3/4);

    }

    public void encoderDrive (int distance, double speed) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLF.setTargetPosition(distance * 3/4 + robot.motorLF.getCurrentPosition());
        robot.motorLM.setTargetPosition(distance + robot.motorLM.getCurrentPosition());
        robot.motorLB.setTargetPosition(distance * 3/4 + robot.motorLB.getCurrentPosition());
        robot.motorRF.setTargetPosition(distance * 3/4 + robot.motorRF.getCurrentPosition());
        robot.motorRM.setTargetPosition(distance + robot.motorRM.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance * 3/4 + robot.motorRB.getCurrentPosition());

        robot.motorLF.setPower(speed * 3/4);
        robot.motorLM.setPower(speed);
        robot.motorLB.setPower(speed * 3/4);
        robot.motorRF.setPower(speed * 3/4);
        robot.motorRM.setPower(speed);
        robot.motorRB.setPower(speed * 3/4);

        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorRF.isBusy() && opModeIsActive())
        {
            telemetry.addData("LF encoder value", robot.motorLF.getCurrentPosition());
            telemetry.addData("LM encoder value", robot.motorLM.getCurrentPosition());
            telemetry.addData("LB encoder value", robot.motorLB.getCurrentPosition());
            telemetry.addData("RF encoder value", robot.motorRF.getCurrentPosition());
            telemetry.addData("RM encoder value", robot.motorRM.getCurrentPosition());
            telemetry.addData("RB encoder value", robot.motorRB.getCurrentPosition());
            telemetry.update();
        }

        robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }



    public void stopDrive () {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLB.setPower(0);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0);

        telemetry.addData("im here", 0);
        telemetry.update();
    }
}
