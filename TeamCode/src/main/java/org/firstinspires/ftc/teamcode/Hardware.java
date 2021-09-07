package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Hardware {

    //drive motors
    public DcMotor DM1;
    public DcMotor DM2;
    public DcMotor DM3;
    public DcMotor DM4;

    //odometry encoders
    public DcMotor En1;
    public DcMotor En2;
    public DcMotor En3;

    //example for pid control
    public DcMotorEx PIDC;

    //servos!
    //public CRServo name;
    //public Servo name;

    //webcam
    public WebcamName webcam;
    public OpenCvCamera camera;

    //variables
    public double[] GlobalPos; // x-0 y-1 a-2
    public double[] PreviousE; // 1-0 2-1 3-2
    public double lastError = 0;
    public double sumofErrors = 0;

    HardwareMap hardwareMap;

    public Hardware() {

    }

    public void init(HardwareMap hardwareMap) {

        //drive motors
        DM1 = this.hardwareMap.get(DcMotor.class, "DM1");
        DM2 = this.hardwareMap.get(DcMotor.class, "DM2");
        DM3 = this.hardwareMap.get(DcMotor.class, "DM3");
        DM4 = this.hardwareMap.get(DcMotor.class, "DM4");

        DM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DM3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DM4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DM3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DM4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DM2.setDirection(DcMotor.Direction.REVERSE); //                           front
        DM4.setDirection(DcMotor.Direction.REVERSE); //motors are layed out like 1^^^^^2
                                                     //                          3^^^^^4

        //example for PID
        PIDC = this.hardwareMap.get(DcMotorEx.class, "PIDC");
        PIDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDC.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //odometry encoders
        En1 = this.hardwareMap.get(DcMotor.class, "En1");
        En2 = this.hardwareMap.get(DcMotor.class, "En2");
        En3 = this.hardwareMap.get(DcMotor.class, "En3");

        En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //servos
        //name = this.hardwareMap.get(CRServo.class, "name");
        //name = this.hardwareMap.get(Servo.class, "name");
        //name.setPosition(Position);

        //webcam
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);


    }

    public void powerControl(double one, double two, double three, double four) {

        DM1.setPower(one);
        DM2.setPower(two);
        DM3.setPower(three);
        DM4.setPower(four);


    }

    public void updateLocation() {

        //constants
        double d1 = 1;
        double d2 = 1;
        double d3 = 1;
        double wheelCircumference = 3 * 2 * Math.PI; // inches
        double ticksPerRotation = 8192;

        //variables
        double x, y, a;
        double gx, gy, ga;
        double e1, e2, e3;
        double de1, de2, de3;
        double r1, r2;
        double t1, t2, tt, ta, ata;
        gx = GlobalPos[0];
        gy = GlobalPos[1];
        ga = GlobalPos[2];
        e1 = En1.getCurrentPosition();
        e2 = En2.getCurrentPosition();
        e3 = En3.getCurrentPosition();

        de1 = wheelCircumference * (e1 - PreviousE[0]) / ticksPerRotation;
        de2 = wheelCircumference * (e2 - PreviousE[1]) / ticksPerRotation;
        de3 = wheelCircumference * (e3 - PreviousE[2]) / ticksPerRotation;
        a = (de2 - de1) / (d1 + d2);
        if (a == 0) {
            x = de1;
            y = de3;
        }
        else {
            r1 = (de2 / a) - d2;
            r2 = (de3 / a) - d3;
            t1 = 2 * r1 * Math.sin(a / 2);
            t2 = 2 * r2 * Math.sin(a / 2);
            tt = Math.hypot(t1, t2);
            ta = Math.atan2(t2, t1);
            ata = ta + ga + (a / 2);
            x = Math.sin(ata) * tt;
            y = Math.cos(ata) * tt;
        }
        gx += x;
        gy += y;
        ga += a;

        MathFunctions.angleWrap(ga);

        PreviousE[0] = e1;
        PreviousE[1] = e2;
        PreviousE[2] = e3;

        GlobalPos[0] = gx;
        GlobalPos[1] = gy;
        GlobalPos[2] = ga;

    }

    public void setPIDCSpeed(double targetSpeed) {

        double kp = 1;
        double ki = 1;
        double kd = 1;
        double sumLimit = 1;
        double error = targetSpeed - PIDC.getVelocity();
        double output = (kp * error) + (ki * sumofErrors) + (kd * (error - lastError));
        if (output > 1) {
            output = 1;
        }
        else if (output < -1) {
            output = -1;
        }
        PIDC.setPower(output);
        lastError = error;
        if ((sumofErrors < sumLimit && sumofErrors > -sumLimit) || (sumofErrors > sumLimit && error < 0) || (sumofErrors < -sumLimit && error > 0)) {
            sumofErrors += error;
        }

    }

    public void moveToPosition(double endX, double endY, double turnSpeed, double movementSpeed, double preferredAngle) {

        double cx = GlobalPos[0];
        double cy = GlobalPos[1];
        double ca = GlobalPos[2];
        double distanceToPoint = Math.hypot((endX - cx), (endY - cy));
        double relativeAngleToPoint = MathFunctions.angleWrap(ca - Math.atan2((endY - cy), (endX - cx)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToPoint;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToPoint;
        double xPower = relativeXToPoint * movementSpeed;
        double yPower = relativeYToPoint * movementSpeed;
        double turnPower = Range.clip((relativeAngleToPoint + preferredAngle) / Math.toRadians(30), -1, 1) * turnSpeed;
        if (distanceToPoint > 3) {
            turnPower = 0;
        }

        powerControl(yPower + xPower + turnPower,
                     yPower - xPower - turnPower,
                    yPower - xPower + turnPower,
                     yPower + xPower - turnPower);

    }

    public void startVision() {

        Pipeline pipeline = new Pipeline();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);            }
        });

        camera.setPipeline(pipeline);

    }

}
