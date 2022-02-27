package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class ExampleMethod extends LinearOpMode {

    MathSpline mathSpline = new MathSpline();
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 0;    // eg: TETRIX Motor Encoder
    static final double     MAX_VELOCITY_DT         = 0;    //velocity when calls "getVelocity()"
    static final double     DRIVE_GEAR_REDUCTION    = 0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        variableHeading(1,20,20,1);

    }

    //Declare your motors

    public DcMotorEx lf   = null;
    public DcMotorEx  rf   = null;
    public DcMotorEx  lb   = null;
    public DcMotorEx  rb   = null;

    //Core Movement Algorithm
    public void variableHeading(double speed, double xPose, double yPose, double timeoutS) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;


            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = rb.getCurrentPosition() + (int) rightDistance;
            }

            lf.setTargetPosition(FleftEncoderTarget);
            lb.setTargetPosition(BleftEncoderTarget);
            rf.setTargetPosition(FrightEncoderTarget);
            rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && lf.isBusy() && rf.isBusy()
                    && lb.isBusy() && rb.isBusy()) {

                lf.setVelocity(speed * mathSpline.returnLPower());
                rf.setVelocity(speed * mathSpline.returnRPower());
                lb.setVelocity(speed * mathSpline.returnLPower());
                rb.setVelocity(speed * mathSpline.returnRPower());


            }

            // Stop all motion;
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Turn off RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

