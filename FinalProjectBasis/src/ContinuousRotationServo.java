import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.Servo;

class ContinuousRotationServo implements Motor {

    private Servo mServo;
    private boolean mReverse;
    private int mRange;

    public ContinuousRotationServo(Servo servo, boolean reverse, int range){
        mServo = servo;
        mReverse = reverse;
        mRange = range;
    }

    public void setPower(int power) {
        if (mReverse)
            power = -power;

        if (power == 0) {
            mServo.off();
            return;
        }
        else if (power > Motor.MAX_FORWARD)
            power = Motor.MAX_FORWARD;
        else if (power < Motor.MAX_REVERSE)
            power = Motor.MAX_REVERSE;

        mServo.setPosition((power * mRange) / Motor.MAX_FORWARD + 50);
    }

    public void brake() {
        mServo.setPosition(50);
    }

    public void stop() {
        mServo.off();
    }
}