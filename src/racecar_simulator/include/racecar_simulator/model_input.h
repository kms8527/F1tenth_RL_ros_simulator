
// 제어 입력 인터페이스
class IControlInput {
public:
    virtual ~IControlInput() {}
    virtual double getAccel() const = 0;
    virtual double getSteerVelocity() const { return 0; }         // 기본 구현, 필요시 재정의
    virtual double getSteerVelocity() const { return 0; } // 기본 구현, 필요시 재정의
};

// Original 모델을 위한 입력 클래스
class ODEControlInput : public IControlInput {
    double accel;
    double steer_angle_vel;

public:
    ODEControlInput(double a, double s) : accel(a), steer_angle_vel(s) {}

    double getAccel() const override { return accel; }
    double getSteerVelocity() const override { return steer_angle_vel; }
};

class PacejkaInput : public IControlInput {
    double accel;
    double steer_angle_vel;
public:
    PacejkaInput(double a, double s) : accel(a), steer_angle_vel(s) {}

    double getAccel() const override { return accel; }
    double getSteerVelocity() const override { return steer_angle_vel; }
};
