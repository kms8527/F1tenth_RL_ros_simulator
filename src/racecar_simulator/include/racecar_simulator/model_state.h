// 상태 인터페이스
class IState {
public:
    virtual ~IState() {}
    // 여기에 상태 관련 메소드 추가
};

// ODEState 상태 구현
struct ODEState : public IState {
    double x;
    double y;
    double theta;
    double velocity;
    double steer_angle;
    double angular_velocity;
    double slip_angle;

    ODEState(double x, double y, double theta, double velocity, double steer_angle,
             double angular_velocity, double slip_angle)
        : x(x), y(y), theta(theta), velocity(velocity), steer_angle(steer_angle),
          angular_velocity(angular_velocity), slip_angle(slip_angle) {}
};

// PacejkaState 상태 구현
struct PacejkaState : public IState {
    double x;
    double y;
    double theta;
    double velocity;
    double steer_angle;
    double angular_velocity;
    double slip_angle;

    PacejkaState(double x, double y, double theta, double velocity, double steer_angle,
                 double angular_velocity, double slip_angle, double s, double vs)
        : x(x), y(y), theta(theta), velocity(velocity), steer_angle(steer_angle),
          angular_velocity(angular_velocity), slip_angle(slip_angle) {}
};