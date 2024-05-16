% % 필요한 심볼릭 변수를 선언합니다.
% syms pD pC pB p_l_f alpha state_vel yawdot slip_angle steer real
% 
% % 슬립 각도 계산 함수
% vx = state_vel * cos(slip_angle);
% vy = state_vel * sin(slip_angle);
% front_slip_angle = -atan2(vy + p_l_f * yawdot, vx) + steer;
% 
% % 횡방향 힘 F_fy 정의
% F_fy = pD * sin(pC * atan(pB * alpha));  % 여기서 alpha는 front_slip_angle로 대체됩니다.
% 
% dF_fy_dalpha = diff(F_fy, alpha);
% 
% dalpha_dstate_vel = diff(front_slip_angle, state_vel);
% dalpha_dyawdot = diff(front_slip_angle, yawdot);
% dalpha_dslip_angle = diff(front_slip_angle, slip_angle);
% dalpha_dsteer = diff(front_slip_angle, steer);
% 
% % 각 변수에 대한 F_fy의 미분
% dF_fy_dstate_vel = dF_fy_dalpha * dalpha_dstate_vel;
% dF_fy_dyawdot = dF_fy_dalpha * dalpha_dyawdot;
% dF_fy_dslip_angle = dF_fy_dalpha * dalpha_dslip_angle;
% dF_fy_dsteer = dF_fy_dalpha * dalpha_dsteer;
% 
% disp('dF_fy/dstate_vel:');
% disp(dF_fy_dstate_vel);
% disp('dF_fy/dyawdot:');
% disp(dF_fy_dyawdot);
% disp('dF_fy/dslip_angle:');
% disp(dF_fy_dslip_angle);
% disp('dF_fy/dsteer:');
% disp(dF_fy_dsteer);


% rear tire force
syms pD pC pB p_l_r state_vel yawdot slip_angle alpha real

% Variables for rear slip angle calculations
vx = state_vel * cos(slip_angle);
vy = state_vel * sin(slip_angle);
rear_slip_angle = -atan2(vy - p_l_r * yawdot, vx);

% Define the rear tire force F_ry
F_ry = pD * sin(pC * atan(pB * alpha));

% Differentiate F_ry with respect to alpha
dF_ry_dalpha = diff(F_ry, alpha);

% Differentiate rear_slip_angle with respect to influencing parameters
dalpha_dstate_vel = diff(rear_slip_angle, state_vel);
dalpha_dyawdot = diff(rear_slip_angle, yawdot);
dalpha_dslip_angle = diff(rear_slip_angle, slip_angle);

% Total derivatives of F_ry using the chain rule
dF_ry_dv = simplify(dF_ry_dalpha * dalpha_dstate_vel);
dF_ry_dyawdot = simplify(dF_ry_dalpha * dalpha_dyawdot);
dF_ry_dslip_angle = simplify(dF_ry_dalpha * dalpha_dslip_angle);

fprintf('dF_ry/dv: %s\n', char(dF_ry_dv));
fprintf('dF_ry/dyawdot: %s\n', char(dF_ry_dyawdot));
fprintf('dF_ry/dslip_angle: %s\n', char(dF_ry_dslip_angle));

