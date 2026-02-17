function lab2(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
% Lab 2 (Forward Kinematics)
% Computes the transformation matrix T of the end-effector using the joint
% angles of the UR3e robot
%
% Input: 6 joint angles theta_1...theta_6
%
% Output: none (but prints forward kinematics results to command window


%% Your code begins here
%Convert input into radian

theta_1 = deg2rad(theta_1);
theta_2 = deg2rad(theta_2);
theta_3 = deg2rad(theta_3);
theta_4 = deg2rad(theta_4);
theta_5 = deg2rad(theta_5);
theta_6 = deg2rad(theta_6);
% First, define the rest of the screw axes, in (mm)

S1 = [0, 0, 1, -300, 0, 0]';
S2 = [0, 1, 0, -240, 0, 0]'; 
S3 = [0, 1, 0, -240, 0, 244]'; 
S4 = [0, 1, 0, -240, 0, 457]'; 
S5 = [0, 0, -1, 169, 457, 0]'; 
S6 = [0, 1, 0, -155, 0, 457]'; 

S1T = VecTose3(S1);
S2T = VecTose3(S2);
S3T = VecTose3(S3);
S4T = VecTose3(S4);
S5T = VecTose3(S5);
S6T = VecTose3(S6);
% Next, define the M matrix (the zero-position e-e transformation matrix),
% in (mm)

M = [ 1, 0, 0, 457;
      0, 1, 0, 1;
      0, 0, 1, 155;
      0, 0, 0, 1 ];



% Now, calculate the T matrix using forward kinematics
I = eye(4);
T = expm(S1T*theta_1)*expm(S2T*theta_2)*expm(S3T*theta_3)...
    *expm(S4T*theta_4)*expm(S5T*theta_5)*expm(S6T*theta_6)*M;


error = sqrt(1/3*((T(1,4)-220)^2+(T(2,4)-(-35))^2+(T(3,4)-360)^2));


%% Your code ends here

% Output results to command window
disp('The transformation matrix T is:')
disp(T)
fprintf('The position of the end-effector is %4.2f, %4.2f, %4.2f (mm) \n',T(1,4),T(2,4),T(3,4))
disp(error);
end

