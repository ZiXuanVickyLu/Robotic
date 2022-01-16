
%my robot designed in part A

L(1) = Link('d', 1, 'a', 0, 'alpha', pi/2);
L(2) = Link('d', 0, 'a', 0, 'alpha', pi/2);
L(3) = Link('a', 0.0, 'alpha', 0, 'theta', 0);
L(3).jointtype='P';

L(3).qlim=[0,5];
L(4) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2);
L(6) = Link('d', 1, 'a', 0, 'alpha', 0);

robot_mine=SerialLink(L,'name','robotMine');

%-------------------------interpolation num ---------------------------
n = 60;
%------------------------------end-------------------------------------

% states
A = zeros(n,6);

T = robot_mine.fkine([0,0,0,0,0,0]);

%plot the ground_truth curve
for i = 1:n
qi = calc_from_function(1/n * i);
qx(i) = qi(1); qy(i) = qi(2);qz(i) = qi(3);
end

plot3(qx,qy,qz);
hold on;

%use for error analysis
err = zeros(3,n);

% --------------------------main control flow----------------------------
for i = 1:n
    
% x
qi = calc_from_function(1/n * i);
B = pose_desired(qi);
F = double(T);
F([1,2,3],4) = 0;
T = SE3.convert(F) + B;

%induce 5% noise 
%-----------------------------change resampling--------------------------
eta = sum_and_mean(1000);
%--------------------------------end-------------------------------------
eta(3) = abs(eta(3));

%feedback <-
% backward kinematics
A(i,:) = robot_mine.ikine(T ,'mask',[1 1 1 0 0 0]);
A(i,3) = abs(A(i,3)); 
scatter3(qi(1),qi(2),qi(3),'filled');
% forward kinematics
T = robot_mine.fkine(A(i,:) + eta);
tmp = double(T);
err(:,i) = tmp([1 2 3],4);

%feedback ->

end
%-------------------------------end control--------------------------------


robot_mine.plot(A,'workspace',[-6,6,-6,6,-10,2]);

s = analysis_err(err,qx,qy,qz,n);


% from spatial curve generate the position.
% u in [0,1]
function q = calc_from_function(u) 
q(1) =  5 * sin(2*pi*u);
q(2) = -3 *u^2 - 4 * sin( 8 * pi * u);
q(3) = -6*u^3;
end

% change pose T with dx
function x_d = pose_desired(q)
c = zeros(4,4);
c(1,4) = q(1);
c(2,4) = q(2);
c(3,4) = q(3);
x_d = SE3.convert(c);
end

%analysis error: ground truth v.s. pose
function eta = analysis_err(err, qx ,qy, qz, n)
figure(2)
index = zeros(1,n);
M = zeros(1,n);
for i = 1:n
    index(i) = i;
end
eta = ((err(1,:) - qx).^2 + (err(2,:) - qy).^2 + (err(3,:) - qz).^2).^(1/2)*10;
m = mean(eta);
for i = 1:n
    M(i) = m;
end

plot([0 index],[m M])
hold on
scatter(index',eta','filled')
title('5% noise, 1000 resampling')
xlabel('iterative step')
ylabel('relative error %')
legend('average error')
end

% resampling
function eta = sum_and_mean(n)
eta = zeros(1,6);
for i = 1:n
eta = eta + pi/40 * randn(1, 6);
end
eta = eta/n;
end
