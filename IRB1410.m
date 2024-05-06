%% General Information:
%{
IRB1410 Simulation using the DH Apporach based on the simplified model
and compared to the vlaues from IRB1410 Studio.

Created as part of:
Malla, O. and Shanmugavel, M. (2024),
"Simplified model to study the kinematics of manipulators
with parallelogram linkages", Industrial Robot,
Vol. ahead-of-print No. ahead-of-print.
https://doi.org/10.1108/IR-01-2024-0046 

The code requires the use of Peter Corke's Robotics Toolbox for Matlab:
https://petercorke.com/toolboxes/robotics-toolbox/

% q4 and q7 are needed for the DH table to accureately represent the model

% q3 = q3 - q2 : parallelogram

% Link (theta, D: joint extension, a: joint offset, alpha: joint twist)
%}

%% Robot Specs (Dimensions are in mm):

% q4 and q7 are needed for the DH table to accureately represent the model
% q3 = q3 - q2 : parallelogram
% Link (theta, D: joint extension, a: joint offset, alpha: joint twist)

L1 = 475; % length of link 1 : d1
a1 = 150;
L2 = 600;
L3 = 120;
L4 = 720; % length of link 2
L5 = 85; % length of link 3

fprintf('IRB 1410: \n\n');
% Joints' limits:
qmin =  [-170 , -70, -140, 0, -150, -115, 0, -300];
qmax =  [+170 , +70, +140, 0, +150, +115, 0, +300];

% Joint Limits Radians:
theta1_min = deg2rad(qmin(1)); % minimum joint angle 1
theta1_max = deg2rad(qmax(1)); % maximum joint angle 1
theta2_min = deg2rad(qmin(2)); % minimum joint angle 2
theta2_max = deg2rad(qmax(2)); % maximum joint angle 2
theta3_min = deg2rad(qmin(3)); % minimum joint angle 3
theta3_max = deg2rad(qmax(3)); % maximum joint angle 3
theta4_min = deg2rad(qmin(4)); % This joint is extra % No need to control
theta4_max = deg2rad(qmax(4)); % This joint is extra % No need to control
theta5_min = deg2rad(qmin(5)); % minimum joint angle 4
theta5_max = deg2rad(qmax(5)); % maximum joint angle 4
theta6_min = deg2rad(qmin(6)); % minimum joint angle 5
theta6_max = deg2rad(qmax(6)); % maximum joint angle 5
theta7_min = deg2rad(qmin(7)); % This joint is extra % No need to control
theta7_max = deg2rad(qmax(7)); % This joint is extra % No need to control
theta8_min = deg2rad(qmin(8)); % minimum joint angle 6
theta8_max = deg2rad(qmax(8)); % maximum joint angle 6

% Defining robot base relative to workspace origin point "Camera":
Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
fprintf('Robot 1 base transformation from workspace base point: \n');
disp(Tbase1);

% Discretizing the joint rotations:
SampleRate = 10;

n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;
n4 = ((qmax(5)-qmin(5))/SampleRate)+1; % 180/SampelRate = 1800 points
n5 = ((qmax(6)-qmin(6))/SampleRate)+1;
n6 = ((qmax(8)-qmin(8))/SampleRate)+1;

N = n1*n2*n3*n4*n5*n6;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta3_max,n3);
q4 = linspace(theta5_min,theta5_max,n4);
q5 = linspace(theta6_min,theta6_max,n5);
q6 = linspace(theta7_min,theta7_max,n6);

% DH Method:
% Link([0,d,a,theta])

IRB1410M(1)= Link([0 , L1  , a1 , -pi/2]);
IRB1410M(2)= Link([0 ,  0  , L2 , 0     ]);
IRB1410M(3)= Link([0 ,  0  , L3 , 0     ]);
IRB1410M(4)= Link([0 ,  0  , 0  , -pi/2 ]);
IRB1410M(5)= Link([0 ,  L4 , 0  , pi/2  ]);
IRB1410M(6)= Link([0 ,  0  , L5 , 0     ]);
IRB1410M(7)= Link([0 ,  0 , 0   , -pi/2 ]);
IRB1410M(8)= Link([0 ,  0 , 0  , 0  ]);

%%%%%%%%%%%%%%%%%%%%% q4 not to be modified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% q7 not to be modified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%IRB1410M(1).offset = pi;
IRB1410M(1).qlim = [theta1_min,theta1_max];
IRB1410M(2).offset = -pi/2;
IRB1410M(2).qlim = [theta2_min,theta2_max];
%IRB1410M(3).offset = pi/2 - IRB1410M(2).theta;
IRB1410M(4).offset = 0;
IRB1410M(3).qlim = [theta3_min,theta3_max];
IRB1410M(4).qlim = [theta4_min,theta4_max];
IRB1410M(5).qlim = [theta5_min,theta5_max];
IRB1410M(5).offset = 0;
IRB1410M(6).qlim = [theta6_min,theta6_max];
IRB1410M(6).offset = pi/2;
IRB1410M(7).offset = -pi/2;
IRB1410M(7).qlim = [theta7_min,theta7_max];
IRB1410M(8).offset = pi;

Rob1 = SerialLink(IRB1410M,'name','IRB1410');

figure;
Q5 = [deg2rad(77.07),deg2rad(28.93),deg2rad(-28.93),0,deg2rad(-26),deg2rad(24.53),0,deg2rad(-52)];
Q6 = [deg2rad(61.20),deg2rad(17.73),deg2rad(-1-17.73),0,deg2rad(-26),deg2rad(24.53),0,deg2rad(-52)];
Q7 = [deg2rad(-37.6),deg2rad(35.1),deg2rad(1.1-35.1),0,deg2rad(-67.1),deg2rad(-15.1),0,deg2rad(-65.8)];
Q7 =[deg2rad(24.7),deg2rad(12.6),deg2rad(9.9-12.6),0,deg2rad(11.3),deg2rad(7.9),0,deg2rad(13.6)];
Rob1.plot(Q6,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.5,0.1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);
Q = Q7;
Rob1.teach(Q,'eul'); %'rpy')%'approach')%,'eul');

% CONFIRM Orientation WITH EPSON CRS IN ROBODK

%% Calculating the Workspace from the DH-Method:
% Calculating the Workspace: 
x   = 1;

%TR1 = zeros(4,4,N);

for i = 1:length(q1)
    for j=1:length(q2)
        for ii = 1:length(q3)
            for jj = 1:length(q4)
                for iii = 1:length(q5)
                    for jjj = 1:length(q6)
                    % for jj = 1:length(q4)  
                       % TR1(:,:,x) = Rob1.fkine([q1(i),q2(j),q3(ii)-q2(j),-q3(ii),q4(jj)]);
                       % for faster results q4 is considered 0 as it will not have
                       % effects on the workspace with the system studied
                       Q = [q1(i),q2(j),q3(ii)-q2(j),0,q4(jj),q5(iii),0,q6(jjj)];
                       TR1(:,:,x) = Rob1.fkine(Q);
                       x = x+1;
            
                    end
                end
            end
        end
    end
end

A1 = transl(TR1);

%% Centroid and Alpha Shape
% K-Means clsutering to find the center of mass for the points of Robot 2:
num_clusters1 = 1;
[~, centroid1] = kmeans(A1 , num_clusters1);

Shrink_Factor = 0.8;
[k_A,vol_A] = boundary(A1,Shrink_Factor);

%% Plotting the workspace of the Robot:
% Alpha-Shape
figure;
trisurf(k_A, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB1410 with AlphaShape: Sf =  " + (Shrink_Factor));
axis equal;
hold off;

% Plotting with a color map representation:
map = [0 0 0.2; 0 0 0.3; 0 0 0.4; 0 0 0.45; 0.1 0.1 0.5; 0.1 0.1 0.55; 
    0.15 0.15 0.6; 0.2 0.2 0.65; 0.25 0.25 0.7; 0.3 0.3 0.8; 0.4 0.4 0.9];
figure;
custom_colormap = map;
min_cdata = min(A1(:,1));
max_cdata = max(A1(:,1));
normalized_cdata = (A1(:,1) - min_cdata) / (max_cdata - min_cdata);
colormap(custom_colormap);
scatter3(A1(:,1), A1(:,2), A1(:,3), 'filled', 'CData', normalized_cdata,'MarkerEdgeAlpha',0.1,'MarkerFaceAlpha',0.4);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Point-cloud of IRB1410's Workspace: ");
axis equal;
hold off;

%
% figure;
% %Rob1.plot([0,0,0,0,0]);
% %c = [0.2 0.2 0.6];
% c=  [0.4 0.4 0.7];
% % scatter3(A1(:,1),A1(:,2),A1(:,3),100,'filled','MarkerEdgeColor','b','MarkerFaceColor',c,'MarkerEdgeAlpha',0,'MarkerFaceAlpha',0.2)
% scatter3(A1(:,1),A1(:,2),A1(:,3),'filled','CData', A1(:,2));
% hold on
% plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title("Point-cloud of IRB1410's Workspace: ");
% axis equal;
% hold off;

% Convex Hull
k_convex = convhull(A1);
figure;
trisurf(k_convex, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB1410 - using Convex Hull");
axis equal;
hold off;

% Delaunay Triangulation
DT = delaunay(A1);
figure;
trisurf(DT, A1(:,1), A1(:,2), A1(:,3), 'FaceAlpha', 0.3, 'FaceColor', [0.4 0.6 0.7]);
hold on;
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB1410 - using Delaunay Triangulation");
axis equal;
hold off;

%% Test Cases:
% Target_j_n = [j1,j2,j3,j4,j5,j6];
% Targetn = [X,Y,Z,q1,q2,q3,q4]; quaternions

Target_j_1 = [30.80,-12.13,15.33,-40.00,59.80,-28.00];
Target_1 = [691.027481335,356.960034976,921.353218153,0.129239344,-0.572731138,0.746819702,-0.31230845];
Target_j_2 = [40.80,-17.73,39.17,-30.00,47.53,32.00];
Target_2 = [483.357754156,375.80817434,606.4456307,0.13862261,-0.242750247,0.949700763,-0.141154349];
Target_j_3 = [-11.33,27.07,19.00,-30.00,39.87,80.00];
Target_3 = [1160.639819891,-260.405414977,822.484566064,0.334258363,0.56295022,0.755295025,-0.029796333];
Target_j_4 = [9.07,39.20,46.50,-8.00,62.87,44.00];
Target_4 = [1072.337088003,160.459738433,420.616108119,0.125173499,-0.255279144,-0.95055749,0.124918423];

Target_j_5 = [-20.40,36.40,63.00,2.00,35.27,84.00];
Target_5 = [870.067200235,-321.74769978,286.787197393,0.066003952,-0.797135821,-0.59941592,0.03030703];
Target_j_6 = [-27.20,27.07,42.83,-40.00,59.80,-8.00];
Target_6 = [890.685775054,-510.842808247,537.443556083,0.18166292,0.044911271,-0.95484921,0.230747801];
Target_j_7 = [-134,17.73,28.17,44.00,21.47,40.00];
Target_7 = [-733.279630613,-797.697667876,755.346436265,0.400395276,0.884247649,-0.231926932,-0.06324252];
Target_j_8 = [-95.20,28.93,6.17,-20.00,21.47,40.00];
Target_8 = [-123.388011229,-1238.419066418,1004.513593115,0.387932122,0.738800923,0.418458807,-0.358572296];
Target_j_9 = [22.67,28.93,9.83,-14.00,6.13,72.00];
Target_9 = [1156.106596183,480.438829756,972.267553245,0.466271708,0.249945798,0.757299548,0.382903625];
Target_j_10 = [43.07,2.80,17.17,-4.00,18.40,20.00];
Target_10 = [711.246065239,662.234742188,927.050306672,0.402628515,-0.201610983,0.865916047,0.217790472];
Target_j_11 = [43.07,4.67,30.00,6.00,-4.60,-44.00];
Target_11 = [701.179733179,654.41229213,780.442154315,0.533800484,-0.547804799,0.643869325,0.019980928];
Target_j_12 = [31.73,-8.40,61.17,12.00,-36.80,24.00];
Target_12 = [508.349971408,301.925043085,460.090671293,0.479876027,0.047676194,0.840934416,0.245510259];

Target_j_13 = [54.40,22.40,48.33,34.00,10.73,-52.00];
Target_13 = [570.648656609,812.280300102,500.534152205,0.252265907,-0.580868779,0.762162053,0.134396353];
Target_j_14 = [-38.53,5.60,15.33,-62.00,9.20,-24.00];
Target_14 = [785.63241451,-641.006224447,969.131559959,0.217540104,-0.288867915,0.763718218,-0.534757996];

Target_j_15 = [29.47,-12.13,-3.00,-12.00,-39.87,84.00];
Target_15 = [690.055363126,402.895981694,1275.750141652,0.53911998,0.099061876,0.396274635,0.736547898];
Target_j_16 = [2.27,0.93,8.00,-62.00,-73.60,28.00];
Target_16 = [914.994192705,108.270208169,1128.116862725,0.818602029,-0.460420568,0.29097464,0.18231121];
Target_j_17 = [15.87,-7.47,15.33,-88.00,-53.67,-28.00];
Target_17 = [796.340913702,297.486304705,984.235735011,0.630447989,-0.750672555,0.03563847,-0.194308897];
Target_j_18 =  [15.87,-12.13,6.17,0.00,0.00,40.00];
Target_18 = [805.226227667,228.868334277,1094.428431439,0.590216816,0.15555747,0.727675996,0.31294349];
Target_j_19 = [20.40,-12.13,35.50,2.00,-13.80,28.00];
Target_19 = [711.366428473,263.799855159,709.74587652,0.509463159,0.071165143,0.824541115,0.235615709];
Target_j_20 = [-6.80,-9.33,8.00,-4.00,9.20,-4.00]; % Image Target
Target_20 = [857.403131805,-103.193832995,1060.582052718,0.588078816,-0.00475728,0.804759938,-0.080635707];

%% Forward Solution Test:

target = Target_1;
targetj = Target_j_1;

targetj = [targetj(1),targetj(2),targetj(3)-targetj(2),0,targetj(4),targetj(5),0,targetj(6)];
ForwardTest = Rob1.fkine(deg2rad(targetj));

XYZs = ForwardTest.t;
ErrorInverse = sqrt((XYZs(1)-target(1))^2+(XYZs(2)-target(2))^2+(XYZs(3)-target(3))^2);
fprintf("Error of the forward solution for Position is: %f \n", ErrorInverse);

%for orientation error:
Os = [ForwardTest.n(1:3),ForwardTest.o(1:3),ForwardTest.a(1:3)];
qreal = target(length(target)-3:length(target));
qreal_conjugate = [qreal(1), -qreal(2:4)];
qsol = rotm2quat(Os);

[OrientError1,OrientError2,OrientError3] = quat2angle(quatmultiply(qsol, qreal_conjugate),"ZYX");
fprintf("Orientation Errors as a series of transformations ZYX: %f %g %f \n",rad2deg([OrientError1,OrientError2,OrientError3]));
XYZs
qsol
%% Inverse Solution Test:
% Quaternions to Rotation Matrix and then Inverse solution Accuracy:

target = Target_12;
T = zeros(4,4);
T(1:3,1:3) = quat2rotm(target(length(target)-3:length(target)));
T(1:4,4) = [target(1),target(2),target(3),1]; 
%solution = S2R(Rob1.ikcon(T));
%solution = Rob1.ikcon(T);

Sol = [0, 0, 0, 0, 0, 0, 0, 0];
if abs(Sol(3) - Sol(2)) < 70
   Sol = Rob1.ikcon(T);
   % Sol = IRB1410Inverse(T(1:3,4),T(1:3,1:3));
end

%solution = IRB1410Inverse(T(1:3,4),T(1:3,1:3));

Tsol = Rob1.fkine(Sol);
XYZs = Tsol.t;
%for orientation error:
Os = [Tsol.n(1:3),Tsol.o(1:3),Tsol.a(1:3)];
qreal = target(length(target)-3:length(target));
qreal_conjugate = [qreal(1), -qreal(2:4)];
qsol = rotm2quat(Os);
[OrientError1,OrientError2,OrientError3] = quat2angle(quatmultiply(qsol, qreal_conjugate),"ZYX");

ErrorInverse = sqrt((XYZs(1)-target(1))^2+(XYZs(2)-target(2))^2+(XYZs(3)-target(3))^2);
fprintf("Error of the inverse solution for Position is: %f \n", ErrorInverse);
fprintf("Orientation Errors as a series of transformations ZYX: %f %g %f \n",[OrientError1,OrientError2,OrientError3]);
rad2deg([Sol(1),Sol(2),Sol(3)+Sol(2),Sol(5),Sol(6),Sol(8)])

%% Inverse (other tests):

Q = [deg2rad(20),deg2rad(0),deg2rad(0),deg2rad(0),0,0,0,0];
T = Rob1.fkine(Q);
Sol = [0, 0, 0, 0, 0, 0, 0, 0];

if abs(Sol(3) - Sol(2)) < 70
    Sol = Rob1.ikcon(T);
end

rad2deg(Sol)
Rob1.fkine(Sol)
Rob1.fkine(Q)

% WE COULD POSSIBLY ADD ONE MORE FRAME JUST TO COUNTER THE SECOND JOINT
% ROTATIONS AND THEN WE CAN PLAN THE THIRD JOINT INDEPENDENTALY.

% %% Test Case: Inverse kinematics (Specified):
% 
% TP = [1069.320, 389.201, 850.687]; % Transformations
% RP = [deg2rad(170.480), deg2rad(46.042), deg2rad(157.824)]; % Rotations
% 
% TP = [886.977, 790.108, 489.828]; % Transformations
% RP = [deg2rad(154.733), deg2rad(-16.778), deg2rad(155.267)]; % Rotations
% 
% TP = [-199.591, 1131.939, 1086.197]; % Transformations
% RP = [deg2rad(132.307), deg2rad(-6.409), deg2rad(-140.431)]; % Rotations
% 
% 
% 
% %IntGuess = [-0.3,0,0,0];
% tic;
% %[Q,iterations] = IRB1410Inverse(TestPoint,IntGuess);
% [Qsol,iterations] = IRB1410Inverse(TP,RP); % Without Initial Guess!
% toc;
% t = toc;
% 
% % Mapping Q to Robot's joints:
% Q2 = Qsol;
% F = Rob1.fkine(Qsol).t;
% Error = sqrt((TP(1)-F(1))^2+(TP(2)-F(2))^2+(TP(3)-F(3))^2);
% 
% format short;
% 
% % Display the Q values
% % disp('Real angles Values:');
% % disp(T);
% 
% disp('Q values: ');
% disp(S2R(Q2));
% disp('The Euclidean distance between the solution and the original point is: ');
% disp(Error);
% format short;
% disp('Number of iterations: ');
% disp(iterations);
% disp('Time taken: ');
% disp(t);
% format Long;
% disp('End Effecto Position');
% disp(F);
% 
% figure;
% Rob1.plot(Qsol,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
% set(gca,'color',[0.5,0.6,0.7]);


%% Quaternion to ZYX and ZYZs:

quat5 = [0.555 -0.7923 0.2426 -0.0703] ;
quat6 = [0.547544, -0.746, 0.348, -0.1468];
quat = quat6;
%eulZYZ = quat2eul(quat,"XYZ")
%rad2deg(eulZYZ)
[yaw,pitch,roll] = quat2angle(quat,"ZYX");
% Correct one in robot conversion ZYX:
roll = rad2deg(roll)
pitch = rad2deg(pitch)
yaw = rad2deg(yaw)

%To match robotics toolbox
[yaw,pitch,roll] = quat2angle(quat,"ZYZ");
roll = rad2deg(roll)
pitch = rad2deg(pitch)
yaw = rad2deg(yaw)
%% Functions:
    syms q1v q2v q3v q4v q5v q6v  a1v L1v L2v L3v L4v L5v

    IRB1410M(1)= Link([0 , L1v  , a1v , -pi/2]);
    IRB1410M(2)= Link([0 ,  0  , L2v , 0     ]);
    IRB1410M(3)= Link([0 ,  0  , L3v , 0     ]);
    IRB1410M(4)= Link([0 ,  0  , 0  , -pi/2 ]);
    IRB1410M(5)= Link([0 ,  L4v , 0  , pi/2  ]);
    IRB1410M(6)= Link([0 ,  0  , L5v , 0     ]);
    IRB1410M(7)= Link([0 ,  0 , 0   , -pi/2 ]);
    IRB1410M(8)= Link([0 ,  0 , 0  , 0  ]);

    IRB1410M(2).offset = -pi/2;
    IRB1410M(4).offset = 0;
    IRB1410M(5).offset = 0;
    IRB1410M(6).offset = pi/2;
    IRB1410M(7).offset = -pi/2;
    IRB1410M(8).offset = pi;
    
    Robeq = SerialLink(IRB1410M,'name','IRB1410');
    
    % Compute the end-effector pose in terms of the symbolic joint angles
    Q = [q1v, q2v, q3v-q2v, 0, q4v, q5v, 0, q6v];
    
    clear Xeq Yeq Zeq Rxeq Ryeq Rzeq qmin qmax
    
    TeqDH = Robeq.fkine(Q);
    Xeq = TeqDH.t(1);
    Yeq = TeqDH.t(2);
    Zeq = TeqDH.t(3);
    Rxeq = atan2(TeqDH.a(2),TeqDH.a(1)); % Roll
    Ryeq = atan2(-TeqDH.n(1),sqrt(TeqDH.o(1)^2 + TeqDH.a(1)^2)); % Pitch
    Rzeq = atan2(TeqDH.n(2),TeqDH.n(1)); % Yaw

    % Robot Link Lengths
    L1 = 475; % length of link 1 : d1
    a1 = 150;
    L2 = 600;
    L3 = 120;
    L4 = 720; % length of link 2
    L5 = 85; % length of link 3
    
    X  = subs(Xeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
    Y  = subs(Yeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
    Z  = subs(Zeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
    Rx = subs(Rxeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
    Ry = subs(Ryeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
    Rz = subs(Rzeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);

%% 2. Inverse Kinematics Function:
% Call IRB1410Inverse to calculate first 3 joints' variables
% If currentPose is provided, it will be used as an initial guess
% Otherwise, a default initial guess [0,42.5,52.5,0] is used
% 'interior-point' handles large, sparse problems, as well as small dense problems.
% The algorithm satisfies bounds at all iterations, and can recover from NaN or Inf results. 
% How to Use: [endEffector, currentPose] = End_effector_position;

%% Iterations are removed 
%function [Q, numIterations, timetaken] = IRB1410Inverse(endEffector,Rotation,currentPose)
function [Q, timetaken] = IRB1410Inverse(endEffector,Rotation,currentPose)
%     syms q1v q2v q3v q4v q5v q6v  a1v L1v L2v L3v L4v L5v
% 
%     IRB1410M(1)= Link([0 , L1v  , a1v , -pi/2]);
%     IRB1410M(2)= Link([0 ,  0  , L2v , 0     ]);
%     IRB1410M(3)= Link([0 ,  0  , L3v , 0     ]);
%     IRB1410M(4)= Link([0 ,  0  , 0  , -pi/2 ]);
%     IRB1410M(5)= Link([0 ,  L4v , 0  , pi/2  ]);
%     IRB1410M(6)= Link([0 ,  0  , L5v , 0     ]);
%     IRB1410M(7)= Link([0 ,  0 , 0   , -pi/2 ]);
%     IRB1410M(8)= Link([0 ,  0 , 0  , 0  ]);
% 
%     IRB1410M(2).offset = -pi/2;
%     IRB1410M(4).offset = 0;
%     IRB1410M(5).offset = 0;
%     IRB1410M(6).offset = pi/2;
%     IRB1410M(7).offset = -pi/2;
%     IRB1410M(8).offset = pi;
%     
%     Robeq = SerialLink(IRB1410M,'name','IRB1410');
%     
%     % Compute the end-effector pose in terms of the symbolic joint angles
%     Q = [q1v, q2v, q3v-q2v, 0, q4v, q5v, 0, q6v];
%     
%     clear Xeq Yeq Zeq Rxeq Ryeq Rzeq qmin qmax
%     
%     TeqDH = Robeq.fkine(Q);
%     Xeq = TeqDH.t(1);
%     Yeq = TeqDH.t(2);
%     Zeq = TeqDH.t(3);
%     Rxeq = atan2(TeqDH.a(2),TeqDH.a(2));
%     Ryeq = -asin(TeqDH.a(1));
%     Rzeq = atan2(TeqDH.o(1),TeqDH.a(1));
% 
%     % Robot Link Lengths
%     L1 = 475; % length of link 1 : d1
%     a1 = 150;
%     L2 = 600;
%     L3 = 120;
%     L4 = 720; % length of link 2
%     L5 = 85; % length of link 3
    
%     X  = subs(Xeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
%     Y  = subs(Yeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
%     Z  = subs(Zeq , [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
%     Rx = subs(Rxeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
%     Ry = subs(Ryeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);
%     Rz = subs(Rzeq, [L1v, L2v, L3v, L4v, L5v, a1v], [L1, L2, L3, L4, L5, a1]);

    % Joints' limits: Actual!
    qmin =  [-170 , -70, -140, -150, -115, -300];
    qmax =  [+170 , +70, +140, +150, +115, +300];

    %Simplified Equations: 
%     X  = 150*cos(q(1)) + 720*cos(q(1))*cos(q(3)) + 600*cos(q(1))*sin(q(2)) + 120*cos(q(1))*sin(q(3)) - 85*sin(q(1))*sin(q(4))*sin(q(5)) + 85*cos(q(1))*cos(q(3))*cos(q(5)) - 85*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5));
%     Y  = 150*sin(q(1)) + 720*cos(q(3))*sin(q(1)) + 600*sin(q(1))*sin(q(2)) + 120*sin(q(1))*sin(q(3)) + 85*cos(q(3))*cos(q(5))*sin(q(1)) + 85*cos(q(1))*sin(q(4))*sin(q(5)) - 85*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
%     Z  = 600*cos(q(2)) + 120*cos(q(3)) - 720*sin(q(3)) - 85*cos(q(5))*sin(q(3)) - 85*cos(q(3))*cos(q(4))*sin(q(5)) + 475;
%     Rx = angle(cos(q(3))*cos(q(5))*sin(q(1))*(1 + 1i) + cos(q(1))*sin(q(4))*sin(q(5))*(1 + 1i) - cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5))*(1 + 1i));
%     Ry = asin(sin(q(1))*sin(q(4))*sin(q(5)) - cos(q(1))*cos(q(3))*cos(q(5)) + cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)));
%     Rz = atan2(-cos(q(4))*cos(q(6))*sin(q(1)) + cos(q(1))*cos(q(6))*sin(q(3))*sin(q(4)) + cos(q(1))*cos(q(3))*sin(q(5))*sin(q(6)) + cos(q(5))*sin(q(1))*sin(q(4))*sin(q(6))  + cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3))*sin(q(6)) , cos(q(1))*cos(q(3))*cos(q(5)) - sin(q(1))*sin(q(4))*sin(q(5)) - cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)));


    % Define the objective function (error to minimize)
    objective = @(q) norm([(150*cos(q(1)) + 720*cos(q(1))*cos(q(3)) + 600*cos(q(1))*sin(q(2)) + 120*cos(q(1))*sin(q(3)) - 85*sin(q(1))*sin(q(4))*sin(q(5)) + 85*cos(q(1))*cos(q(3))*cos(q(5)) - 85*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5))) - endEffector(1);
                           (150*sin(q(1)) + 720*cos(q(3))*sin(q(1)) + 600*sin(q(1))*sin(q(2)) + 120*sin(q(1))*sin(q(3)) + 85*cos(q(3))*cos(q(5))*sin(q(1)) + 85*cos(q(1))*sin(q(4))*sin(q(5)) - 85*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5))) - endEffector(2);
                           (600*cos(q(2)) + 120*cos(q(3)) - 720*sin(q(3)) - 85*cos(q(5))*sin(q(3)) - 85*cos(q(3))*cos(q(4))*sin(q(5)) + 475) - endEffector(3); 
                           (angle(cos(q(3))*cos(q(5))*sin(q(1))*(1 + 1i) + cos(q(1))*sin(q(4))*sin(q(5))*(1 + 1i) - cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5))*(1 + 1i))) - Rotation(1);
                           (asin(sin(q(1))*sin(q(4))*sin(q(5)) - cos(q(1))*cos(q(3))*cos(q(5)) + cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)))) - Rotation(2);
                           (atan2(-cos(q(4))*cos(q(6))*sin(q(1)) + cos(q(1))*cos(q(6))*sin(q(3))*sin(q(4)) + cos(q(1))*cos(q(3))*sin(q(5))*sin(q(6)) + cos(q(5))*sin(q(1))*sin(q(4))*sin(q(6))  + cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3))*sin(q(6)) , cos(q(1))*cos(q(3))*cos(q(5)) - sin(q(1))*sin(q(4))*sin(q(5)) - cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)))) - Rotation(3)]);


%     Objective Function FULL
%     objective = @(q) norm([ (150*cos(q(1)) - (223549092000967995*sin(q(1)))/5070602400912917605986812821504 + 720*cos(q(1))*cos(q(3)) + 600*cos(q(1))*sin(q(2)) + 120*cos(q(1))*sin(q(3)) - (422259396001828435*cos(q(5))*sin(q(1)))/81129638414606681695789005144064 - 85*sin(q(1))*sin(q(4))*sin(q(5)) + 85*cos(q(1))*cos(q(3))*cos(q(5)) - (422259396001828435*cos(q(1))*cos(q(3))*sin(q(4))*sin(q(5)))/81129638414606681695789005144064 - 85*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)))- endEffector(1); ...
%         ((223549092000967995*cos(q(1)))/5070602400912917605986812821504 + 150*sin(q(1)) + (422259396001828435*cos(q(1))*cos(q(5)))/81129638414606681695789005144064 + 720*cos(q(3))*sin(q(1)) + 600*sin(q(1))*sin(q(2)) + 120*sin(q(1))*sin(q(3)) + 85*cos(q(3))*cos(q(5))*sin(q(1)) + 85*cos(q(1))*sin(q(4))*sin(q(5)) - (422259396001828435*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5)))/81129638414606681695789005144064 - 85*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5))) - endEffector(2); ...
%         (600*cos(q(2)) + 120*cos(q(3)) - 720*sin(q(3)) - 85*cos(q(5))*sin(q(3)) + (422259396001828435*sin(q(3))*sin(q(4))*sin(q(5)))/81129638414606681695789005144064 - 85*cos(q(3))*cos(q(4))*sin(q(5)) + 475) - endEffector(3); ...
%         (angle((cos(q(1)) + sin(q(1))*1i)*(cos(q(4))*403032377821159473656973322630821148176053960704i + cos(q(5))*403032377821159473656973322630821148176053960704i - 24678615572571482867467662723121*cos(q(3))*cos(q(4)) + 6582018229284824168619876730229402019930943462534319453394436096*cos(q(3))*cos(q(5)) + 403032377821159473656973322630821148176053960704*sin(q(3))*sin(q(4)) + sin(q(4))*sin(q(5))*6582018229284824168619876730229402019930943462534319453394436096i - 403032377821159473656973322630821148176053960704*cos(q(3))*sin(q(4))*sin(q(5)) - 6582018229284824168619876730229402019930943462534319453394436096*cos(q(4))*sin(q(3))*sin(q(5))))) - Rotation(1); ...
%         (atan2(cos(q(4))*sin(q(1))*sin(q(6)) - (24678615572571482867467662723121*cos(q(5))*sin(q(1))*sin(q(6)))/6582018229284824168619876730229402019930943462534319453394436096 - (4967757600021511*cos(q(6))*sin(q(1))*sin(q(5)))/81129638414606681695789005144064 + (4967757600021511*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(6)))/81129638414606681695789005144064 + (4967757600021511*cos(q(1))*cos(q(3))*cos(q(5))*sin(q(6)))/81129638414606681695789005144064 + cos(q(1))*cos(q(3))*cos(q(6))*sin(q(5)) + cos(q(5))*cos(q(6))*sin(q(1))*sin(q(4)) - cos(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) - (4967757600021511*sin(q(1))*sin(q(4))*sin(q(5))*sin(q(6)))/81129638414606681695789005144064 + (4967757600021511*cos(q(1))*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(4)))/81129638414606681695789005144064 + cos(q(1))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(3)) - (24678615572571482867467662723121*cos(q(1))*cos(q(3))*sin(q(4))*sin(q(5))*sin(q(6)))/6582018229284824168619876730229402019930943462534319453394436096 - (4967757600021511*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6)))/81129638414606681695789005144064, (((4967757600021511*cos(q(4))*sin(q(1)))/81129638414606681695789005144064 + (4967757600021511*cos(q(5))*sin(q(1)))/81129638414606681695789005144064 + sin(q(1))*sin(q(4))*sin(q(5)) + (24678615572571482867467662723121*cos(q(1))*cos(q(3))*cos(q(4)))/6582018229284824168619876730229402019930943462534319453394436096 - cos(q(1))*cos(q(3))*cos(q(5)) - (4967757600021511*cos(q(1))*sin(q(3))*sin(q(4)))/81129638414606681695789005144064 + (4967757600021511*cos(q(1))*cos(q(3))*sin(q(4))*sin(q(5)))/81129638414606681695789005144064 + cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)))^2 + ((24678615572571482867467662723121*cos(q(5))*cos(q(6))*sin(q(1)))/6582018229284824168619876730229402019930943462534319453394436096 - cos(q(4))*cos(q(6))*sin(q(1)) - (4967757600021511*sin(q(1))*sin(q(5))*sin(q(6)))/81129638414606681695789005144064 - (4967757600021511*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(6)))/81129638414606681695789005144064 - (4967757600021511*cos(q(1))*cos(q(3))*cos(q(5))*cos(q(6)))/81129638414606681695789005144064 + cos(q(1))*cos(q(6))*sin(q(3))*sin(q(4)) + cos(q(1))*cos(q(3))*sin(q(5))*sin(q(6)) + cos(q(5))*sin(q(1))*sin(q(4))*sin(q(6)) + (4967757600021511*cos(q(6))*sin(q(1))*sin(q(4))*sin(q(5)))/81129638414606681695789005144064 + (4967757600021511*cos(q(1))*cos(q(3))*cos(q(5))*sin(q(4))*sin(q(6)))/81129638414606681695789005144064 + (24678615572571482867467662723121*cos(q(1))*cos(q(3))*cos(q(6))*sin(q(4))*sin(q(5)))/6582018229284824168619876730229402019930943462534319453394436096 + cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3))*sin(q(6)) + (4967757600021511*cos(q(1))*cos(q(4))*cos(q(6))*sin(q(3))*sin(q(5)))/81129638414606681695789005144064)^2)^(1/2))) - Rotation(2); ...
%         (angle(-(cos(q(1)) + sin(q(1))*1i)*(403032377821159473656973322630821148176053960704*cos(q(3))*cos(q(4))*sin(q(6)) + cos(q(5))*sin(q(6))*24678615572571482867467662723121i + cos(q(6))*sin(q(5))*403032377821159473656973322630821148176053960704i - 6582018229284824168619876730229402019930943462534319453394436096*sin(q(3))*sin(q(4))*sin(q(6)) + sin(q(4))*sin(q(5))*sin(q(6))*403032377821159473656973322630821148176053960704i - cos(q(4))*sin(q(6))*6582018229284824168619876730229402019930943462534319453394436096i + 403032377821159473656973322630821148176053960704*cos(q(3))*cos(q(5))*sin(q(6)) + 6582018229284824168619876730229402019930943462534319453394436096*cos(q(3))*cos(q(6))*sin(q(5)) - cos(q(5))*cos(q(6))*sin(q(4))*6582018229284824168619876730229402019930943462534319453394436096i + 403032377821159473656973322630821148176053960704*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(4)) + 6582018229284824168619876730229402019930943462534319453394436096*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(3)) - 24678615572571482867467662723121*cos(q(3))*sin(q(4))*sin(q(5))*sin(q(6)) - 403032377821159473656973322630821148176053960704*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6))))) - Rotation(3)]);


    % Initial guess:
    tic;
    % Set initial_guess to currentPose if it's supplied, otherwise use [0, ..., 0]
    if nargin < 3
        initial_guess = [0, 0, 0, 0 , 0, 0];
    else
        initial_guess = currentPose;
    end

    % Define lower and upper bounds (optional, if needed)
    lb = deg2rad(qmin);
    ub = deg2rad(qmax);

%     % Call fmincon:
%     % options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'iter','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
%     options = optimoptions('fmincon','Display', 'iter','MaxIterations', 200,'TolFun', 1e-8, 'TolX', 1e-8);
%     [q_optimal, ~, ~, output] = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);
%     % Access the number of iterations from the output structure
%     numIterations = output.iterations;

    options = optimoptions('particleswarm', 'Display', 'iter', 'MaxIterations', 200, 'SwarmSize', 100, 'HybridFcn', @fmincon,'MaxStallIterations', 50);
    [q_optimal, ~, ~, output] = particleswarm(objective, numel(initial_guess), lb, ub, options);

    options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 200, 'TolFun', 1e-8, 'TolX', 1e-8);    
    % Define the global optimization problem
    problem = createOptimProblem('fmincon', 'objective', objective, 'x0', initial_guess, 'lb', lb, 'ub', ub, 'options', options);
   
    % With contraint
    % problem = createOptimProblem('fmincon', 'objective', objective, 'x0', initial_guess, 'lb', lb, 'ub', ub, 'nonlcon', @(q) myConstraint(q), 'options', options);
    
    % Create a GlobalSearch object
    gs = GlobalSearch('Display', 'iter');
    % Run the global optimization
    [q_optimal, ~, ~, output] = run(gs, problem);

    toc;
    timetaken = toc;
    
    % Access the number of iterations from the output structure
    % numIterations = output.iterations;
    % Calculate Q values using DH parameters
    Q = [q_optimal(1), q_optimal(2), q_optimal(3), 0, q_optimal(4), q_optimal(5), 0, q_optimal(6)]; 

%     function [c, ceq] = myConstraint(q)
%         c = abs(q(3) - q(2)) + 70;  % Inequality constraint
%         ceq = [];  % No equality constraint
%     end

end


% 5. Function to convert real manipulator variables into simulation:
function Qout = R2S(Qin)  
    Qout = [Qin(1),Qin(2),Qin(3)-Qin(2),0,Qin(4),Qin(5),Qin(6)];
end

% 6. Function to convert simulation variables into real manipulator:
function qout = S2R(qin)  
    qout = rad2deg([qin(1),qin(2),qin(3)+qin(2),qin(5),qin(6),qin(8)]);
end

