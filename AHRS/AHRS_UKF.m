
%%chuzhiwei 2020.10.27
clear variables;
clc;
format long;
addpath('../data');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%  ��������  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     test = xlsread('imu.xlsx');
%     nav = xlsread('nav.xlsx');

imudata = importdata('IMU2.txt');
navdata = importdata('NAV.txt');
test = imudata.data;
nav = navdata.data;

time = test(:,1);
adis_ax = test(:,8);
adis_ay = test(:,9);
adis_az = test(:,10);
adis_gx = test(:,11);
adis_gy = test(:,12);
adis_gz = test(:,13);
nav_pitch = nav(:,3);
nav_roll = nav(:,4);
%     bdf_roll = test(:,7);
%     bdf_pitch = test(:,8);
%     bdf_yaw = test(:,9);
%     magx = test(:,16);
%     magy = test(:,17);
%     magz = test(:,18);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% IMU&GNSS˫���ߺ��˽����ںϳ�ʼ�� %%%%%%%%%%%%%%%%%%%%%%%

T = 0.01;%��������(s) (IMU: 100Hz)

row = size(adis_ax, 1); %�������������ܵĲ�������
row = 2 * 10000;

Xhat(1,:) = [0 0 0];%%Ŀ���ʼ������̬�ǣ���λ�㣩
Xhat_1(1,:) = [0 0 0];

Z = zeros(row,3); %%���⺽�˳�ʼ��
%  dualHeadingFlag = 0;%˫���ߺ�����Ч��־λ
pitch_o = zeros(row,1);%%pitch������ֵ
roll_o = zeros(row,1);%%roll������ֵ
yaw_o =  zeros(row,1);%%yaw������ֵ
P = 100*diag([1,1,1]);%��ʼ�������Э����

adis_ax = moving_average_filter(adis_ax, 3);
adis_ay = moving_average_filter(adis_ay, 3);
adis_az = moving_average_filter(adis_az, 3);
%     Gyro_adis = [adis_gx*180/pi, adis_gy*180/pi, adis_gz*180/pi];
Gyro_adis = [adis_gx, adis_gy, adis_gz];
Accel_adis = [adis_ax, adis_ay, adis_az];

%�˴���������Ӧ��ͨ�˲�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���ٶȼ�������̬�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%����pitch/roll
for i = 1 : row
    
    a1 = Accel_adis(i, 1);
    a2 = -Accel_adis(i, 2);
    a3 = -Accel_adis(i, 3);
    pitch_o(i) = atan(a1 / sqrt((a2)^2 + (a3)^2));
    if  ( a3>0 )
        roll_o(i) = atan( a2/a3 );%%%%%%%%%%0-+-90��
    elseif  ( (a2>=0) && (a3<0) )
        roll_o(i) = pi + atan(a2/a3);%%%%%90-180��
    elseif  ( (a2<0) && (a3<0) )
        roll_o(i) = -pi + atan(a2/a3);%%%%%%-90--180��
    end
    
    %Z(t, 1) = yaw_o(t)*180/pi - 1.05;%����ƫ��
    Z(i, 1) = 0;
    Z(i, 2) = pitch_o(i)*180/pi;
    Z(i, 3) = roll_o(i)*180/pi;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ˫���ߺ�������ֵ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Z(1, 1) = Heading_heading(1);%����ǳ�ʼ��

% for t = 1 : row   %%%%%����������⸳ֵ
%     
%     %        Z(t, 1) = yaw_o(t)*180/pi - 1.05;%����ƫ��
%     Z(t, 1) = 0;
%     Z(t, 2) = pitch_o(t)*180/pi;
%     Z(t, 3) = roll_o(t)*180/pi;
%     
% end

Xhat(1,:) = [Z(1,1) 0 0];
xik = 0;
Ro = [1e-8,0,0
    0,1e-8,0
    0,0,1e-8];
R = Ro;

Q = (1e-10) * diag([1,1,1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ʱ����·��̣�Ԥ�⣩ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t = 2 : row
    T  = time(t)-time(t-1);
    %���������㼯����Ȩֵ
    sp = chol((3+xik) * P);  %P�����ǰ���������cholesky�ֽ⣡����ƽ����
    X = UT_transform(Xhat(t-1,:),sp);
    W0 = xik / (3+xik);
    W1 = 1 / (2 * (3+xik));
    W = [W0, W1, W1, W1, W1, W1, W1];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %������Ƽ����ֵ��Э����
    for i = 1 : 7
        X(:,i) = sigma_predict(X(:,i),Gyro_adis(t,:),T);
    end
    
    
    Xhat(t,:) = sigma_mean_value(X,W);%�������ֵ
    Xhat(t, 1) = norm_yaw(Xhat(t, 1));
    Xhat(t, 2) = norm_pitch(Xhat(t, 2));
    Xhat(t, 3) = norm_roll(Xhat(t, 3));
    
    Xhat_1(t,:) = Xhat(t,:);%%%%%%%%������Ϣ���������ֵ
    Z(t,1) = Xhat_1(t,1);
    
    %%%%%%%%%%%%%%%%%%%%%%% ������һ�̺������ֵ��˿�����ֵ�Ĳ��ǰ������ֵ�Ĳ���Լ�������֮��ľ��Բ�ֵ��Ϊ����ӦUKF������׼�� %%%%%
    Xhat_yaw_d =  Xhat_1(t,1) - Xhat(t-1,1);
    Xhat_pitch_d = Xhat_1(t,2) - Xhat(t-1,2);
    Xhat_roll_d =  Xhat_1(t,3) - Xhat(t-1,3) ;
    Z_yaw_d =  Z(t,1) - Z(t-1,1) ;
    Z_pitch_d =  Z(t,2) - Z(t-1,2) ;
    Z_roll_d =  Z(t,3) - Z(t-1,3) ;
    
    Yaw_d = abs( Xhat_yaw_d - Z_yaw_d );
    Yaw_d = norm_do(Yaw_d);
    Pitch_d = abs( Xhat_pitch_d - Z_pitch_d);
    Pitch_d = norm_do(Pitch_d);
    Roll_d = abs( Xhat_roll_d - Z_roll_d);
    Roll_d = norm_do(Roll_d);
    
    P =  prior_P(W, X, Xhat(t,:), Q); %����������Э����
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% �������·��̣������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Pzz = P_zz(W, X, Xhat(t,:), R);
    Pxz = Pzz - R;
    
    K = Pxz / (Pzz);
    Z_X = Z(t,:)' - Xhat(t,:)';
    if(Z_X(1) < -180)
        Z_X(1) = Z_X(1) + 360;
    elseif(Z_X(1) > 180)
        Z_X(1) = Z_X(1) - 360;
    end
    if(Z_X(3) < -180)
        Z_X(3) = Z_X(3) + 360;
    elseif(Z_X(3) > 180)
        Z_X(3) = Z_X(3) - 360;
    end
    Xhat(t, :) = Xhat(t, :)' + K * (Z_X);
    
    %       Xhat(t, 1) = norm_yaw(Xhat(t, 1));
    Xhat(t,1) = 0;
    
    
    P = P - K * Pzz * K';
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% ��������Ӧ����������Դ���ѣ� %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if  Pitch_d > 0.03
        Xhat(t, 2) = Xhat_1(t, 2);%pitch
    end
    if  Roll_d > 0.01
        Xhat(t, 3) = Xhat_1(t, 3);%roll
    end
    
    Xhat(t, 2) = norm_pitch(Xhat(t, 2));
    Xhat(t, 3) = norm_roll(Xhat(t, 3));
end

adis_UKF_Yaw = Xhat(:,1);
adis_UKF_Pitch  = Xhat(:,2);
adis_UKF_Roll = Xhat(:,3);

%%%%%%%% �溽�� %%%%%%%
t = 1 : row;
figure(1)
plot(t, adis_UKF_Yaw);
hold on;
plot( nav(t,2));
hold on;
set(gca,'ygrid','on');
xlabel('���ʱ�䣨0.01s��');
ylabel('���򣨡㣩');
legend('simulat_UKF','realtime_nav');
title('�溽��','fontsize',13);

%%%%%%%% ������ %%%%%%%
figure(2)
plot(t, adis_UKF_Pitch);
hold on;
plot(nav(t,3));
hold on;
set(gca,'ygrid','on');
xlabel('���ʱ�䣨0.01s��');
ylabel('Pitch���㣩');
legend('simulat_UKF','realtime_nav');
title('������','fontsize',13);

%%%%%%%% ����� %%%%%%%
figure(3)
plot(t, adis_UKF_Roll);
hold on;
plot(nav(t,4));
hold on;
set(gca,'ygrid','on');
xlabel('���ʱ�䣨0.01s��');
ylabel('Roll���㣩');
legend('simulat_UKF','realtime_nav');
title('�����','fontsize',13);
