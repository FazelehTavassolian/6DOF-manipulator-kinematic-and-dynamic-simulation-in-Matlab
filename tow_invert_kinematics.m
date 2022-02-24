format compact;
tic
disp('-----invert kinematic-------')
syms q1 q2 q3 q4 q5 q6 l1 l2 t real
phi=0;siy=-pi/2;R=1;
d1=1;d6=1;d=.5;
a2=1;a3=1;
for i=1:50
    siy=siy+pi/50;
    phi=phi+pi/50;
    o=[R*cos(phi).*cos(siy)+50 R*cos(phi).*sin(siy)+50 R*sin(phi)+50];
    r=[1.1 1 1.2;1.3 1.4 1.4;.9 1.2 1];
    xc(i)=o(1,1)-d6*r(1,3);
    yc(i)=o(1,2)-d6*r(2,3);
    zc(i)=o(1,3)-d6*r(3,3);
    x(i)=o(1);
    y(i)=o(2);
    z(i)=o(3);
    D(i)=(xc(i)^2+yc(i)^2-d^2+(zc(i)-d1)^2-a2^2-a3^2)/(2*a2*a3);
    teta1(i)=atan2(xc(i),yc(i));
    teta3(i)=atan2(D(i),sqrt(1-D(i)^2));
    teta2(i)=atan2(sqrt(xc(i)^2+yc(i)^2-d^2),zc(i)-d1)-atan2(a2+a3*cos(teta3(i)),a3*sin(teta3(i)));
    teta4(i)=atan2(cos(teta1(i))*cos(teta2(i)+teta3(i))*r(1,3)+sin(teta1(i))*cos(teta2(i)+teta3(i))...
        *r(2,3)+sin(teta2(i)+teta3(i))*r(3,3),...
    -cos(teta1(i))*sin(teta2(i)+teta3(i))*r(1,3)-sin(teta1(i))*sin(teta2(i)+teta3(i))...
    *r(2,3)+cos(teta2(i)+teta3(i))*r(3,3));
    teta5(i)=atan2(sin(teta1(i))*r(1,3)-cos(teta1(i))*r(2,3),sqrt(1-(sin(teta1(i))...
        *r(1,3)-cos(teta1(i))*r(2,3))^2));
    teta6(i)=atan2(-sin(teta1(i))*r(1,1)+cos(teta1(i))*r(2,1),sin(teta1(i))*r(1,2)-cos(teta1(i))*r(2,2));
   % plot(o(1,1),o(1,2),'*')
   plot3(o(1,1),o(1,2),o(1,3),'*')
   hold on
end
Q1=teta1*180/pi;   Q2=teta2*180/pi;   Q3=teta3*180/pi;
Q4=teta4*180/pi;   Q5=teta5*180/pi;   Q6=teta6*180/pi;
Q=[Q1;Q2;Q3;Q4;Q5;Q6]
%save('Q','Q1','Q2','Q3','Q4','Q5','Q6');
j=[jv6;jw6];
syms dx dy d4 d6
%--------trajectory-----------------
for i=1:50
    dz=.2;
    DX=(2*dy*(y(i)-yc(i))+2*dz*(z(i)-zc(i)))/(2*(x(i)-xc(i)));
    e=dx^2+dy^2+dz^2-1;
    ee=subs(e,dx,DX);
    dyy=(solve(ee));dyy=dyy(1);
    dxx=subs(DX,dy,dyy);dxx=dxx(1);
    newj=subs(j,{q1,q2,q3,q4,q5,q6,d4,d6,l1,l2},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i),1,.8,1,1});
    dqq=newj\[dxx;dyy;0.2;0;0;0];
    outt=dqq;
    DQQ1(i)=outt(1);
    DQQ2(i)=outt(2);
    DQQ3(i)=outt(3);
    DQQ4(i)=outt(4);
    DQQ5(i)=outt(5);
    DQQ6(i)=outt(6);
 end
DQ1=(DQQ1);
DQ2=(DQQ2);
DQ3=(DQQ3);
DQ4=(DQQ4);
DQ5=(DQQ5);
DQ6=(DQQ6);
DQ=[DQ1;DQ2;DQ3;DQ4;DQ5;DQ6]
a1=[];a2=[];a3=[];a4=[];a5=[];a6=[];DDQ1=[];DDQ2=[];DDQ3=[];DDQ4=[];DDQ5=[];DDQ6=[];
%  save('DQ','DQ1','DQ2','DQ3','DQ4','DQ5','DQ6');
% %---------------velocity equation(dq)-------------
for i=1:49
        tstf=[1 i i^2 i^3;
        0 1 2*i 3*i^2;
        1 i+1 (i+1)^2 (i+1)^3;
        0 1 2*(i+1) 3*(i+1)^3];
        invt=inv(tstf);
        aa1=invt*[Q1(i) DQ1(i) Q1(i+1) DQ1(i+1)]';
        a1=[a1 aa1];
        aa2=invt*[Q2(i) DQ2(i) Q2(i+1) DQ2(i+1)]';
        a2=[a2 aa2];
        aa3=invt*[Q3(i) DQ3(i) Q3(i+1) DQ3(i+1)]';
        a3=[a3 aa3];
        aa4=invt*[Q4(i) DQ4(i) Q4(i+1) DQ4(i+1)]';
        a4=[a4 aa4];
        aa5=invt*[Q5(i) DQ5(i) Q5(i+1) DQ5(i+1)]';
        a5=[a5 aa5];
        aa6=invt*[Q6(i) DQ6(i) Q6(i+1) DQ6(i+1)]';
        a6=[a6 aa6];
end
%------------------acceleration(ddq)--------------
 for i=1:49
         dq1=a1(1,i)+2*a1(2,i)*t+3*a1(3,i)*t^2;
         dq2=a2(1,i)+2*a2(2,i)*t+3*a2(3,i)*t^2;
         dq3=a3(1,i)+2*a3(2,i)*t+3*a3(3,i)*t^2;
         dq4=a4(1,i)+2*a4(2,i)*t+3*a4(3,i)*t^2;
         dq5=a5(1,i)+2*a5(2,i)*t+3*a5(3,i)*t^2;
         dq6=a6(1,i)+2*a6(2,i)*t+3*a6(3,i)*t^2;
         ddqq1=jacobian(dq1,t);
         ddqq2=jacobian(dq2,t);
         ddqq3=jacobian(dq3,t);
         ddqq4=jacobian(dq4,t);
         ddqq5=jacobian(dq5,t);
         ddqq6=jacobian(dq6,t);

         DDQQ1=subs(ddqq1,t,i);
         DDQQ2=subs(ddqq2,t,i);
         DDQQ3=subs(ddqq3,t,i);
         DDQQ4=subs(ddqq4,t,i);
         DDQQ5=subs(ddqq5,t,i);
         DDQQ6=subs(ddqq6,t,i);
         %-------SHETAB -------
         DDQ1=[DDQ1 DDQQ1];
         DDQ2=[DDQ2 DDQQ2];
         DDQ3=[DDQ3 DDQQ3];
         DDQ4=[DDQ4 DDQQ4];
         DDQ5=[DDQ5 DDQQ5];
         DDQ6=[DDQ6 DDQQ6];
 end 
 DDQ=[DDQ1;DDQ2;DDQ3;DDQ4;DDQ5;DDQ6]
%  save('DDQ','DDQ1','DDQ2','DDQ3','DDQ4','DDQ5','DDQ6');
 time=1:50;tt=1:49;
%----------------first link------------------------------------------------
figure
subplot(3,1,1),plot(time,Q1,'black'),ylabel('q1(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ1,'black'),ylabel('dq1(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ1,'black'),xlabel('time(sec)'),ylabel('ddq1(deg/s^2)')
%---------------second link------------------------------------------------
figure
subplot(3,1,1),plot(time,Q2,'black'),ylabel('q2(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ2,'black'),ylabel('dq2(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ2,'black'),xlabel('time(sec)'),ylabel('ddq2(deg/s^2)')
%---------------tird link--------------------------------------------------
figure
subplot(3,1,1),plot(time,Q3,'black'),ylabel('q3(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ3,'black'),ylabel('dq3(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ3,'black'),xlabel('time(sec)'),ylabel('ddq23(deg/s^2)')
%---------------forth link-------------------------------------------------
figure
subplot(3,1,1),plot(time,Q4,'black'),ylabel('q4(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ4,'black'),ylabel('dq4(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ4,'black'),xlabel('time(sec)'),ylabel('ddq4(deg/s^2)')
%---------------fifth link-------------------------------------------------
figure
subplot(3,1,1),plot(time,Q5,'black'),ylabel('q5(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ5,'black'),ylabel('dq5(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ5,'black'),xlabel('time(sec)'),ylabel('ddq5(deg/s^2)')
%----------------sixth link------------------------------------------------
figure
subplot(3,1,1),plot(time,Q6,'black'),ylabel('q6(degree)'),xlabel('time(sec)')
subplot(3,1,2),plot(time,DQ6,'black'),ylabel('dq6(deg/s)'),xlabel('time(sec)')
subplot(3,1,3),plot(tt,DDQ6,'black'),xlabel('time(sec)'),ylabel('ddq6(deg/s^2)')
 toc