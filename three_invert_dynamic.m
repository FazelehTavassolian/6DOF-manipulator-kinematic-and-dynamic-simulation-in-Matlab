format compact;
tic
%----------forward kinematic-------------------
q1=sym('q1','real');
q2=sym('q2','real');
q3=sym('q3','real');
q4=sym('q4','real');
q5=sym('q5','real');
q6=sym('q6','real');
syms m1 m2 m3 m4 m5 m6 t l1 l2 l3 l4 l5 l6 d4 d6
syms dq1 dq2 dq3 dq4 dq5 dq6 
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 t G
%----------------jacobian &Transformation matrix-----------------
% load j
% load A
% load T
%-----------------Rotation matrix--------------
R1=A1(1:3,1:3)
R2=t12(1:3,1:3)
R3=t13(1:3,1:3)
R4=t14(1:3,1:3)
R5=t15(1:3,1:3)
R6=t16(1:3,1:3)
%----------------- inertia matris ------------
I1=[m1*l1^2/12+m1*(l1/2)^2 0 0;0 m1*l1^2/12+m1*(l1/2)^2 0;0 0 m1*l1^2/12+m1*(l1/2)^2]
I2=[m2*l2^2/12+m2*(l2/2)^2 0 0;0 m2*l2^2/12++m2*(l2/2)^2 0;0 0 m2*l2^2/12+m2*(l2/2)^2]
I3=[m3*l3^2/12++m3*(l3/2)^2 0 0;0 m3*l3^2/12+m3*(l3/2)^2 0;0 0 m3*l3^2/12+m3*(l3/2)^2]
I4=[m4*l4^2/12++m4*(l4/2)^2 0 0;0 m4*l4^2/12+m4*(l4/2)^2 0;0 0 m4*l4^2/12+m4*(l4/2)^2] 
I5=[m5*l5^2/12+m5*(l5/2)^2 0 0;0 m5*l5^2/12+m5*(l5/2)^2 0;0 0 m5*l5^2/12+m5*(l5/2)^2] 
I6=[m6*l6^2/12+m6*(l6/2)^2 0 0;0 m6*l6^2/12+m6*(l6/2)^2 0;0 0 m6*l6^2/12+m6*(l6/2)^2] 
%---------------- potential energy ----------
P1=m1*G*l1;
P2=m2*G*(l1+l2*sin(q2));
P3=m3*G*(l1+l2*sin(q2)+l3*sin(q2+q3));
P4=m4*G*(l1+l2*sin(q2)+(l3+l4)*sin(q2+q3));
P5=m5*G*(l1+l2*sin(q2)+(l3+l4)*sin(q2+q3)...
    +l5*sin(q2+q3+q5));
P6=m6*G*(l1+l2*sin(q2)+(l3+l4)*sin(q2+q3)...
    +(l5+l6)*sin(q2+q3+q5));
P=P1+P2+P3+P4+P5+P6
% %-----------------q,dq,ddq-----------------
q=[q1 q2 q3 q4 q5 q6];
dq=[dq1 dq2 dq3 dq4 dq5 dq6];
ddq=[ddq1 ddq2 ddq3 ddq4 ddq5 ddq6];
% %---------------- inertia matris D ------------ 
 D=zeros(6,6);
m=[m1 m2 m3 m4 m5 m6];
I=[I1 I2 I3 I4 I5 I6];
jv=[jv1 jv2 jv3 jv4 jv5 jv6];
jw=[jw1 jw2 jw3 jw4 jw5 jw6];
R=[R1 R2 R3 R4 R5 R6];
for i=1:6
    ts=(i*6)-5;
    tf=i*6;
    D =D+ m(i)*transpose(jv(1:3,ts:tf))*jv(1:3,ts:tf)+transpose(jw(1:3,ts:tf))...
        *R(1:3,(i*3-2):(i*3))*I(1:3,(i*3-2):(i*3))*transpose(R(1:3,(i*3-2):(i*3)))*jw(1:3,ts:tf);
end
D
%---------------- christoffel symbolsz----------
c=zeros(6,6);
for j=1:6
    for k=1:6
        c=0;
        for i=1:6
             c=c+(jacobian(D(k,j),q(j))+jacobian(D(k,i),...
                 q(j))-jacobian(D(i,j),q(j)))*dq(i)/2;
        end
         C(k,j)=c;
    end
end
%C
%---------------- gravity vector--------------
 g=jacobian(P,q)'
% ----------Euler_lagrange equations -----
for k=1:6
    for j=1:6
        for i=1:6
            L(k)=sum(D(k,j)*transpose(ddq(j)))+...
                sum(C(k,j)*transpose(dq(i))*transpose(dq(j))+g(k));
        end
    end
end
torque_1=L(1) 
torque_2=L(2)  
torque_3=L(3)
torque_4=L(4)  
torque_5=L(5)  
torque_6=L(6)
% % ---------solve difrential equations-----
data={l1,l2,l3,l4,l5,l6,m1,m2,m3,m4,m5,m6,d4,d6,G};
datn={1,1,1,1,1,1,1,1,1,1,1,1,1,.8,9.806};
lagr1=subs(torque_1,data,datn);
lagr2=subs(torque_2,data,datn);
lagr3=subs(torque_3,data,datn);
lagr4=subs(torque_4,data,datn);
lagr5=subs(torque_5,data,datn);
lagr6=subs(torque_6,data,datn);
time=1:49;
TORQUE_T=[lagr1;lagr2;lagr3;lagr4;lagr5;lagr6];
TORQUE1=[];TORQUE2=[];TORQUE3=[];TORQUE4=[];TORQUE5=[];TORQUE6=[];
    for i=1:49
        q={ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,dq1,dq2,dq3,dq4,dq5,dq6,q1,q2,q3,q4,q5,q6};
        Q={DDQ1(i),DDQ2(i),DDQ3(i),DDQ4(i),DDQ5(i),DDQ6(i),DQ1(i),DQ2(i),DQ3(i)...
            ,DQ4(i),DQ5(i),DQ6(i),Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)};
        TORQUE=subs(TORQUE_T,q,Q);
        TORQUE1=[TORQUE1 TORQUE(1)];
        TORQUE2=[TORQUE2 TORQUE(2)];
        TORQUE3=[TORQUE3 TORQUE(3)];
        TORQUE4=[TORQUE4 TORQUE(4)];
        TORQUE5=[TORQUE5 TORQUE(5)];
        TORQUE6=[TORQUE6 TORQUE(6)];
    end
      figure,plot(time,TORQUE1),xlabel('time(s)'),ylabel('TORQUE 01(N.m)')
      figure,plot(time,TORQUE2),xlabel('time(s)'),ylabel('TORQUE 12(N.m)')
      figure,plot(time,TORQUE3),xlabel('time(s)'),ylabel('TORQUE 23(N.m)')
      figure,plot(time,TORQUE4),xlabel('time(s)'),ylabel('TORQUE 34(N.m)')
       figure,plot(time,TORQUE5),xlabel('time(s)'),ylabel('TORQUE 45(N.m)')
      figure,plot(time,TORQUE6),xlabel('time(s)'),ylabel('TORQUE 56(N.m)')
      toc