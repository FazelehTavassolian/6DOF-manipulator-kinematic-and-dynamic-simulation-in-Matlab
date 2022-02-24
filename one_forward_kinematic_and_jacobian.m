clc;clear all;format compact;
tic
%----------forward kinematic-------------------
syms q1 q2 q3 q4 q5 q6 real
syms l1 l2 l3 l4 l5 l6 d4 d6
%------------------- Denavit-Hartenberg Convention-----------------
teta=[q1 q2 q3 q4 q5 q6];
alpha=[90,0,90,-90,90,0];
a=[l1,l2,0,0,0,0];
d=[0,0,0,d4,0,d6];
for i=1:6
     A=[cos(teta(i))   -sin(teta(i))*cosd(alpha(i))   sin(teta(i))*sind(alpha(i))    a(i)*cos(teta(i));
        sin(teta(i))   cos(teta(i))*cosd(alpha(i))    -cos(teta(i))*sind(alpha(i))   a(i)*sin(teta(i));
        0              sind(alpha(i))                 cosd(alpha(i))                 d(i);
        0              0                             0                             1]; 
    switch(i)
        case 1,A1=A
        case 2,A2=A
        case 3,A3=A
        case 4,A4=A
        case 5,A5=A
        case 6,A6=A     
    end
end
%-----------------Transformation matrix--------------
disp('Transformation matrix ')
t12=(A1*A2)
t13=(t12*A3)
t14=(t13*A4)
t15=(t14*A5)
t16=(t15*A6)
%-----------------jacobian parameter---------------
disp('jacobian parameter ')
z0=[0 0 1]
z1=A1(1:3,3)'
z2=t12(1:3,3)'
z3=t13(1:3,3)'
z4=t14(1:3,3)'
z5=t15(1:3,3)'
o0=[0 0 0]
o1=A1(1:3,4)'
o2=t12(1:3,4)'
o3=t13(1:3,4)'
o4=t14(1:3,4)'
o5=t15(1:3,4)'
o6=t16(1:3,4)'
%-----------------first link---------------
disp(' jocobian for links ')
jv1=[cross(z0,(o1-o0));zeros(5,3)]'
jw1=[z0;zeros(5,3)]'
%----------------second link-------------- 
jv2=[cross(z0,(o2-o0));cross(z1,(o2-o1));zeros(4,3)]'
jw2=[z0;z1;zeros(4,3)]'
%----------------tird link--------------
jv3=[cross(z0,(o3-o0));cross(z1,(o3-o1));cross(z2,(o3-o2));zeros(3,3)]'
jw3=[z0;z1;z2;zeros(3,3)]'
%----------------fourth link--------------
jv4=[cross(z0,(o4-o0));cross(z1,(o4-o1));cross(z2,(o4-o2));...
    cross(z3,(o4-o3));zeros(2,3)]'
jw4=[z0;z1;z2;z3;zeros(2,3)]'
%----------------fifth link--------------
jv5=[cross(z0,(o5-o0));cross(z1,(o5-o1));cross(z2,(o5-o2));...
    cross(z3,(o5-o3));cross(z4,(o5-o4));zeros(1,3)]'
jw5=[z0;z1;z2;z3;z4;zeros(1,3)]'
%----------------sixth link--------------
jv6=[cross(z0,(o6-o0));cross(z1,(o6-o1));...
    cross(z2,(o6-o2));cross(z3,(o6-o3));cross(z4,(o6-o4));cross(z5,(o6-o5))]'
jw6=[z0;z1;z2;z3;z4;z5]'
toc
j=[jv6;jw6 ];
% save('A','A1','A2','A3','A4','A5','A6');
% save('T','t12','t13','t14','t15','t16');
% save('j','jv1','jw1','jv2','jw2','jv3','jw3','jv4','jw4','jv5','jw5','jw6','jv6');