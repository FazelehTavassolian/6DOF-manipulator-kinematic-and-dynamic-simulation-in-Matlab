load QQ
syms q1 q2 q3 q4 q5 q6 real
l1=1;l2=1;d4=1;d6=1;
%------------------- Denavit-Hartenberg Convention-----------------
teta=[q1 q2 q3 q4 q5 q6];
alpha=[pi/2,0,pi/2,-pi/2,pi/2,0];
a=[l1,l2,0,0,0,0];
d=[0,0,0,d4,0,d6];
for i=1:6
     A=[cos(teta(i))   -sin(teta(i))*cos(alpha(i))   sin(teta(i))*sin(alpha(i))    a(i)*cos(teta(i));
        sin(teta(i))   cos(teta(i))*cos(alpha(i))    -cos(teta(i))*sin(alpha(i))   a(i)*sin(teta(i));
        0              sin(alpha(i))                 cos(alpha(i))                 d(i);
        0              0                             0                             1]; 
    switch(i)
        case 1,A1=A;
        case 2,A2=A;
        case 3,A3=A;
        case 4,A4=A;
        case 5,A5=A;
        case 6,A6=A;     
    end
end
%-----------------Transformation matrix--------------
t12=(A1*A2);
t13=(t12*A3);
t14=(t13*A4);
t15=(t14*A5);
t16=(t15*A6);
 
M=moviein(12);
incr=0;
for i=1:length(Q1)
o0=[0 0 0];
o1=[0 0 l1];
o2=o1+subs(t12(1:3,4)',{q1,q2,q3,q4,q5,q6},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)});
o3=o2+subs(t13(1:3,4)',{q1,q2,q3,q4,q5,q6},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)});
o4=o3+subs(t14(1:3,4)',{q1,q2,q3,q4,q5,q6},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)});
o5=o4+subs(t15(1:3,4)',{q1,q2,q3,q4,q5,q6},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)});
o6=o5+subs(t16(1:3,4)',{q1,q2,q3,q4,q5,q6},{Q1(i),Q2(i),Q3(i),Q4(i),Q5(i),Q6(i)});
 plot3([0,o1(1)],[0,o1(2)],[0,o1(3)],'m',[o1(1),o2(1)],[o1(2),o2(2)],[o1(3),o2(3)],'green',...
       [o2(1),o3(1)],[o2(2),o3(2)],[o2(3),o3(3)],'blue',...
       [o3(1),o4(1)],[o3(2),o4(2)],[o3(3),o4(3)],'red',[o4(1),o5(1)],[o4(2),o5(2)],[o4(3),o5(3)],...
       [o5(1),o6(1)],[o5(2),o6(2)],[o5(3),o6(3)],'green','LineWidth',4)
   xlim([-8 1]);
   ylim([-8 1]);
   zlim([0 8]);
   grid
   incr=incr+1;
   M(:,incr)=getframe;
end
movie(M)
movie2avi(M,'sixr.avi')
   

