%%%% derive passive walker with round feet
%%% Pranav A. BHounsule, 21 April 2009. pab47@cornell.edu
clc;
clear all;

syms q1 q2 real %Angles as defined in figures 
syms u1 u2 real %Angular velocity
syms ud1 ud2 real%Angular Acceleration
syms gam g real %slope of ramp,, gravity
syms c w l r real % Distances as defined in figures
syms M m I real%Mass Hip, leg, Inertia
syms xp1 real
syms Th real%Th motor torques
syms v1 v2 real %angles before heelstrike
syms r1 r2 real %velocities before heelstrike

i=[1 0 0];
j=[0 1 0];
k=[0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Reference Frames                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X1 = sin(q1)*i-cos(q1)*j;
Y1 = cos(q1)*i+sin(q1)*j;

X2 = sin(q1-q2)*i - cos(q1-q2)*j; 
Y2 = cos(q1-q2)*i + sin(q1-q2)*j;

J  = -sin(gam)*i+cos(gam)*j; %Fictitious ramp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Original position vectors
r_H_G1 = c*X1 + w*Y1;
r_H_G2 = c*X2 + w*Y2;
r_P1_H = r*j - l*X1;
r_P2_H = r*j - l*X2;

%Derived position vectors
r_P1_G1 = r_P1_H + r_H_G1;
r_P1_G2 = r_P1_H + r_H_G2;
r_P1_P2 = r_P1_H - r_P2_H;

r_H_P2 = -r_P2_H;

%Position vectors for heelstrike
r_P2_G1 = r_P2_H + r_H_G1;
r_P2_G2 = r_P2_H + r_H_G2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        Angular Velocities and Accelerations       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

om1 = u1*k;
om2 = (u1-u2)*k;
 
al1 = ud1*k;
al2 = (ud1-ud2)*k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Constraints, Linear Velocities and Accelerations %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = [q1; q2];
u = [u1; u2];
ud = [ud1; ud2];

xh = -l*sin(q1) - r*q1 + xp1;
yh =  l*cos(q1) + r;
    
    xhdot  = jacobian(xh,q)*u;
    xhddot = jacobian(xhdot,q)*u + jacobian(xhdot,u)*ud;
    yhdot  = jacobian(yh,q)*u;
    yhddot = jacobian(yhdot,q)*u + jacobian(yhdot,u)*ud;    

v_H  = xhdot*i+yhdot*j; 

a_H  = xhddot*i+yhddot*j;

v_G1 = v_H +cross(om1,r_H_G1);
v_G2 = v_H +cross(om2,r_H_G2);

a_G1 = a_H +cross(om1,cross(om1,r_H_G1))+cross(al1,r_H_G1);
a_G2 = a_H +cross(om2,cross(om2,r_H_G2))+cross(al2,r_H_G2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        Collision Condition (Events)               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CD = dot(r_P1_P2,j);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Change of Angular Momentum & External Moments    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

M_P1    = dot(m*g*cross(r_P1_G1,-J)+m*g*cross(r_P1_G2,-J)+M*g*cross(r_P1_H,-J),k);
Hdot_P1 = dot(m*cross(r_P1_G1,a_G1)+m*cross(r_P1_G2,a_G2)+M*cross(r_P1_H,a_H)+I*(al1+al2),k);


M_H    = dot(m*g*cross(r_H_G2,-J)+Th*k,k); %Th
Hdot_H = dot(m*cross(r_H_G2,a_G2)+I*al2,k);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Angular Momentum, (Before and After Heelstrike  % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

H_P1p = dot(m*cross(r_P1_G1,v_G1)+m*cross(r_P1_G2,v_G2)+M*cross(r_P1_H,v_H)+I*(om1+om2),k);
H_P2n = subs(dot(m*cross(r_P2_G2,v_G2)+m*cross(r_P2_G1,v_G1)+M*cross(r_P2_H,v_H)+I*(om1+om2),k),...
             {u1,u2,q1,q2},{v1,v2,r1,r2});

H_Hp = dot(m*cross(r_H_G2,v_G2)+I*om2,k);
H_Hn = subs(dot(m*cross(r_H_G1,v_G1)+I*om1,k),... 
            {u1,u2,q1,q2},{v1,v2,r1,r2}); 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   EOM:[M-Hdot=0](Single Stance and Double Stance)  % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AMB_P1 = M_P1-Hdot_P1;
AMB_H  = M_H-Hdot_H;


%%% Single stance equations %%%
    RHSs1 = -subs(AMB_P1,[ud1 ud2],[0 0]); 
    Ms11  =  subs(AMB_P1,[ud1 ud2],[1 0]) + RHSs1;
    Ms12  =  subs(AMB_P1,[ud1 ud2],[0 1]) + RHSs1;

    RHSs2 = -subs(AMB_H,[ud1 ud2],[0 0]); 
    Ms21  =  subs(AMB_H,[ud1 ud2],[1 0]) + RHSs2;
    Ms22  =  subs(AMB_H,[ud1 ud2],[0 1]) + RHSs2;
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heelstrike:[H+ = H-](Single Stance and Double Stance) %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%  Heelstrike: H+
AM_P1 = H_P1p;
AM_H  = H_Hp;

%%%% Heelstrike: H-, Transition to Single stance 
    RHSh1 = H_P2n;
    RHSh2 = H_Hn;

    Mh11  =  subs(AM_P1,[u1 u2],[1 0]);
    Mh12  =  subs(AM_P1,[u1 u2],[0 1]);
    
    Mh21  =  subs(AM_H,[u1 u2],[1 0]);
    Mh22  =  subs(AM_H,[u1 u2],[0 1]);   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Energies                                   %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Get required positions of masses
%wrt ramp frame
xH = xh - xp1;
yH = yh;
xG1 = xH+dot(r_H_G1,i);
yG1 = yH+dot(r_H_G1,j);
xG2 = xH+dot(r_H_G2,i);
yG2 = yH+dot(r_H_G2,j);

%wrt to global frame
Y_H = yH*cos(gam) - xH*sin(gam);
Y_G1 = yG1*cos(gam) - xG1*sin(gam); 
Y_G2 = yG2*cos(gam) - xG2*sin(gam);
X_H = xH*cos(gam) + yH*sin(gam);
X_G1 = xG1*cos(gam) + yG1*sin(gam);
X_G2 = xG2*cos(gam) + yG2*sin(gam);


KE = 0.5*(simplify(m*dot(v_G1,v_G1) + m*dot(v_G2,v_G2) + M*dot(v_H,v_H) + I*(dot(om1,om1) + dot(om2,om2))));
PE = simplify(m*g*Y_G1+m*g*Y_G2+M*g*Y_H);
TE = simplify(KE + PE);
DTE = jacobian(TE,q)*u + jacobian(TE,u)*ud;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Ground Reaction Forces                     %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = m*(a_G1+a_G2) + M*a_H + (2*m+M)*g*j;
Rx = dot(R,i); %Reaction from front foot on the ground in x direction
Ry = dot(R,j); %Reaction from front foot on the ground in y direction 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      Writing the right hand side automatically        %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Write single_stance.m %%%

fid=fopen(   'single_stance.m','w');
fprintf(fid, 'function zdot=single_stance(t,z,flag,GL_DIM)  \n\n');

fprintf(fid, 'q1 = z(1);   u1 = z(2);                         \n');
fprintf(fid, 'q2 = z(3);   u2 = z(4);                         \n');
fprintf(fid, 'xh = z(6);  vxh = z(7);                       \n');
fprintf(fid, 'yh = z(8);  vyh = z(9);                     \n\n');

fprintf(fid, 'M = walker.M;  m = walker.m; I = walker.I;  \n');
fprintf(fid, 'l = walker.l;  c = walker.c; w = walker.w;    \n');
fprintf(fid, 'r = walker.r;  g = walker.g; gam = walker.gam; \n\n');

fprintf(fid, 'Th=0;                      \n\n');

fprintf(fid,'M11 = %s; \n', char(simplify(Ms11)) );
fprintf(fid,'M12 = %s; \n', char(simplify(Ms12)) );
fprintf(fid,'\n');

fprintf(fid,'M21 = %s; \n', char(simplify(Ms21)) );
fprintf(fid,'M22 = %s; \n', char(simplify(Ms22)) );
fprintf(fid,'\n');

fprintf(fid,'RHS1 = %s; \n', char(simplify(RHSs1)) );
fprintf(fid,'RHS2 = %s; \n', char(simplify(RHSs2)) );
fprintf(fid,'\n');


fprintf(fid,'MM = [M11 M12;                               \n');
fprintf(fid,'     M21 M22];                               \n\n');

fprintf(fid,'RHS = [RHS1; RHS2];                       \n\n');

fprintf(fid,'X = MM \\ RHS;                                    \n\n');

fprintf(fid,'ud1 = X(1);                                       \n');
fprintf(fid,'ud2 = X(2);                                       \n\n');

fprintf(fid,'DTE = %s; \n', char(simplify(DTE)) );
fprintf(fid,'axh = %s; \n', char(simplify(xhddot)) );
fprintf(fid,'ayh = %s; \n', char(simplify(yhddot)) );
fprintf(fid,'\n');

fprintf(fid,'zdot = [u1 ud1 u2 ud2 ...           \n'); 
fprintf(fid,'        DTE vxh axh vyh ayh]'';                   \n');

fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Write heelstrike.m %%%
%%% Heelstrike transitioning to single stance

fid=fopen(   'heelstrike.m','w');
fprintf(fid, 'function zplus=heelstrike(t,z,GL_DIM)      \n\n');

fprintf(fid, 'r1 = z(1);   v1 = z(2);                         \n');
fprintf(fid, 'r2 = z(3);   v2 = z(4);                         \n');
fprintf(fid, 'xh = z(6);   yh = z(8);                       \n\n'); 

fprintf(fid, 'q1 = r1 - r2;                         \n');
fprintf(fid, 'q2 = -r2;                                       \n\n');

fprintf(fid, 'M = walker.M;  m = walker.m; I = walker.I;  \n');
fprintf(fid, 'l = walker.l;  c = walker.c; w = walker.w;    \n');
fprintf(fid, 'r = walker.r;  g = walker.g; gam = walker.gam; \n\n');

fprintf(fid,'M11 = %s; \n', char(simplify(Mh11)) );
fprintf(fid,'M12 = %s; \n', char(simplify(Mh12)) );
fprintf(fid,'\n');

fprintf(fid,'M21 = %s; \n', char(simplify(Mh21)) );
fprintf(fid,'M22 = %s; \n', char(simplify(Mh22)) );
fprintf(fid,'\n');

fprintf(fid,'RHS1 = %s; \n', char(simplify(RHSh1)) );
fprintf(fid,'RHS2 = %s; \n', char(simplify(RHSh2)) );
fprintf(fid,'\n');

fprintf(fid,'MM = [M11 M12;                               \n');
fprintf(fid,'     M21 M22];                               \n\n');

fprintf(fid,'RHS = [RHS1; RHS2];                      \n\n');

fprintf(fid,'X = MM \\ RHS;                                    \n\n');

fprintf(fid,'u1 = X(1);                                       \n');
fprintf(fid,'u2 = X(2);                                      \n\n');

fprintf(fid,'TE = %s; \n',  char(simplify(TE)) );
fprintf(fid,'vxh = %s; \n', char(simplify(xhdot)) );
fprintf(fid,'vyh = %s; \n', char(simplify(yhdot)) );
fprintf(fid,'\n');

fprintf(fid,'zplus = [q1 u1 q2 u2 TE xh vxh yh vyh];               \n'); 

fclose(fid);

%%%%%%%%%%%%%%%
disp('Files written');