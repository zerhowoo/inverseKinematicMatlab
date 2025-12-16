function p1=Rodrigues(p,v,theta)
kt=v/norm(v);
kx=kt(1);ky=kt(2);kz=kt(3);
K=[0 -kz ky;
   kz 0 -kx;
   -ky kx 0];
R = eye(3) + sind(theta)*K + (1- cosd(theta))*K*K;
p1= R*p';
p1=p1';