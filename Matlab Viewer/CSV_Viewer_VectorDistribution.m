clear all; 
close all;
q = csvread ('Sensordaten.csv');

q(112:122,:) = [];

#E1=Eoben
E1=q(:,1);
E1(1)=[];
E1(1:11:end)=[];

#E2=Eunten
E2=q(:,2);
E2(1)=[];
E2(1:11:end)=[];

#Achse 2 (theta) = Rotation um X-Achse
theta=q(:,4);
theta(1)=[];
theta(1:11:end)=[];

#Achse 1 (phi) = Rotation um Y-Achse
phi=q(:,3);
phi(1)=[];
phi(1:11:end)=[];
#Sensor dreht in neg. Richtungssinn (während der Messung)
phi=phi.*(-1);

E=E1-E2;

Zeilen = size(E,1);
#Rotationsmatix

C = [];


 for i=1:Zeilen ;
   #zu rotierende Koordinate
   d=[0;0 ;E(i)];
   
#Rotation um y-Achse

#roty geht nur von -90 bis +90 Grad meine Winkel laufen aber von -0 bis -180 deshalb muss ab -90 angepasst werden
  if phi(i) < -90
    d=d*-1
    phiy(i)=phi(i)+180
    T=roty(phiy(i));
  else
    phi(i) = phi(i)
    T=roty(phi(i));
  endif

#Berechnung der rotierten Koordinate
  v=T*d;

#Eingaben für AxelRot
#Verschiebung der Gerade
#geht die immer durch den Nullpunkt
  x0=[0;0;0];

#Rotationswinkel
#Richtungsvektor entspricht der X-Achse des Sensors

#bei Winkeln von tan größer 90 Grad ändert sich die Verktorrichtung von m, das wird durch neg Vorzeichen korrigiert
  if phi(i)< -90;
    theta(i)=theta(i)*-1;
    k=tan(deg2rad(-phiy(i)))*j;
  else
    theta(i)=theta(i);
    k=tan(deg2rad(-phi(i)))*j;
  endif
  
  j=1;
  m=[j;0;k];

  #Funktion von Mathworks
  #Matt J (2023). 3D Rotation about Shifted Axis (https://www.mathworks.com/matlabcentral/fileexchange/30864-3d-rotation-about-shifted-axis), MATLAB Central File Exchange. Retrieved March 24, 2023.
  M=AxelRot(theta(i), m ,x0);
  M(:,4)=[];
  M(4,:)=[];

  #rotierter Vektor rot
  B=M*v ;

  C = [C , B];
end



C=C';

x=C(:,1);
y=C(:,2);
z=C(:,3);

u=0;
u=repmat(u,Zeilen,1);
v=0;
v=repmat(v,Zeilen,1);
w=0;
w=repmat(w,Zeilen,1);

figure(1)
quiver3 (u, v, w, x, y, z)
title ("Light Vector Distribution");
xlabel ("E in lux");
ylabel ("E in lux");
zlabel ("E in lux");
grid minor on


figure(2)
scatter3(x,y,z)

figure(3)
hist(E)


#Zain Mecklai (2023). Making Surface Plots From Scatter Data (https://www.mathworks.com/matlabcentral/fileexchange/5105-making-surface-plots-from-scatter-data), MATLAB Central File Exchange. Retrieved January 18, 2023.
%% Making Surface Plots From Scatter Data

figure(4)
plot3(x,y,z,'.-')
title ("Light Vector Distribution");
xlabel ("E in lux");
ylabel ("E in lux");
zlabel ("E in lux");
grid minor on

%% Little triangles

tri = delaunay(x,y);
figure(5)
plot(x,y,'.')

% How many triangles are there?

[r,c] = size(tri);
disp(r)

%% Plot it with TRISURF

h = trisurf(tri, x, y, z);
axis vis3d

#Hüllkurve
figure(6)
K = convhulln(C);
h2=trisurf(K,x,y,z)
title ("Light Vector Distribution");
xlabel ("E in lux");
ylabel ("E in lux");
zlabel ("E in lux");
