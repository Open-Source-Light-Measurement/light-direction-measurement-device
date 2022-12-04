#This Code uses:
# Matt J (2022). 3D Rotation about Shifted Axis (https://www.mathworks.com/matlabcentral/fileexchange/30864-3d-rotation-about-shifted-axis), MATLAB Central File Exchange. Retrieved December 4, 2022.

#Aktuelle Fehler zu beheben:
#es ist eine kleine Kippung drin, hab ich auch beim Drehprozess gesehen, im Python Code anpassen

clear all; close all;
x = csvread ('Sensordaten.csv');
#letzte Messreihe mit 180 Grad gel�scht, damit Doppeldarstellung vermieden wird
x(112:122,:) = [];
#Spaltenvektoren erstellen
#in dem Datensatz wird immer der erste Wert bei 0Grad gel�scht

#E1=Eoben
E1=x(:,1)
E1(1)=[];
E1(1:11:end)=[];

#E2=Eunten
E2=x(:,2)
E2(1)=[];
E2(1:11:end)=[];

#Achse 2 (theta) = Rotation um X-Achse
theta=x(:,4)
theta(1)=[];
theta(1:11:end)=[];

#Achse 1 (phi) = Rotation um Y-Achse
phi=x(:,3)
phi(1)=[];
phi(1:11:end)=[];
#Sensor dreht in neg. Richtungssinn (w�hrend der Messung)
phi=phi.*(-1)


#E=E2-E1 (es darf kein Betrag genommen werden, da sonst Richtungssinn nicht simmt)
E=E1-E2


Zeilen = size(E,1);
#Rotationsmatix

C = [];


 for i=1:Zeilen
   #zu rotierende Koordinate
   d=[0;0 ;E(i)]

   #Gew�nschter Rotationswinkel phi(i)

#Rotation um y-Achse
#Winkel in Grad eingeben nicht rad
  T=roty(phi(i))

#Berechnung der rotierten Koordinate
  v=T*d;

#Eingaben f�r AxelRot
#Verschiebung der Gerade
# in meinem Fall geht die immer durch den Nullpunkt
x0=[0;0;0]

#Rotationswinkel
#Richtungsvektor entspricht in meinem Fall der X-Achse des Sensors
#m=[1;1;0] entspricht einer steigenden Geraden im 45Grad Winkel in XY-Ebene

#phi muss hier pos. sein (*-1) weil es oben neg. gemacht wurde, sonst ist der Richtungsvektor falsch
j=1

if phi(i)== -90
  j=0
  k=1
else
  k=tan(deg2rad(-phi(i)))*j
end

m=[j;0;k]

#Funktion aus Mathwork heruntergeladen
M=AxelRot(theta(i), m ,x0)
M(:,4)=[]
M(4,:)=[]

#rotierter Vektor rot
B=M*v

#_______________________

  C = [C , B];
end



C=C'

x=C(:,1)
y=C(:,2)
z=C(:,3)

u=0
u=repmat(u,Zeilen,1);
v=0
v=repmat(v,Zeilen,1);
w=0
w=repmat(w,Zeilen,1);

figure(1)
quiver3 (u, v, w, x, y, z)

figure(2)
scatter3(x,y,z)

figure(3)
hist(E)

