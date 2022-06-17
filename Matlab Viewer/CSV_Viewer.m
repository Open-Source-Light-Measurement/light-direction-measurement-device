#Aktuelle Fehler zu beheben:
#der Winkel 0 und 180 wird doppelt gemessen, im Python Code anpassen!!
#es ist eine kleine Kippung drin, hab ich auch beim Drehprozess gesehen, im Python Code anpassen

clear all; close all;
x = csvread ('Sensordaten.csv')
#E1=Eoben
E1=x(:,1)
E1(1)=[];
E1(1:11:end)=[];
#E2=Eunten
E2=x(:,2)
E2(1)=[];
E2(1:11:end)=[];
#Achse 1 = unklar
theta=x(:,4)
theta(1)=[];
theta(1:11:end)=[];
#Achse 2 = phi
phi=x(:,3)
phi(1)=[];
phi(1:11:end)=[];
#phi=90-phi
#Muss ich klären, ob rad oder deg default sind

#E2-E1 oder umgedreht?
E=E1-E2


Zeilen = size(E,1);
#Rotationsmatix

B = [];

 for i=1:Zeilen  
   #zu rotierende Koordinate
   x=[0;0 ;E(i)]
   
   #Gewünschter Rotationswinkel
   Winkel=phi(i)

#Rotation um y-Achse
#Winkel in Grad eingeben nicht rad
  T=roty(Winkel)

#Berechnung der rotierten Koordinate
  v=T*x;
  B = [B , v];
end

C = [];

 for i=1:Zeilen  
   
   
   #Gewünschter Rotationswinkel
   Winkel=theta(i)

#Rotation um y-Achse
#Winkel in Grad eingeben nicht rad
  T=rotx(Winkel)

#Berechnung der rotierten Koordinate
  v=T*B(:,i);
  C = [C , v];
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

#[xq,yq] = meshgrid(-3000:100:3000, -3000:100:3000);
#vq = griddata(x,y,z,xq,yq);
#figure(2)
#mesh(xq,yq,vq)
#colorbar
#surf (vq)
