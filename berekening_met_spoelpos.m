l1 = 253.5; % length of first arm
l2 = 253.5; % length of second arm
l3 = 252;   % lenth of thirth arm

X = 100; %dit is de Xcoordinaat om in te vullen
Z = 500; %dit is de Zcoordinaat om in te vullen
phi = 60; %dit is de hoek van de spoel

%XD = 0:10:X; 
%ZD = (Z-X):10:Z;
%Data = [];
theta1 = 0:0.03422:180; %180 bewegings vrijheid van de schouder met stapgroote van 1 encoder count
theta2 = 0:0.06844:331; %331/2=165.5 bewegings vrijheid van de elleboog met stapgroote van 1 encoder count
theta3 = 0:0.10267:220; %220 bewegings vrijheid van de yaw met stapgroote van 1 encoder count
Laxis = 800; %lengte van assen in figuur


xcor = l3*cos(degtorad(phi));
zcor = l3*sin(degtorad(phi)); %dit is de correctie die van de oorspronkelijke coordinaten af gaat om pos van de pols te achterhalen

xpols = X - xcor;
zpols = Z - zcor;
%theta1D = [];
%theta2D = [];
%xz = [];

disp('xpols');disp(xpols)
disp('zpols');disp(zpols)

c2 = (abs(xpols)^2 + abs(zpols)^2);
L12 = sqrt(c2);
Theta2= 2*radtodeg(asin((L12/2)/l1));
Theta10= radtodeg(asin(zpols/L12));
Theta11= radtodeg(acos((L12/2)/l1));
if X<0
    Theta1= 90+radtodeg(asin(abs(xpols/L12)))+Theta11;
elseif X>=0
    Theta1= abs(Theta10) +abs(Theta11);
end
z1 = l1 * sin(degtorad(Theta1));
x1 = l1 * cos(degtorad(Theta1));
Theta3 = 360-(Theta1 + Theta2) + phi;

%disp('c2');disp(c2)
%disp('L12');disp(L12)
disp('theta1');disp(Theta1)
disp('theta2');disp(Theta2)
disp('theta3');disp(Theta3)
encoder1 = round(Theta1/0.03422);
encoder2 = round(Theta2/0.06844);
encoder3 = round(Theta3/0.10267);
disp('encoder1');disp(encoder1)
disp('encoder2');disp(encoder2)
disp('encoder3');disp(encoder3)

%disp('theta10');disp(Theta10)
%disp('theta11');disp(Theta11)


plot(xpols,zpols,'-o','LineWidth',4);
hold on
plot(0,0,'-o','LineWidth',4);
plot(X,Z,'-o','LineWidth',4);
plot(x1,z1,'-o','LineWidth',4);
axis([-Laxis,Laxis,-Laxis,Laxis]);
grid on;
hold off
pause(0.0001);


