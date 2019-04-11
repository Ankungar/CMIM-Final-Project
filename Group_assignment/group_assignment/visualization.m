%% Visualization of the plots
%This code is used to visualize the three bodies in the threebodiesrun.m
%file. First it loads the Q matrix saved from the previously mentioned
%m-file and plots the movement of the three bodies.

load Q.mat

X1cp = Q(:,4); %x-values of bar 1
Y1cp = Q(:,5); %y-values of bar 1

X2cp = Q(:,7); %x-values of bar 2
Y2cp = Q(:,8); %y-values of bar 2

X3cp = Q(:,10); %x-values of bar 3
Y3cp = Q(:,11); %y-values of bar 3

ep1 = [0,0]; %zero coordinates

p1 = []; p2 = []; p3 = []; %define plots

%% Start plot
figure
hold on
axis([-0.05 0.105 -0.05 0.12])
plot(0,0,'k*','LineWidth',2) %Set the fixed points
plot(0.1,0,'k*','LineWidth',2)

for i = 1:length(X1cp)
    [X1 Y1] = visual([X1cp(i), Y1cp(i)],ep1);
    ep2 = [X1(2),Y1(2)];
    [X2 Y2] = visual([X2cp(i), Y2cp(i)],ep2);
    ep3 = [X2(2),Y2(2)];
    [X3 Y3] = visual([X3cp(i), Y3cp(i)],ep3);
    
    delete(p1);
    delete(p2);
    delete(p3);

    p1 = plot(X1, Y1,'b','LineWidth',3);
    %c1 = plot(X1cp(i),Y1cp(i),'b*','LineWidth',2);
    p2 = plot(X2, Y2,'b','LineWidth',3);
    %c2 = plot(X2cp(i),Y2cp(i),'r*','LineWidth',2);
    p3 = plot(X3, Y3,'b','LineWidth',3);
    %c3 = plot(X3cp(i),Y3cp(i),'y*','LineWidth',2);
    pause(0.01);

end
hold off