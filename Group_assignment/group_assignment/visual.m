function [x y] = visual(cp,ep)
%Give a center point and edge point and calculate how the bar looks like

%one edge point of the bar as defined earlier
x(1) = ep(1); 
y(1) = ep(2);

%other edge point caclulated from first edge point and center point
x(2) = (cp(1)-ep(1))*2 + ep(1);
y(2) = (cp(2)-ep(2))*2 + ep(2);
