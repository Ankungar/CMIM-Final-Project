function [x y] = visual(cp,ep)
%Give a center point and edge point and calculate how the bar looks like

x(1) = ep(1);
y(1) = ep(2);

x(2) = (cp(1)-ep(1))*2 + ep(1);
y(2) = (cp(2)-ep(2))*2 + ep(2);
