
BD = sqrt(4^2 + 10^2 - 2*4*10*cosd(120))

gamma = acosd((BD^2 - 10^2 - 12^2) / (-2*12*10))

alpha = asind(10/BD*sind(gamma))

beta = asind(4/BD*sind(120))

zeta = alpha + beta

teta = 360 - 120 - gamma - alpha - beta

AG2 = sqrt(4^2 + 5^2 - 2*4*5*cosd(teta))

eta = asind(5/AG2*sind(teta))

omega = 120 - eta

xG2 = AG2*cosd(omega)

yG2 = AG2*sind(omega)

delta = teta - (360 - 120 -90 - 90)
