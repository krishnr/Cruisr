s = tf('s');

% pidTuner(P);

P = s/(0.0328*s^2-0.3757);

Kp = 2.72;
Ki = 22.9;
Kd = 0.00335;

b = 0;
m = 1;
