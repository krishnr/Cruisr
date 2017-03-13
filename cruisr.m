s = tf('s');

P = s/(0.0066*s^2-0.0751);

pidTuner(P);