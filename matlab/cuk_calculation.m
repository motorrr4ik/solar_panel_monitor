clear;clc;

Vin = 5;% input voltage unit V
Vout = 5;% output voltage unit V
fs = 30000;% switching frequency unit HZ
Iout = 0.24;% Output Current Unit A
DeltaIin = 0.2;% input current ripple unit a
DeltaIout = 0.2;% Output Current Wave Unit A
DeltaVc = 0.01;% Output voltage ripple unit V

Ts = 1 / fs;% switch cycle
duty = Vout / (Vout+Vin);
L1 = (Vin * duty * Ts) / DeltaIin;
L2 = (Vin * duty * Ts) / DeltaIout;
C1 = (Iout * duty * Ts) / DeltaVc;
C2 = (DeltaIout * Ts) / ( 8 * DeltaVc);

duty = duty * 100;% unit%
Ts = Ts * 10 ^ 6;% unit US
L1 = L1 * 10 ^ 6;% unit UH
L2 = L2 * 10 ^ 6;% unit UH
C1 = C1 * 10 ^ 6;% unit UF
C2 = C2 * 10 ^ 6;% unit UF

fprintf('duty  = %.1f%%\n',duty);
fprintf('Ts	  = %.1fus\n',Ts);
fprintf('L1	  = %.1fuH\n',L1);
fprintf('L2	  = %.1fuH\n',L2);
fprintf('C1	  = %.1fuF\n',C1);
fprintf('C2	  = %.1fuF\n',C2);