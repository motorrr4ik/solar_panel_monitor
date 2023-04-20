path = "out/";
algo_name = "in";
%% solar panel voltage
vp_out = squeeze(out.v_out);
tout = squeeze(out.tout);
plot(tout,vp_out,'LineWidth',2,'Color',[0,0,0]);
hold on 
grid on
ylabel('$Voltage, [V]$','Interpreter','latex')
xlabel('$Time, [s]$','Interpreter','latex')
saveas(gcf, path+'solar_panel_voltage_'+algo_name+'.png')
close all;
%% solar panel power 
tout = squeeze(out.tout);
p_out = squeeze(out.p_out);
plot(tout, p_out,'LineWidth',2,'Color',[0,0,0]);
hold on 
grid on
ylabel('$Power, [mW]$','Interpreter','latex')
xlabel('$Time, [s]$','Interpreter','latex')
saveas(gcf, path+'solar_panel_power_stat_'+algo_name+'.png')
close all;
%% solar panel power characteristic
v_out = squeeze(out.v_out);
p_out = squeeze(out.p_out);
plot(v_out, p_out,'LineWidth',2,'Color',[0,0,0]);
hold on 
grid on
ylabel('$Power, [mW]$','Interpreter','latex')
xlabel('$Voltage, [V]$','Interpreter','latex')
saveas(gcf, path+'solar_panel_power_stat_'+algo_name+'.png')
close all;
%% load power characteristic
tout = squeeze(out.tout);
p_load = squeeze(out.p_load);
plot(tout, p_load,'LineWidth',2,'Color',[0,0,0]);
hold on 
grid on
ylabel('$Power, [mW]$','Interpreter','latex')
xlabel('$Time, [s]$','Interpreter','latex')
saveas(gcf, path+'load_power_stat_'+algo_name+'.png')
% close all;