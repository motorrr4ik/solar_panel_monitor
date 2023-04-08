v_out = squeeze(out.v_out)
p_out = squeeze(out.p_out)
plot(v_out, p_out,'LineWidth',2,'Color',[0,0,0]);
hold on 
grid on
ylabel('$Power, [mW]$','Interpreter','latex')
xlabel('$Voltage, [V]$','Interpreter','latex')