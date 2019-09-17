 A=importdata('s_logger_singlepl.txt');
 x=10*log10(A(:,1));
 y=A(:,2);
 plot(y,x,'g-*');
 grid on;
 xlabel('Distance(m)');
 ylabel('pathloss(dB)')