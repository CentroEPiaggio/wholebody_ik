dat = reshape(data,3,length(data)/3)';
close all
boxplot(dat*1e3,'labels',{'SVD','Damped SVD','QR'})
ylabel('time [ms]')
grid on
print('-depsc','test.eps')