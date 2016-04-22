incell=dlmread('dat_wc.txt');
W=(incell(1:3,:)).'; 
C=(incell(4:6,:)).';
H_wc=W/C;
dlmwrite('H_wc.txt',H_wc,'delimiter','\t');
incell=dlmread('dat_pc.txt');
P=(incell(1:3,:)).'; 
C=(incell(4:6,:)).';
for i=1:3
   C(1,i)=C(1,i)/640-1;
   C(2,i)=1-C(2,i)/400;
end
H_pc=P/C;
dlmwrite('H_pc.txt',H_pc,'delimiter','\t');
H_pw=H_pc/H_wc;
dlmwrite('H_pw.txt',H_pw,'delimiter','\t');