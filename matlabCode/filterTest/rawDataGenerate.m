t=0:0.001:5;
dataLength=length(t);
t=t';
fid=fopen("rawData.txt",'w');
f1=1;
f2=5;
f3=35;
rawData1=sin(2*pi*f1*t)+0.1*cos(2*pi*f3*t);
rawData2=cos(2*pi*f2*t);
for i=1:1:dataLength
    fprintf(fid, '%f ',rawData1(i));
    fprintf(fid, '%f\n', rawData2(i));
end
fclose(fid);