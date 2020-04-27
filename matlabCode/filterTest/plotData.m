close all;
t=0:0.001:5;
t=t';
rawData=importdata('rawData.txt');
filterData=importdata('filteredData.txt');
[m,n]=size(rawData);
for i=1:1:n
    figure(i);
    plot(t,rawData(:,i));
    hold on;
    plot(t,filterData(:,i));
end