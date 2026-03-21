%% ME353 LAB - ALEX TRINH 
close all; 
% density = 
numColumns = size(ME353LabData, 2);
dP = zeros(1, numColumns - 1);

for i = 2:numColumns
    dP(i - 1) = mean(ME353LabData{:, i});
end


R = [0
0.2,; 
0.38,; 
0.53,; 
0.65,; 
0.75,; 
0.84,; 
0.91,; 
0.95,; 
0.97,; 
0.99,; 
1]


figure; 
scatter(R, dP)
xlabel('Radius');
ylabel('Average Pressure (MPa)');
title('Pressure with Radius');
grid on;

% Ti = [ 469.691872,466.3699,458.941584,447.86986,436.5865,425.877672,414.244896,398.6016,394.6719,386.517168,380.78944,378.526484]
Ti = [347.006661,345.547325, 343.063675,340.406834,337.910623,335.272777,331.978088,328.844622,326.341669,324.645705,322.703313,321.284952]

specific_density = 353./Ti; 
u = sqrt((2*dP)./specific_density);

figure; 
scatter(R,transpose(u))
xlabel("Radius (in)"); 
ylabel("Velocity (m/s)");
title("Velocity Profile with Radius")
grid on; 