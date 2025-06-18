% Data points
element_sizes = [6e-5, 4e-5, 3e-5, 1e-5, 6e-6];
y0 = [3.43e-9, 3.86e-9, 4.18e-9, 4.32e-9, 4.32e-9];
freq = [10776, 10168, 9764, 9623, 9609];

% Create the plot
figure;
% plot(element_sizes, y0, '-o'); 
plot(element_sizes, freq, '-o');  
set(gca, 'XScale', 'log');

% Labeling the axes
xlabel('Element Size');
ylabel('Eigenfrequency');

% Adding a title
title('Convergence Plot of Eigenfrequency vs. Element Size');

% Optional: set the axis limits to better view the data points
%xlim([0.018, 0.35]);
%ylim([0.97, 1]);

% Optional: Enable grid
grid on;
