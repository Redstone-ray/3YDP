%%% Compute global axis limits %%%
all_vals = [x_error(:,2); x_u(:,2); x_reference(:,2)];
ymin = min(all_vals-0.1);
ymax = max(all_vals+0.1);

%%% Create plots %%%
figure;
sgtitle('Ball Response with Updating Trajectory');

subplot(3,1,1);
plot(x_error(:,1), x_error(:,2), 'r', 'LineWidth', 1.5);
title('Error (e)');
ylabel('m'); grid on;
ylim([ymin ymax]);

subplot(3,1,2);
plot(x_u(:,1), x_u(:,2), 'g', 'LineWidth', 1.5);
title('Controller Command (u)');
ylabel('θ'); grid on;
ylim([ymin ymax]);

subplot(3,1,3);
plot(x_reference(:,1), x_reference(:,2), 'b', 'LineWidth', 1.5);
title('Reference Position (x_r)');
xlabel('Time (s)');
ylabel('m'); grid on;
ylim([ymin ymax]);
