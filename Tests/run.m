close all;
clear all;
clc;

load('exp1-3.mat');

% % % % % % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % EXP 1-3 % % % % % % % % % %% % % % % % % % % % % % %
% % % % % % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % %
% % % % figure(1)
% % % % subplot(2,1,1);
% % % % plot([1:15], ply_1_25(:,1)), hold on;
% % % % plot([6:18], oct_1_25(:,1)), hold on;
% % % % plot(11, dsl_1_25(:,1), 'o'), box on, grid on;
% % % % 
% % % % plot([1:26], ply_2_25(:,1)), hold on;
% % % % plot([3:27], oct_2_25(:,1)), hold on;
% % % % plot(21, dsl_2_25(:,1), 'o'), box on, grid on;
% % % % 
% % % % plot([1:92], ply_3_25(:,1)), hold on;
% % % % plot([3:90], oct_3_25(:,1)), hold on;
% % % % plot(15, dsl_3_25(:,1), 'o'), box on, grid on;
% % % % 
% % % % 
% % % % title('CPU');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
% % % % 
% % % % subplot(2,1,2);
% % % % plot([1:15], ply_1_25(:,2)), hold on;
% % % % plot([6:18], oct_1_25(:,2)), hold on;
% % % % plot(11, dsl_1_25(:,2), 'o'), box on, grid on;
% % % % 
% % % % plot([1:26], ply_2_25(:,2)), hold on;
% % % % plot([3:27], oct_2_25(:,2)), hold on;
% % % % plot(21, dsl_2_25(:,2), 'o'), box on, grid on;
% % % % 
% % % % plot([1:92], ply_3_25(:,2)), hold on;
% % % % plot([3:90], oct_3_25(:,2)), hold on;
% % % % plot(15, dsl_3_25(:,2), 'o'), box on, grid on;
% % % % 
% % % % title('MEM');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
% % % % 
% % % % 
% % % % figure(2)
% % % % subplot(2,1,1);
% % % % plot([1:25], ply_1_50(:,1)), hold on;
% % % % plot([3:10], oct_1_50(:,1)), hold on;
% % % % plot([9:24], dsl_1_50(:,1)), box on, grid on;
% % % % 
% % % % plot([1:20], ply_2_50(:,1)), hold on;
% % % % plot([2:21], oct_2_50(:,1)), hold on;
% % % % plot(1, 0), box on, grid on;
% % % % 
% % % % plot([1:54], ply_3_50(:,1)), hold on;
% % % % plot([2:51], oct_3_50(:,1)), hold on;
% % % % plot(37, dsl_3_50(:,1), 'o'), box on, grid on;
% % % % 
% % % % title('CPU');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
% % % % 
% % % % subplot(2,1,2);
% % % % plot([1:25], ply_1_50(:,2)), hold on;
% % % % plot([3:10], oct_1_50(:,2)), hold on;
% % % % plot([9:24], dsl_1_50(:,2)), box on, grid on;
% % % % 
% % % % plot([1:20], ply_2_50(:,2)), hold on;
% % % % plot([2:21], oct_2_50(:,2)), hold on;
% % % % plot(1, 0), box on, grid on;
% % % % 
% % % % plot([1:54], ply_3_50(:,2)), hold on;
% % % % plot([2:51], oct_3_50(:,2)), hold on;
% % % % plot(37, dsl_3_50(:,2), 'o'), box on, grid on;
% % % % 
% % % % title('MEM');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
% % % % 
% % % % figure(3)
% % % % subplot(2,1,1);
% % % % plot([1:24], ply_1_100(:,1)), hold on;
% % % % plot([3:8], oct_1_100(:,1)), hold on;
% % % % plot([7:20], dsl_1_100(:,1)), box on, grid on;
% % % % 
% % % % plot([1:36], ply_2_100(:,1)), hold on;
% % % % plot([2:17], oct_2_100(:,1)), hold on;
% % % % plot([14:32], dsl_2_100(:,1)), box on, grid on;
% % % % 
% % % % plot([1:56], ply_3_100(:,1)), hold on;
% % % % plot([2:42], oct_3_100(:,1)), hold on;
% % % % plot([29:55], dsl_3_100(:,1)), box on, grid on;
% % % % 
% % % % title('CPU');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
% % % % 
% % % % subplot(2,1,2);
% % % % plot([1:24], ply_1_100(:,2)), hold on;
% % % % plot([3:8], oct_1_100(:,2)), hold on;
% % % % plot([7:20], dsl_1_100(:,2)), box on, grid on;
% % % % 
% % % % plot([1:36], ply_2_100(:,1)), hold on;
% % % % plot([2:17], oct_2_100(:,2)), hold on;
% % % % plot([14:32], dsl_2_100(:,2)), box on, grid on;
% % % % 
% % % % plot([1:56], ply_3_100(:,2)), hold on;
% % % % plot([2:42], oct_3_100(:,2)), hold on;
% % % % plot([29:55], dsl_3_100(:,2)), box on, grid on;
% % % % 
% % % % title('MEM');
% % % % xlabel('[seconds]')
% % % % ylabel('[%]')
% % % % legend('Exp1_{ply-publisher}', 'Exp1_{octomap-server}', 'Exp1_{dsl}', ...
% % % %     'Exp2_{ply-publisher}', 'Exp2_{octomap-server}', 'Exp2_{dsl}',...
% % % %      'Exp3_{ply-publisher}', 'Exp3_{octomap-server}', 'Exp3_{dsl}')
 
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % % 
% % % % % % % % % % % % % EXP 1 % % % % % % % % % %% % % % % % % % % % % % %
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % %
figure(1)
subplot(2,1,1);
plot([1:15], ply_1_25(:,1)), hold on;
plot([6:18], oct_1_25(:,1)), hold on;
plot(11, dsl_1_25(:,1), 'o'), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 0.25. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:15], ply_1_25(:,2)), hold on;
plot([6:18], oct_1_25(:,2)), hold on;
plot(11, dsl_1_25(:,2), 'o'), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 0.25. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')


figure(2)
subplot(2,1,1);
plot([1:25], ply_1_50(:,1)), hold on;
plot([3:10], oct_1_50(:,1)), hold on;
plot([9:24], dsl_1_50(:,1)), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 0.5. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:25], ply_1_50(:,2)), hold on;
plot([3:10], oct_1_50(:,2)), hold on;
plot([9:24], dsl_1_50(:,2)), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 0.5. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

figure(3)
subplot(2,1,1);
plot([1:24], ply_1_100(:,1)), hold on;
plot([3:8], oct_1_100(:,1)), hold on;
plot([7:20], dsl_1_100(:,1)), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 1.0. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:24], ply_1_100(:,2)), hold on;
plot([3:8], oct_1_100(:,2)), hold on;
plot([7:20], dsl_1_100(:,2)), box on, grid on;

title('Map: 25x25x5. Octomap resolution: 1.0. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')
 
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % % 
% % % % % % % % % % % % % EXP 2 % % % % % % % % % %% % % % % % % % % % % % %
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % %
figure(4)
subplot(2,1,1);
plot([1:26], ply_2_25(:,1)), hold on;
plot([3:27], oct_2_25(:,1)), hold on;
plot(21, dsl_2_25(:,1), 'o'), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 0.25. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:26], ply_2_25(:,2)), hold on;
plot([3:27], oct_2_25(:,2)), hold on;
plot(21, dsl_2_25(:,2), 'o'), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 0.25. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')


figure(5)
subplot(2,1,1);
plot([1:20], ply_2_50(:,1)), hold on;
plot([2:21], oct_2_50(:,1)), hold on;
plot(1, 0), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 0.5. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:20], ply_2_50(:,2)), hold on;
plot([2:21], oct_2_50(:,2)), hold on;
plot(1, 0), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 0.5. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

figure(6)
subplot(2,1,1);
plot([1:36], ply_2_100(:,1)), hold on;
plot([2:17], oct_2_100(:,1)), hold on;
plot([14:32], dsl_2_100(:,1)), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 1.0. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:36], ply_2_100(:,1)), hold on;
plot([2:17], oct_2_100(:,2)), hold on;
plot([14:32], dsl_2_100(:,2)), box on, grid on;

title('Map: 50x50x5. Octomap resolution: 1.0. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')
 
 
 
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % % 
% % % % % % % % % % % % % EXP 3 % % % % % % % % % %% % % % % % % % % % % % %
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % %
figure(7)
subplot(2,1,1);
plot([1:92], ply_3_25(:,1)), hold on;
plot([3:90], oct_3_25(:,1)), hold on;
plot(15, dsl_3_25(:,1), 'o'), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 0.25. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:92], ply_3_25(:,2)), hold on;
plot([3:90], oct_3_25(:,2)), hold on;
plot(15, dsl_3_25(:,2), 'o'), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 0.25. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

figure(8)
subplot(2,1,1);
plot([1:54], ply_3_50(:,1)), hold on;
plot([2:51], oct_3_50(:,1)), hold on;
plot(37, dsl_3_50(:,1), 'o'), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 0.5. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:54], ply_3_50(:,2)), hold on;
plot([2:51], oct_3_50(:,2)), hold on;
plot(37, dsl_3_50(:,2), 'o'), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 0.5. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

figure(9)
subplot(2,1,1);
plot([1:56], ply_3_100(:,1)), hold on;
plot([2:42], oct_3_100(:,1)), hold on;
plot([29:55], dsl_3_100(:,1)), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 1.0. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')

subplot(2,1,2);
plot([1:56], ply_3_100(:,2)), hold on;
plot([2:42], oct_3_100(:,2)), hold on;
plot([29:55], dsl_3_100(:,2)), box on, grid on;

title('Map: 100x100x5. Octomap resolution: 1.0. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('ply-publisher', 'octomap-server', 'dsl')



% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % % 
% % % % % % % % % % % % % EXP 1-3 % % % % % % % % % %% % % % % % % % % % % % %
% % % % % % % % % % % % %% % % % % % % % % % % % %% % % % % % % % % % % % %
figure(10)
subplot(2,1,1);
plot(1, dsl_1_25(:,1), 'o'), hold on, box on, grid on;
plot(1, dsl_2_25(:,1), 'o'), hold on, box on, grid on;
plot(1, dsl_3_25(:,1), 'o'), hold on, box on, grid on;

title('DSL. Octomap resolution: 0.25. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')

subplot(2,1,2);
plot(1, dsl_1_25(:,2), 'o'), hold on, box on, grid on;
plot(1, dsl_2_25(:,2), 'o'), hold on, box on, grid on;
plot(1, dsl_3_25(:,2), 'o'), hold on, box on, grid on;

title('DSL. Octomap resolution: 0.25. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')

figure(11)
subplot(2,1,1);
plot([1:16], dsl_1_50(:,1)), hold on, box on, grid on;
plot(1, 0), hold on, box on, grid on;
plot(1, dsl_3_50(:,1), 'o'), hold on, box on, grid on;

title('DSL. Octomap resolution: 0.5. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')

subplot(2,1,2);
plot([1:16], dsl_1_50(:,2)), hold on, box on, grid on;
plot(1, 0), box on, grid on;
plot(1, dsl_3_50(:,2), 'o'), hold on, box on, grid on;

title('DSL. Octomap resolution: 0.5. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')

figure(12)
subplot(2,1,1);
plot([1:14], dsl_1_100(:,1)), hold on, box on, grid on;
plot([1:19], dsl_2_100(:,1)), hold on, box on, grid on;
plot([1:27], dsl_3_100(:,1)), hold on, box on, grid on;

title('DSL. Octomap resolution: 1.0. CPU');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')

subplot(2,1,2);
plot([1:14], dsl_1_100(:,2)), hold on, box on, grid on;
plot([1:19], dsl_2_100(:,2)), hold on, box on, grid on;
plot([1:27], dsl_3_100(:,2)), hold on, box on, grid on;

title('DSL. Octomap resolution: 1.0. MEM');
xlabel('[seconds]')
ylabel('[%]')
legend('25x25x5', '50x50x5', '100x100x5')


FolderName = '/home/anton/ros_workspaces/release_ws/src/dsl_gridsearch/Tests/figures';   % Your destination folder
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
for iFig = 1:length(FigList)
  FigHandle = FigList(iFig);
  FigName   = num2str(get(FigHandle, 'Number'));
%   set(0, 'CurrentFigure', FigHandle);
%   savefig(fullfile(FolderName, [FigName '.fig']));
%   print([FolderName,num2str(iFig),'.dpng']);
%   saveas(gcf,['fig' num2str(iFig) '.png']);
  whereToStore=fullfile(FolderName,['fig' num2str(iFig) '.png']);
  saveas(gcf, whereToStore);
end

  
