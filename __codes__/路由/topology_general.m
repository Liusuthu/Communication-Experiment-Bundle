clc
clear
% ͼ���˹���
s = [1 1 1 2 2 3 3 3 4 5 6];
t = [2 3 4 3 4 4 5 6 5 6 7];
w = [2 5 1 3 2 3 1 5 1 2 1];
%  �ڵ���   1    2    3    4    5    6    7 
nodeNames = {'a', 'b', 'c', 'd', 'e', 'f', 'g'};
G = graph(s, t, w, nodeNames);
% �̶����ӻ�ʱ�ڵ�λ��
nodePositions = [0 1; 1 1; 2 2; 1 0; 2 0; 3 1; 4 1];
% ͼ���˿��ӻ�
figure;
plot(G, 'XData', nodePositions(:, 1), 'YData', nodePositions(:, 2), 'EdgeLabel', G.Edges.Weight);
title('�ڵ�����');
axis equal;
hold on;
save('topology_general.mat', 's', 't', 'w', 'nodePositions', 'nodeNames');