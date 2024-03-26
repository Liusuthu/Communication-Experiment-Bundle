clc
clear
% 图拓扑加载
load('topology_general.mat');
% 获取节点数量
N = max([s; t], [], 'all');
% 获取连边数量
E = length(s);
fprintf("拓扑节点数: %d，拓扑边数: %d\n", N, E);
% 邻接图，值为代价，inf代表不连通
G = inf(N, N);
for i = 1:E
    G(s(i), t(i)) = w(i);
    G(t(i), s(i)) = w(i);
end
% 初始化距离矢量矩阵和下一跳矩阵（行表示源节点，列表示目标节点）
% distMatrix(n, :)代表节点n所维护的距离矢量
distMatrix = inf(N, N);
% nextHopMatrix(n, :)代表节点n所维护的下一跳路由器表
nextHopMatrix = zeros(N, N);
for n = 1:N
    distMatrix(n, n) = 0;
end
for i = 1:E
    distMatrix(s(i), t(i)) = w(i);
    distMatrix(t(i), s(i)) = w(i);
    nextHopMatrix(s(i), t(i)) = t(i);
    nextHopMatrix(t(i), s(i)) = s(i);
end

% 迭代更新距离矩阵和下一跳矩阵
[distMatrix, nextHopMatrix] = updateRoute(G, distMatrix, nextHopMatrix, 100);

% 距离向量法（Bellman-Ford 算法）
% 根据拓扑G，更新路由表（距离矩阵和下一跳矩阵）直到收敛
% 返回收敛的路由表（距离矩阵和下一跳矩阵）
function [distMatrix, nextHopMatrix] = updateRoute(G, distMatrix, nextHopMatrix, loops)
    % 获取节点数量
    numNodes = size(G, 1);
    loopCount = 0;
    % 输出路由表
    fprintf('--------------- LOOP START --------------\n')
    disp('距离矢量矩阵:');
    disp(distMatrix);
    disp('下一跳矩阵:')
    disp(nextHopMatrix);

    transferCount = 0;  % 总传输次数
    transferSize = 0;  % 总传输包大小（每条路由记录认为是大小1）

    % 迭代更新距离矩阵和下一跳矩阵
    for numLoop = 1:loops % 不断迭代更新直到收敛
        updated = false; % 是否收敛
        % 假设每个路由器定时向相邻路由器广播路由信息，广播的顺序与路由器节点编号顺序一致
        for n = 1:numNodes % 遍历所有节点（进行广播）
            % 从n发出的距离矢量
            distVector = distMatrix(n, :);
            % 距离矢量表大小（inf的视为不存在）
            dvSize = sum(~isinf(distVector), "all");
            for m = 1:numNodes % 遍历所有相邻路由器（接收到路由信息，触发路由更新）
                if isinf(G(m, n)) % 不相邻
                    continue
                end

                % 统计：传输次数+=1，传输包大小+=距离矢量个数
                transferCount = transferCount + 1;
                transferSize = transferSize + dvSize;

                % 路由表更新：距离矢量从n传输到m，m接收后进行出力
                % 通过邻居的路由（n的路由信息 + m->n的代价）
                altDist = G(m, n) + distVector;
                % 路由更新策略：
                % 1. 如果现有路由表中下一跳为该邻居节点，则更新距离信息为邻居发来的新值
                nextHopIsNAndNeedUpdate = ((nextHopMatrix(m, :) == n).*(altDist ~= distMatrix(m, :)))==1;
                distMatrix(m, nextHopIsNAndNeedUpdate) = altDist(nextHopIsNAndNeedUpdate);
                % 2. 使用代价更低的路由替换原有路由表（距离矢量法）
                betterRoute = ((~nextHopIsNAndNeedUpdate).*(altDist < distMatrix(m, :)))==1;
                distMatrix(m, betterRoute) = altDist(betterRoute);
                nextHopMatrix(m, betterRoute) = n;
                if sum(nextHopIsNAndNeedUpdate, 'all') > 0 || sum(betterRoute, 'all') > 0
                    updated = true;
                end
            end
        end

        % 路由收敛
        if ~updated
            fprintf('路由收敛，总传输次数 %d 次，总传输包大小 %d 。\n', transferCount, transferSize);
            break
        end

        loopCount = loopCount + 1;
        % 输出路由表
        fprintf('---------------- LOOP %d ----------------\n', numLoop)
        disp('距离矩阵:');
        disp(distMatrix);
        disp('下一跳矩阵:')
        disp(nextHopMatrix);
    end
    fprintf('达到循环次数上限或路由收敛，路由信息计算结束。\n');
end
