%robotics.PRMは3Ⅾはできない

%-----------生データ生成-----------%
SampleNum = 50;
theta = linspace(0, 2 * pi, SampleNum);

funcX = @(th) 1 * sin(th);
funcY = @(th) 1 * sin(2 * th);

Position = zeros(SampleNum, 3); % [cm]
for i = 1 : SampleNum
    Position(i, 1) = funcX(theta(i));
    Position(i, 2) = funcY(theta(i));
    Position(i, 3) = i;
end
figure(1)
scatter(Position(:, 1), Position(:, 2));
axis equal

%----------グリッドマップ初期化-----------%
MaxPositions = max(Position);
MinPositions = min(Position);
MaxPosition = ceil(MaxPositions);   % 正に丸める
MinPosition = floor(MinPositions);  % 負に丸める

GridMapOffset = 0;     % [mm]
MapElements_x = (MaxPosition(1) - MinPosition(1) + GridMapOffset / 10); % [cm]
MapElements_y = (MaxPosition(2) - MinPosition(2) + GridMapOffset / 10); % [cm]
GridMap = zeros(MapElements_y, MapElements_x); % 1マス10[mm]のグリッドマップ　(y, x, time)

%------生の位置データをグリッドマップに変換-----------%
for i = 1 : SampleNum
    % 各位置を四捨五入して整数にする
    x = ceil(Position(i, 1)) + abs(MinPosition(1)); % 生データをグリッド座標に写すためにオフセットする
    y = ceil(Position(i, 2)) + abs(MinPosition(2));
    
    GridMap(y, x) = 1;
end

%------バイナリデータに変換-----------%
map3D = occupancyMap3D(SampleNum);
pose = [ 0 0 0 1 0 0 0];
maxRange = SampleNum;
insertPointCloud(map3D,pose,Position,maxRange)

vehicleRadius =0.1;
safetyRadius = 0.1;
inflationRadius = vehicleRadius + safetyRadius;
inflate(map3D, inflationRadius);

figure(2)
show(map3D)


prm = robotics.PRM(map3D);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;
startLocation = [4.0 2.0];
endLocation = [24.0 20.0];
path = findpath(prm, startLocation, endLocation)

% figure
show(prm);

% BinaryMap = binaryOccupancyMap(GridMap); % Convert to binary map data
% figure(3)
% show(BinaryMap)