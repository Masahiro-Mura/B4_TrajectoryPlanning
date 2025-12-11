function combinedValid = exampleHelperGap(isValid, forwardsafetyGap)
for i = 1:numTraj
    try
        finalX = globalTrajectory(i).Trajectory(end,1);
        if finalX < forwardsafetyGap
            gapValid(i) = false;
        end
    catch
        gapValid(i) = false; 
    end
end

% --- 両方がtrueのものだけ有効 ---
combinedValid = isValid & gapValid;
end



% function combinedValid = exampleHelperGap(isValid, globalTrajectory, forwardsafetyGap)
% numTraj = numel(globalTrajectory);
% gapValid = true(numTraj,1);
% 
% % --- 各軌道の終端位置S座標から前方車距離を評価する想定 ---
% for i = 1:numTraj
%     try
%         finalX = globalTrajectory(i).Trajectory(end,1);
%         if finalX < forwardsafetyGap
%             gapValid(i) = false;
%         end
%     catch
%         gapValid(i) = false; 
%     end
% end
% 
% % --- 両方がtrueのものだけ有効 ---
% combinedValid = isValid & gapValid;
% end
