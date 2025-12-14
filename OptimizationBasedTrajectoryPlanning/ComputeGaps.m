function [validIdx, ForwardGap, BehindGap, EgoFrenetFront, EgoFrenetRear, AllActorFrenetRear, minGap, gapForwardLog, gapBehindLog, timeLog, speedLog] ...
         = ComputeGaps(EgoFrenetState, carLen, rearAxleRatio, refPath, AllActorGlobalState, ...
         laneWidth, replanRate, numActors, gapForwardLog, gapBehindLog, leadIdxLog, behindIdxLog, timeLog, simStep, scenario, EgoGlobalState, speedLog)
    
    EgoFrenetFront = EgoFrenetState(1) + carLen*(1-rearAxleRatio);
    EgoFrenetRear  = EgoFrenetState(1) - carLen*rearAxleRatio;
    
    AllActorFrenetState = global2frenet(refPath,AllActorGlobalState);
  
    AllActorFrenetFront = AllActorFrenetState(:,1) + carLen*(1-rearAxleRatio);
    AllActorFrenetRear  = AllActorFrenetState(:,1) - carLen*rearAxleRatio;

    EgoLaneNum = PredictLane(EgoFrenetState, laneWidth, 1/replanRate);
    

    % % --- 同一車線，前方車との距離
    ForwardGap = nan(1, numActors);
    for a = 1:numActors
        AllActorLaneNum = PredictLane(AllActorFrenetState(a,:), laneWidth, 1/replanRate);
        if AllActorLaneNum == EgoLaneNum && AllActorFrenetRear(a,1) > EgoFrenetFront
            ForwardGap(a) = AllActorFrenetRear(a,1) - EgoFrenetFront;
        end
    end

    % 最も近い車両の選定
    validIdx = find(~isnan(ForwardGap)); % ~isnanは論理否定演算子，NaN出ない要素番号を返す
    if ~isempty(validIdx)
        [minGap, relIdx] = min(ForwardGap(validIdx));
        forwardactorIdx = validIdx(relIdx);
    else
        minGap = nan;
        forwardactorIdx = nan;
    end




    % % --- 同一車線，後方車両との距離
    BehindGap = nan(1,numActors);
    for a = 1:numActors
        AllActorLaneNum = PredictLane(AllActorFrenetState(a,:), laneWidth, 1/replanRate);
        if AllActorLaneNum == EgoLaneNum && AllActorFrenetFront(a,1) < EgoFrenetRear
            BehindGap(a) = EgoFrenetRear - AllActorFrenetFront(a,1);
        end
    end

    % 最も近い車両の選定
    validRear = find(~isnan(BehindGap));
    if ~isempty(validRear)
        [minRearGap, relRearIdx] = min(BehindGap(validRear));
        behindActorIdx = validRear(relRearIdx);
    else
        minRearGap = nan;
        behindActorIdx = nan;
    end

    


    
    gapForwardLog(end+1,1) = minGap;          
    gapBehindLog(end+1,1) = minRearGap;            
    leadIdxLog(end+1,1) = forwardactorIdx;          
    behindIdxLog(end+1,1) = behindActorIdx;        
    timeLog(end+1,1) = (simStep-1)*scenario.SampleTime;  
    % Log ego speed (5th element of egoState is speed in this representation)
    if numel(EgoGlobalState) >= 5
        speedLog(end+1,1) = EgoGlobalState(5); 

    else
        speedLog(end+1,1) = nan; 
    end
% catch ME
%     % If frenet conversion fails for any reason, skip logging this step
%     warning('Gap logging skipped at step %d: %s', simStep, ME.message); 
%     gapForwardLog(end+1,1) = nan;
%     leadIdxLog(end+1,1) = nan; 
%     timeLog(end+1,1) = (simStep-1)*scenario.SampleTime; 
%     speedLog(end+1,1) = nan; 
end