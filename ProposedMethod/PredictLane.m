function laneNum = PredictLane(frenetState, laneWidth, dt)
    
    narginchk(3,3);

    P = parameters();
    laneBounds = [inf, (P.laneCount/2:-1:-P.laneCount/2)*laneWidth, -inf];  % 車線数

    laneNum = zeros(numel(dt),1);
    
    for i = 1:numel(dt)
        if dt(i) == 0
            dLaneEgo = laneBounds-frenetState(4);
            
            laneNum(i) = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),P.laneCount); 
        else
            % Retrieve current velocity/acceleration/time
            t  = dt(i);
            a0 = frenetState(6);
            v0 = frenetState(5);

            % Solve for the constant change in acceleration and time of
            % application that arrest the ego vehicle's lateral
            % velocity and acceleration over a given number of seconds.
            if a0 == 0
                avgAcc = -v0/t;
                Ldiff = v0*t + avgAcc/2*t^2;
            else
                a = a0;
                b = (-2*v0-2*a0*t);
                c = (v0*t+a0/2*t^2);

                % Possible time switches
                r = (-b+(sqrt(b^2-4*a*c).*[-1 1]))/(2*a);

                % Select the option that occurs in the future
                rS = r(r>0 & r <= t);

                % Calculate the constant change in acceleration
                da0 = a0/(t-2*rS);

                % Predict total distance traveled over t seconds
                Ldiff = v0*t + a0/2*t^2 + da0/6*t^3 - da0/6*(t-rS)^3;
            end
            % Find distance between predicted offset and each lane
            dLaneEgo = laneBounds-(frenetState(4)+Ldiff);

            % Determine future lane
            laneNum(i) = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),P.laneCount);
        end
    end
end