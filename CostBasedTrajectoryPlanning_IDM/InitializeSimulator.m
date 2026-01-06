function [viewer, futureTrajectory, actorID, actorPoses, egoID, egoPoses, stepPerUpdate, egoglobalState, isRunning, lineHandles] = ...
    InitializeSimulator(scenarioObj, capList, refPath, laneWidth, replanRate, carLen)
    
    restart(scenarioObj)
    close all;

    numActors = numel(scenarioObj.Actors)-1;
    futureTrajectory = repelem(struct('Trajectory',[]),numActors,1);
    

    egoglobalState = frenet2global(refPath,[69 18 0 laneWidth*0 0 0]); 
    
    
    [actorID, actorPoses] = obstaclePose(capList, capList.ObstacleIDs);
    [egoID, egoPoses] = egoPose(capList, capList.EgoIDs);

    stepPerUpdate = (1/replanRate)/scenarioObj.SampleTime;
   
    
    viewer = scenario_20251223;

    viewer.SampleTime = scenarioObj.SampleTime;
  
    chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*10, 0],'ViewHeight',20,'ViewPitch',22);
   
    f = gcf;
    f.Position = [1100, 200, 690, 540]; 
    f.Color = 'white';
    f.Visible = 'on';
    advance(viewer);
    annotationSpacing = 0.05;
    % a = [];
    x = [.9 .95];
    y = [.95 .95];
    bdims = [x(1)-.175 ...
             y(1)-(annotationSpacing*3.5) ...
             (x(2)-x(1))+.2 ...
             annotationSpacing*4];
    p = uipanel(f,'Position',bdims,'BackgroundColor',[.75 .75 .75]);
    commonProps = {'VerticalAlignment','middle','HorizontalAlignment','center','Margin',0,'FontUnits','normalized','Color','k'};
    a1 = annotation(p,'textbox',[.05 .725 .9 .15],'String','Optimal  ',           'EdgeColor','g','BackgroundColor','g','LineWidth',2, commonProps{:});
    a2 = annotation(p,'textbox',[.05 .525 .9 .15],'String','Colliding  ',         'EdgeColor','r','BackgroundColor','r','LineWidth',2, commonProps{:});
    a3 = annotation(p,'textbox',[.05 .325 .9 .15],'String','NotEvaluated  ',      'EdgeColor','w','BackgroundColor','w','LineWidth',2, commonProps{:});
    a4 = annotation(p,'textbox',[.05 .125 .9 .15],'String','ConstraintViolated  ','EdgeColor','c','BackgroundColor','c','LineWidth',2, commonProps{:});
    a1.FontSize = .1;
    a2.FontSize = .1;
    a3.FontSize = .1;
    a4.FontSize = .1;
    
    lineHandles = [];
    isRunning = true;