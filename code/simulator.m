%% ece486_hockey_project_simulation_controller6.m
% MATLAB kinematic simulation matching controller6(3).py control logic.
% The physical arm/gripper are represented by timed animation states, while
% robot navigation, target geometry, stopping logic, reverse controllers,
% measured rotations, parameters, and state order match controller6.

clear; close all; clc;

%% 1. WORKSPACE AND INITIAL POSES
workspaceSize = 5.0;
dt = 0.02;
animationSpeed = 0;          % 0 = fastest, 1 = approximately real time

robot.x = 1; 
robot.y = 0.5; 
robot.theta = deg2rad(90);
robot.length = 0.5; 
robot.width = 0.4;

stickStation.x = 4; 
stickStation.y = 0.5; 
stickStation.yaw = deg2rad(90);

puck.pos = [1; 3]; 
puck.initialPos = puck.pos; 
puck.radius = 0.10; 
puck.velocity = [0;0]; 
puck.moving = false;

goal.center = [3; 4]; 
goal.yaw = deg2rad(30); 
goal.width = 1.0; 
goal.depth = 0.3;

%% 2. CONTROLLER6 PARAMETERS
ctrl.lookaheadL = 0.10;
ctrl.kpPosition = 0.55;
ctrl.maxLinearSpeed = 0.30;
ctrl.maxAngularSpeed = 0.75;
ctrl.positionTolerance = 0.05;
ctrl.preApproachTolerance = 0.05;
ctrl.headingTolerance = deg2rad(4.0);
ctrl.kpHeading = 1.5;
ctrl.maxAlignAngularSpeed = 0.60;
ctrl.maxFinalApproachSpeed = 0.12;
ctrl.minFinalApproachSpeed = 0.025;
ctrl.finalSpeedKp = 0.60;
ctrl.finalHeadingKp = 0.80;
ctrl.maxFinalHeadingCorrection = 0.20;
ctrl.stopLineMargin = 0.025;
ctrl.finalPositionTolerance = 0.05;
ctrl.maxSafeLateralError = 0.40;

pickup.baseHalfLength = 0.15;
pickup.preApproachClearance = 1.00;
pickup.finalEdgeClearance = 0.35;
pickup.lateralOffset = 0.00;
pickup.prepareRaiseTime = 1.0;
pickup.grabDiagonalTime = 1.0;
pickup.gripperCloseTime = 0.8;
pickup.grabReturnTime = 1.0;

shot.robotToPuckDistance = 0.30;
shot.preSwingDistance = 1.00;
shot.side = 1.0;
shot.finalStopEarlyOffset = 0.05;
shot.directionOffset = -0.10;
shot.preRotationAngle = pi;
shot.preRotationSpeed = 0.55;
shot.shotRotationAngle = 3*pi/2;
shot.shotRotationSpeed = 5.0;
shot.preRotationSign = 1.0;
shot.shotRotationSign = -1.0;
shot.stopSettleTime = 0.50;
shot.lowerStickTime = 1.0;
shot.stickGroundSettleTime = 0.40;
shot.maxPreRotationTime = 8.0;
shot.maxShotRotationTime = 10.0;

carriedStick.length = 0.50;
carriedStick.handleOffset = 0.10;
puckLaunchSpeed = 2.6;
puckLinearDamping = 0.35;
goalCaptureRadius = goal.width*0.48;

%% 3. PICKUP GEOMETRY (controller6 calculate_targets)
stationNormal = [cos(stickStation.yaw); sin(stickStation.yaw)];
pickupHeading = atan2(-stationNormal(2), -stationNormal(1));
rightVector = [sin(pickupHeading); -cos(pickupHeading)];
pickupPreTarget = [stickStation.x;stickStation.y] + ...
    (pickup.baseHalfLength + pickup.preApproachClearance)*stationNormal + ...
    pickup.lateralOffset*rightVector;
pickupFinalTarget = [stickStation.x;stickStation.y] + ...
    (pickup.baseHalfLength + pickup.finalEdgeClearance)*stationNormal + ...
    pickup.lateralOffset*rightVector;

%% 4. SHOT GEOMETRY (controller6 calculate_shot_geometry)
shotVector = goal.center - puck.pos;
shotUnit = shotVector/norm(shotVector);
shotNormal = [shotUnit(2); -shotUnit(1)];
assumedPuck = puck.pos + shot.directionOffset*shotUnit;
swingCenterTarget = assumedPuck + shot.side*shot.robotToPuckDistance*shotNormal;
preSwingTarget = assumedPuck + shot.side*shot.preSwingDistance*shotNormal;
contactHeading = atan2(assumedPuck(2)-swingCenterTarget(2), ...
                       assumedPuck(1)-swingCenterTarget(1));
preRotationStartHeading = contactHeading;
backwardHeading = wrapAngle(preRotationStartHeading + pi);
shotApproachHeading = atan2(swingCenterTarget(2)-preSwingTarget(2), ...
                            swingCenterTarget(1)-preSwingTarget(1));

%% 5. FIGURE
fig = figure('Name','ECE 486 Controller6-Matched Simulation', ...
    'Color','w','Position',[80 120 1350 650]);
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); axis(ax,[0 workspaceSize 0 workspaceSize]);
box(ax,'on'); grid(ax,'on'); xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
title(ax,'ECE 486 Hockey Simulation — controller6 algorithm');
plot(ax,[0 workspaceSize workspaceSize 0 0],[0 0 workspaceSize workspaceSize 0],'k-','LineWidth',2);

goalPoly = goalPolygon(goal.center,goal.yaw,goal.width,goal.depth);
patch(ax,goalPoly(1,:),goalPoly(2,:),[0.75 0.90 0.75],'EdgeColor',[0.1 0.45 0.1],'LineWidth',2);
goalLine = goalMouthLine(goal.center,goal.yaw,goal.width);
plot(ax,goalLine(1,:),goalLine(2,:),'Color',[0.1 0.45 0.1],'LineWidth',4);
text(goal.center(1),goal.center(2),'  Goal','FontWeight','bold');

stationPoly = orientedRectangle([stickStation.x;stickStation.y],0.48,0.34,stickStation.yaw);
patch(ax,stationPoly(1,:),stationPoly(2,:),[0.88 0.88 0.88],'EdgeColor',[0.25 0.25 0.25]);
drawLooseStick(ax,[stickStation.x;stickStation.y],stickStation.yaw);
text(stickStation.x,stickStation.y+0.45,'Stick station','HorizontalAlignment','center','FontWeight','bold');

puckHandle = rectangle(ax,'Position',[puck.pos(1)-puck.radius puck.pos(2)-puck.radius 2*puck.radius 2*puck.radius], ...
    'Curvature',[1 1],'FaceColor',[0.1 0.35 0.9],'EdgeColor','k');
text(puck.pos(1),puck.pos(2)+0.32,'Puck','HorizontalAlignment','center','FontWeight','bold');

plot(ax,pickupPreTarget(1),pickupPreTarget(2),'ko','MarkerFaceColor','y');
plot(ax,pickupFinalTarget(1),pickupFinalTarget(2),'ks','MarkerFaceColor','y');
plot(ax,preSwingTarget(1),preSwingTarget(2),'ko','MarkerFaceColor',[1 0.6 0]);
plot(ax,swingCenterTarget(1),swingCenterTarget(2),'ks','MarkerFaceColor',[1 0.6 0]);
plot(ax,[puck.pos(1) goal.center(1)],[puck.pos(2) goal.center(2)],'--','Color',[0.45 0.45 0.45]);

robotPoly = orientedRectangle([robot.x;robot.y],robot.length,robot.width,robot.theta);
robotPatch = patch(ax,robotPoly(1,:),robotPoly(2,:),[0.85 0.25 0.20],'EdgeColor','k','LineWidth',1.5);
headingLine = plot(ax,[robot.x robot.x+0.45*cos(robot.theta)], ...
    [robot.y robot.y+0.45*sin(robot.theta)],'k-','LineWidth',2.5);
carriedStickLine = plot(ax,nan,nan,'Color',[0.10 0.20 0.75],'LineWidth',5);
trajectoryLine = animatedline(ax,'Color',[0.70 0.05 0.05],'LineWidth',1.6);
statusText = text(ax,0.25,9.65,'','FontSize',11,'FontWeight','bold','BackgroundColor','w','Margin',5);

%% 6. CONTROLLER6-MATCHED STATE MACHINE
state = "PREPARE_ARM";
hasStick = false; stickLowered = false; shotContactMade = false; goalScored = false;
stateTimer = 0; simulationTime = 0; maxSimulationTime = 240;
trajectory = zeros(0,4);          % [time, x, y, theta]
controlHistory = zeros(0,3);       % [time, input v, input w]
previousRotationTheta = robot.theta; accumulatedRotation = 0; rotationStartTime = 0;

fprintf('Starting controller6-matched simulation...\n');
while isvalid(fig) && simulationTime < maxSimulationTime
    v = 0; omega = 0;

    switch state
        case "PREPARE_ARM"
            stateTimer = stateTimer + dt;
            if stateTimer >= pickup.prepareRaiseTime
                state = "GO_TO_PRE_APPROACH"; stateTimer = 0;
                fprintf('Arm raised and gripper open.\n');
            end

        case "GO_TO_PRE_APPROACH"
            [v,omega,reached] = driveToPoint(robot,pickupPreTarget,ctrl,ctrl.preApproachTolerance);
            if reached
                state = "ALIGN_TO_BASE";
                fprintf('Reached pickup pre-approach point.\n');
            end

        case "ALIGN_TO_BASE"
            % controller6 aligns to the current base-centre target including offset.
            targetCentre = [stickStation.x;stickStation.y] + pickup.lateralOffset*rightVector;
            headingToBaseCentre = atan2(targetCentre(2)-robot.y,targetCentre(1)-robot.x);
            [omega,aligned] = headingController(robot.theta,headingToBaseCentre,ctrl);
            if aligned
                state = "STRAIGHT_TO_BASE";
                fprintf('Aligned directly toward live stick-base centre.\n');
            end

        case "STRAIGHT_TO_BASE"
            [v,omega,reached] = liveBaseApproachController(robot,stickStation,pickup,rightVector,ctrl);
            if reached
                state = "GRAB_FORWARD_DOWN"; stateTimer = 0;
                fprintf('Reached final grab position.\n');
            end

        case "GRAB_FORWARD_DOWN"
            stateTimer = stateTimer + dt;
            if stateTimer >= pickup.grabDiagonalTime
                state = "CLOSE_GRIPPER"; stateTimer = 0;
            end

        case "CLOSE_GRIPPER"
            stateTimer = stateTimer + dt;
            if stateTimer >= pickup.gripperCloseTime
                hasStick = true; state = "GRAB_BACK_UP"; stateTimer = 0;
            end

        case "GRAB_BACK_UP"
            stateTimer = stateTimer + dt;
            if stateTimer >= pickup.grabReturnTime
                state = "RETURN_TO_PRE_APPROACH"; stateTimer = 0;
                fprintf('Stick grabbed and raised. Reversing to original pre-approach.\n');
            end

        case "RETURN_TO_PRE_APPROACH"
            [v,omega,reached] = reverseToPickupPreController(robot,pickupPreTarget,pickupHeading,ctrl);
            if reached
                state = "GO_TO_PRE_SWING";
                fprintf('Returned to original pickup pre-approach point.\n');
            end

        case "GO_TO_PRE_SWING"
            [v,omega,reached] = driveToPoint(robot,preSwingTarget,ctrl,ctrl.positionTolerance);
            if reached
                state = "ALIGN_FOR_PRE_ROTATION";
                fprintf('Reached 1 m pre-swing point.\n');
            end

        case "ALIGN_FOR_PRE_ROTATION"
            [omega,aligned] = headingController(robot.theta,preRotationStartHeading,ctrl);
            if aligned
                previousRotationTheta = robot.theta; accumulatedRotation = 0;
                rotationStartTime = simulationTime; state = "PRE_ROTATE_180";
                fprintf('Raised stick aligned toward assumed puck.\n');
            end

        case "PRE_ROTATE_180"
            [omega,completed,timedOut,previousRotationTheta,accumulatedRotation] = ...
                measuredRotation(robot.theta,previousRotationTheta,accumulatedRotation, ...
                shot.preRotationSign,shot.preRotationAngle,shot.preRotationSpeed, ...
                simulationTime-rotationStartTime,shot.maxPreRotationTime);
            if timedOut, error('Pre-rotation timed out.'); end
            if completed
                state = "SETTLE_AFTER_PRE_ROTATION"; stateTimer = 0;
                fprintf('Completed measured 180-degree pre-rotation.\n');
            end

        case "SETTLE_AFTER_PRE_ROTATION"
            stateTimer = stateTimer + dt;
            if stateTimer >= shot.stopSettleTime
                state = "BACKWARD_TO_SWING_CENTER";
            end

        case "BACKWARD_TO_SWING_CENTER"
            [v,omega,reached,unsafe] = reverseToSwingCenterController( ...
                robot,swingCenterTarget,shotApproachHeading,backwardHeading,shot,ctrl);
            if unsafe, error('Unsafe lateral error during reverse swing-center approach.'); end
            if reached
                state = "LOWER_STICK"; stateTimer = 0;
                fprintf('Reached final swing-center position while reversing.\n');
            end

        case "LOWER_STICK"
            stateTimer = stateTimer + dt;
            if stateTimer >= shot.lowerStickTime
                stickLowered = true; state = "SETTLE_STICK_ON_GROUND"; stateTimer = 0;
            end

        case "SETTLE_STICK_ON_GROUND"
            stateTimer = stateTimer + dt;
            if stateTimer >= shot.stickGroundSettleTime
                previousRotationTheta = robot.theta; accumulatedRotation = 0;
                rotationStartTime = simulationTime; state = "FAST_SHOT_ROTATE_270";
                fprintf('Starting measured fast 270-degree shot rotation.\n');
            end

        case "FAST_SHOT_ROTATE_270"
            [omega,completed,timedOut,previousRotationTheta,accumulatedRotation] = ...
                measuredRotation(robot.theta,previousRotationTheta,accumulatedRotation, ...
                shot.shotRotationSign,shot.shotRotationAngle,shot.shotRotationSpeed, ...
                simulationTime-rotationStartTime,shot.maxShotRotationTime);
            if timedOut, error('Shot rotation timed out.'); end

            stickTip = [robot.x;robot.y] + carriedStick.length*[cos(robot.theta);sin(robot.theta)];
            if stickLowered && ~shotContactMade && norm(stickTip-puck.pos) <= puck.radius+0.10
                shotContactMade = true; puck.moving = true; puck.velocity = puckLaunchSpeed*shotUnit;
                fprintf('Puck contact detected.\n');
            end
            if completed
                state = "PUCK_TRAVEL"; stateTimer = 0;
                fprintf('Completed measured 270-degree shot rotation.\n');
            end

        case "PUCK_TRAVEL"
            stateTimer = stateTimer + dt;
            if goalScored || stateTimer > 8, state = "DONE"; end

        case "DONE"
            v = 0; omega = 0;
    end

    % Unicycle plant.
    robot.x = robot.x + dt*v*cos(robot.theta);
    robot.y = robot.y + dt*v*sin(robot.theta);
    robot.theta = wrapAngle(robot.theta + dt*omega);
    robot.x = min(max(robot.x,0.2),workspaceSize-0.2);
    robot.y = min(max(robot.y,0.2),workspaceSize-0.2);

    if puck.moving
        puck.pos = puck.pos + puck.velocity*dt;
        puck.velocity = puck.velocity*exp(-puckLinearDamping*dt);
        if pointNearGoalMouth(puck.pos,goal.center,goal.yaw,goalCaptureRadius,goal.depth+0.35)
            goalScored = true; puck.moving = false; puck.velocity(:)=0;
        elseif any(puck.pos<0) || any(puck.pos>workspaceSize)
            puck.moving = false; puck.velocity(:)=0;
        end
    end

    simulationTime = simulationTime + dt;
    trajectory(end+1,:) = [simulationTime robot.x robot.y robot.theta]; %#ok<SAGROW>
    controlHistory(end+1,:) = [simulationTime v omega]; %#ok<SAGROW>
    addpoints(trajectoryLine,robot.x,robot.y);
    robotPoly = orientedRectangle([robot.x;robot.y],robot.length,robot.width,robot.theta);
    set(robotPatch,'XData',robotPoly(1,:),'YData',robotPoly(2,:));
    set(headingLine,'XData',[robot.x robot.x+0.45*cos(robot.theta)], ...
        'YData',[robot.y robot.y+0.45*sin(robot.theta)]);
    if hasStick
        handlePoint = [robot.x;robot.y] + carriedStick.handleOffset*[cos(robot.theta);sin(robot.theta)];
        stickTip = [robot.x;robot.y] + carriedStick.length*[cos(robot.theta);sin(robot.theta)];
        set(carriedStickLine,'XData',[handlePoint(1) stickTip(1)],'YData',[handlePoint(2) stickTip(2)]);
    end
    set(puckHandle,'Position',[puck.pos(1)-puck.radius puck.pos(2)-puck.radius 2*puck.radius 2*puck.radius]);
    set(statusText,'String',sprintf('State: %s   Time: %.1f s',strrep(state,'_',' '),simulationTime));
    drawnow;
    if animationSpeed>0, pause(dt/animationSpeed); end
    if state=="DONE", break; end
end

fprintf('\nSimulation finished at %.2f s. Goal scored: %s\n',simulationTime,string(goalScored));

%% 7. FINAL TRAJECTORY FIGURE
figure('Name','Controller6-Matched Trajectory','Color','w');
hold on; axis equal; grid on; box on;
axis([0 workspaceSize 0 workspaceSize]);
xlabel('x (m)');
ylabel('y (m)');
title('Robot Trajectory');

% Plot the robot trajectory with different colors for forward and reverse motion.
% Forward motion: v > 0  -> blue
% Reverse motion: v < 0  -> red
% Zero-linear-speed rotation does not create a visible path segment.

hForward = plot(nan,nan, ...
    'Color',[0 0.4470 0.7410], ...
    'LineWidth',2.0, ...
    'DisplayName','Forward trajectory');

hReverse = plot(nan,nan, ...
    'Color',[0.8500 0.3250 0.0980], ...
    'LineWidth',2.0, ...
    'DisplayName','Reverse trajectory');

for k = 1:size(trajectory,1)-1
    if controlHistory(k,2) > 0
        plot(trajectory(k:k+1,2),trajectory(k:k+1,3), ...
            'Color',[0 0.4470 0.7410], ...
            'LineWidth',2.0, ...
            'HandleVisibility','off');

    elseif controlHistory(k,2) < 0
        plot(trajectory(k:k+1,2),trajectory(k:k+1,3), ...
            'Color',[0.8500 0.3250 0.0980], ...
            'LineWidth',2.0, ...
            'HandleVisibility','off');
    end
end

% Original hockey-stick station and loose stick.
stationPolyFinal = orientedRectangle( ...
    [stickStation.x;stickStation.y],0.48,0.34,stickStation.yaw);
hStation = patch(stationPolyFinal(1,:),stationPolyFinal(2,:), ...
    [0.88 0.88 0.88], ...
    'EdgeColor',[0.25 0.25 0.25], ...
    'LineWidth',1.2, ...
    'DisplayName','Stick station');
drawLooseStick(gca,[stickStation.x;stickStation.y],stickStation.yaw);

% Puck at its original position before the shot.
rectangle( ...
    'Position',[puck.initialPos(1)-puck.radius, ...
                puck.initialPos(2)-puck.radius, ...
                2*puck.radius,2*puck.radius], ...
    'Curvature',[1 1], ...
    'FaceColor',[0.1 0.35 0.9], ...
    'EdgeColor','k', ...
    'LineWidth',1.2);
hPuckLegend = plot(nan,nan,'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor',[0.1 0.35 0.9], ...
    'MarkerEdgeColor','k', ...
    'LineStyle','none', ...
    'DisplayName','Initial puck position');

% Original goal.
goalPolyFinal = goalPolygon(goal.center,goal.yaw,goal.width,goal.depth);
hGoal = patch(goalPolyFinal(1,:),goalPolyFinal(2,:), ...
    [0.75 0.90 0.75], ...
    'EdgeColor',[0.1 0.45 0.1], ...
    'LineWidth',2, ...
    'DisplayName','Goal');
goalLineFinal = goalMouthLine(goal.center,goal.yaw,goal.width);
plot(goalLineFinal(1,:),goalLineFinal(2,:), ...
    'Color',[0.1 0.45 0.1], ...
    'LineWidth',4, ...
    'HandleVisibility','off');

% Intended shooting route from the puck's initial position to the goal.
hShotLine = plot( ...
    [puck.initialPos(1) goal.center(1)], ...
    [puck.initialPos(2) goal.center(2)], ...
    '--', ...
    'Color',[0.45 0.45 0.45], ...
    'LineWidth',1.4, ...
    'DisplayName','Puck-to-goal shooting line');

legend([hForward,hReverse,hStation,hPuckLegend,hGoal,hShotLine], ...
    'Location','best');

%% 8. ROBOT STATE HISTORY: x, y, AND theta ON ONE AXES
figure('Name','Robot State History','Color','w');
hold on; grid on; box on;

thetaUnwrappedRad = unwrap(trajectory(:,4));

hX = plot(trajectory(:,1),trajectory(:,2), ...
    'Color',[0 0.4470 0.7410], ...
    'LineWidth',1.6, ...
    'DisplayName','x (m)');

hY = plot(trajectory(:,1),trajectory(:,3), ...
    'Color',[0.8500 0.3250 0.0980], ...
    'LineWidth',1.6, ...
    'DisplayName','y (m)');

hTheta = plot(trajectory(:,1),thetaUnwrappedRad, ...
    'Color',[0.9290 0.6940 0.1250], ...
    'LineWidth',1.6, ...
    'DisplayName','θ (rad)');

xlabel('Time (s)');
ylabel('State value');
title('Robot State');
legend([hX,hY,hTheta],'Location','best');


%% 9. CONTROLLER INPUT HISTORY: v AND omega IN ONE FIGURE
figure('Name','Controller Input History','Color','w');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% Linear velocity input
nexttile;
plot(controlHistory(:,1),controlHistory(:,2), ...
    'Color',[0.4940 0.1840 0.5560], ...
    'LineWidth',1.6);
grid on; box on;
ylabel('v (m/s)');
title('Controller Inputs');

% Angular velocity input
nexttile;
plot(controlHistory(:,1),controlHistory(:,3), ...
    'Color',[0.4660 0.6740 0.1880], ...
    'LineWidth',1.6);
grid on; box on;
xlabel('Time (s)');
ylabel('ω (rad/s)');


%% LOCAL FUNCTIONS
function [v,w,reached] = driveToPoint(robot,target,ctrl,tolerance)
px = robot.x + ctrl.lookaheadL*cos(robot.theta);
py = robot.y + ctrl.lookaheadL*sin(robot.theta);
e = target-[px;py]; distance = norm(e);
if distance<tolerance, v=0; w=0; reached=true; return; end
pDot = ctrl.kpPosition*e;
v = cos(robot.theta)*pDot(1)+sin(robot.theta)*pDot(2);
w = (-sin(robot.theta)*pDot(1)+cos(robot.theta)*pDot(2))/ctrl.lookaheadL;
v=clamp(v,-ctrl.maxLinearSpeed,ctrl.maxLinearSpeed);
w=clamp(w,-ctrl.maxAngularSpeed,ctrl.maxAngularSpeed); reached=false;
end

function [w,aligned] = headingController(theta,desired,ctrl)
e=wrapAngle(desired-theta);
if abs(e)<ctrl.headingTolerance, w=0; aligned=true; return; end
w=clamp(ctrl.kpHeading*e,-ctrl.maxAlignAngularSpeed,ctrl.maxAlignAngularSpeed); aligned=false;
end

function [v,w,reached] = liveBaseApproachController(robot,station,pickup,rightVector,ctrl)
% Exact controller6 live-base-centre logic; station is static in this simulator.
target = [station.x;station.y] + pickup.lateralOffset*rightVector;
distanceToBase = norm(target-[robot.x;robot.y]);
stopDistance = pickup.baseHalfLength + pickup.finalEdgeClearance;
remainingDistance = distanceToBase-stopDistance;
if remainingDistance<=ctrl.stopLineMargin, v=0; w=0; reached=true; return; end
px=robot.x+ctrl.lookaheadL*cos(robot.theta); py=robot.y+ctrl.lookaheadL*sin(robot.theta);
e=target-[px;py]; pDot=ctrl.kpPosition*e;
w=(-sin(robot.theta)*pDot(1)+cos(robot.theta)*pDot(2))/ctrl.lookaheadL;
w=clamp(w,-ctrl.maxFinalHeadingCorrection,ctrl.maxFinalHeadingCorrection);
v=clamp(ctrl.finalSpeedKp*remainingDistance,ctrl.minFinalApproachSpeed,ctrl.maxFinalApproachSpeed);
reached=false;
end

function [v,w,reached] = reverseToPickupPreController(robot,target,approachHeading,ctrl)
e=target-[robot.x;robot.y]; distance=norm(e);
if distance<=ctrl.preApproachTolerance, v=0; w=0; reached=true; return; end
headingError=wrapAngle(approachHeading-robot.theta);
w=clamp(ctrl.finalHeadingKp*headingError,-ctrl.maxFinalHeadingCorrection,ctrl.maxFinalHeadingCorrection);
reverse=[-cos(approachHeading);-sin(approachHeading)]; remaining=dot(e,reverse);
if remaining<=0, v=0; w=0; reached=true; return; end
speed=clamp(ctrl.finalSpeedKp*remaining,ctrl.minFinalApproachSpeed,ctrl.maxFinalApproachSpeed);
v=-speed; reached=false;
end

function [v,w,reached,unsafe] = reverseToSwingCenterController(robot,target,approachHeading,backwardHeading,shot,ctrl)
forward=[cos(approachHeading);sin(approachHeading)];
stopTarget=target-shot.finalStopEarlyOffset*forward;
e=stopTarget-[robot.x;robot.y]; remainingForward=dot(e,forward);
lateralError=-e(1)*forward(2)+e(2)*forward(1); centerDistance=norm(e);
unsafe=abs(lateralError)>ctrl.maxSafeLateralError;
if remainingForward<=ctrl.stopLineMargin || centerDistance<=ctrl.finalPositionTolerance
    v=0; w=0; reached=true; return;
end
if unsafe, v=0; w=0; reached=false; return; end
rear=[robot.x;robot.y]-ctrl.lookaheadL*[cos(robot.theta);sin(robot.theta)];
rearTarget=stopTarget-ctrl.lookaheadL*[cos(backwardHeading);sin(backwardHeading)];
ev=rearTarget-rear; pDot=ctrl.kpPosition*ev;
v=cos(robot.theta)*pDot(1)+sin(robot.theta)*pDot(2);
w=(sin(robot.theta)*pDot(1)-cos(robot.theta)*pDot(2))/ctrl.lookaheadL;
v=clamp(v,-ctrl.maxFinalApproachSpeed,-ctrl.minFinalApproachSpeed);
w=clamp(w,-ctrl.maxFinalHeadingCorrection,ctrl.maxFinalHeadingCorrection); reached=false;
end

function [w,done,timedOut,previousTheta,accumulated] = measuredRotation(theta,previousTheta,accumulated,signValue,targetAngle,speed,elapsed,timeout)
delta=wrapAngle(theta-previousTheta); previousTheta=theta;
increment=signValue*delta; if increment>0, accumulated=accumulated+increment; end
if accumulated>=targetAngle, w=0; done=true; timedOut=false; return; end
if elapsed>=timeout, w=0; done=false; timedOut=true; return; end
w=signValue*speed; done=false; timedOut=false;
end

function poly=orientedRectangle(center,L,W,yaw)
local=0.5*[L L -L -L; W -W -W W]; R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]; poly=R*local+center;
end
function drawLooseStick(ax,center,yaw)
R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]; p=R*[-0.18 0 0.18 0.30;0 0 0 0.12]+center;
plot(ax,p(1,:),p(2,:),'Color',[0.10 0.20 0.75],'LineWidth',5);
end
function poly=goalPolygon(center,yaw,W,D)
f=[cos(yaw);sin(yaw)]; s=[-sin(yaw);cos(yaw)]; a=center+0.5*W*s; b=center-0.5*W*s; poly=[a b b+D*f a+D*f];
end
function lineData=goalMouthLine(center,yaw,W)
s=[-sin(yaw);cos(yaw)]; lineData=[center+0.5*W*s center-0.5*W*s];
end
function inside=pointNearGoalMouth(point,center,yaw,halfWidth,depth)
f=[cos(yaw);sin(yaw)]; s=[-sin(yaw);cos(yaw)]; r=point-center;
inside=dot(r,f)>=-0.20 && dot(r,f)<=depth && abs(dot(r,s))<=halfWidth;
end
function value=clamp(value,minimum,maximum), value=max(min(value,maximum),minimum); end
function angle=wrapAngle(angle), angle=atan2(sin(angle),cos(angle)); end
