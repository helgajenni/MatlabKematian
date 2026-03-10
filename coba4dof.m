%% ============================================================
clear; clc; close all;

%% ===================== MAP & OBSTACLE SETUP =====================
mapSize   = [33 50];
xMax      = mapSize(2);
yMax      = mapSize(1);

obstacles = [10  10   0.25;
             20  20   0.25;
             30  10   0.25;
             40  20   0.25;
             17  16.5 0.25;
             41  16   0.25];

start    = [1,  8];
waypoint = [25, 20.0];
goal     = [48, 13.0];

safetyMargin  = 2.0;   % RRT* planning clearance [m]
displayMargin = 0.5;   % visual ring only [m]

%% =================== USV PHYSICAL PARAMETERS ====================
USV.L   = 1.2;
USV.B   = 0.4;
USV.m   = 11.8;
USV.U0  = 1.5;      % desired surge speed [m/s] — TETAP 1.5, tidak boleh diubah

USV.mx  = 0.59;
USV.my  = 4.72;
USV.Iz  = 1.416;
USV.Jz  = 0.142;
USV.Ix  = 0.157;
USV.Jx  = 0.016;

USV.Xu  = -2.0;
USV.Xuu = -1.5;
USV.Yv  = -35.0;    % Yv=-20: R_turn=4.1m, beta=11.8deg
USV.Yvv = -5.0;
USV.Nr  = -4.0;
USV.Nrr = -1.5;
USV.Nv  = -0.5;
USV.Yr  =  0.2;

USV.Kp_roll = -1.5;
USV.Kpp     = -0.3;
USV.GMT     =  0.15;
USV.Kv_roll =  0.1;
USV.Kr_roll = -0.1;

% T0 = |Xuu|*U0^2 + |Xu|*U0 = 1.5*2.25 + 2.0*1.5 = 6.375 N
USV.T0 = 6.4;       % nominal thrust [N] — JANGAN PERNAH dikurangi

USV.Kn_r = 2.5;
USV.Ky_r = 0.30;

USV.deltaMax  = deg2rad(35);
USV.deltaRate = deg2rad(20);

%% =================== RRT* PARAMETERS ============================
rrt.maxIter   = 12000;
rrt.stepSize  = 1.0;
rrt.goalBias  = 0.15;
rrt.rewireRad = 3.0;
rrt.goalTol   = 1.0;

%% =================== RUN RRT* ===================================
fprintf('=== RRT* Segment 1: Start -> Waypoint ===\n');
rawPath1 = rrtStar(start, waypoint, obstacles, mapSize, rrt, safetyMargin);
if isempty(rawPath1), error('RRT* Seg1 failed'); end
fprintf('  Raw path 1: %d pts\n', size(rawPath1,1));

fprintf('=== RRT* Segment 2: Waypoint -> Goal ===\n');
rawPath2 = rrtStar(waypoint, goal, obstacles, mapSize, rrt, safetyMargin);
if isempty(rawPath2), error('RRT* Seg2 failed'); end
fprintf('  Raw path 2: %d pts\n', size(rawPath2,1));

rawPathAll = [rawPath1; rawPath2(2:end,:)];

%% =================== G2CBS SMOOTHING ============================
fprintf('=== G2CBS Smoothing ===\n');

% Smooth per-segmen: kapal WAJIB melalui waypoint
smoothPath1 = g2cbsSmooth(rawPath1, 100);
smoothPath1 = repairPathObstacles(smoothPath1, obstacles, safetyMargin);

smoothPath2 = g2cbsSmooth(rawPath2, 120);
smoothPath2 = repairPathObstacles(smoothPath2, obstacles, safetyMargin);

% Gabung: titik junction = waypoint
fullPath      = [smoothPath1; smoothPath2(2:end,:)];
smoothFull    = fullPath;
wpJunctionIdx = size(smoothPath1, 1);
seg1Len       = wpJunctionIdx;
nWP           = size(fullPath, 1);

wpSpacing = sum(sqrt(sum(diff(fullPath).^2,2))) / (nWP-1);
fprintf('  G2CBS: %d pts, spacing=%.3f m, WP junction idx=%d\n', nWP, wpSpacing, seg1Len);

if seg1Len > 1 && seg1Len < nWP
    h1 = atan2(fullPath(seg1Len,2)-fullPath(seg1Len-1,2), fullPath(seg1Len,1)-fullPath(seg1Len-1,1));
    h2 = atan2(fullPath(seg1Len+1,2)-fullPath(seg1Len,2), fullPath(seg1Len+1,1)-fullPath(seg1Len,1));
    fprintf('  Heading jump at WP: %.1f deg\n', abs(rad2deg(wrapToPi(h2-h1))));
end

pathCurv = computePathCurvature(fullPath);

%% =================== CONTROLLER PARAMETERS ======================
% KEMBALI KE STANDAR MURNI: Delta = 8.0 m (Rasio 1.95 terhadap R_turn 4.1m)
ILOS.Delta    = 3.0;    % JANGAN DIUBAH (Wajib konstan)
ILOS.gamma    = 0.5;   
ILOS.beta_hat = 0;
ILOS.kappa    = 0.;

WP_RADIUS  = 1.5;   % Toleransi masuk area Waypoint
WP_PAUSE   = 3.5;   % Kapal berhenti di Waypoint
GOAL_TOL   = 1.0;   % Toleransi akhir (Wajib agak besar agar tidak muter)
WP_ACCEPT  = 0.5;   

PD.Kp = 15.0;
PD.Kd = 7.0;

U0_saved = USV.U0; % 1.5 m/s konstan
fprintf('  ILOS Delta=%.1fm (R=4.1m, ratio=%.2f) | Kp=%.1f Kd=%.1f\n', ...
    ILOS.Delta, ILOS.Delta/4.1, PD.Kp, PD.Kd);

%% =================== SIMULATION SETUP ===========================
dt   = 0.05;
Tsim = 200;
N    = round(Tsim/dt);

% Inisialisasi posisi mata kapal (wpIdx)
wpIdx = 2;
for ii = 2:min(20, nWP)
    seg   = fullPath(ii,:) - fullPath(ii-1,:);
    toSt  = start - fullPath(ii-1,:);
    along = dot(toSt, seg) / max(norm(seg), 1e-6);
    if along < norm(seg)
        wpIdx = ii;
        break;
    end
end
psi0 = atan2(fullPath(wpIdx,2) - start(2), fullPath(wpIdx,1) - start(1));
state      = zeros(N+1, 8);
state(1,:) = [start(1), start(2), psi0, 0, USV.U0, 0, 0, 0];

delta      = zeros(N+1, 1);
psi_e_prev = 0;
dpsi_e     = 0;
wpReached  = false;
wpPauseEnd = -1;
log.t       = zeros(N+1,1);
log.psi_d   = zeros(N+1,1);
log.psi_e   = zeros(N+1,1);
log.cte     = zeros(N+1,1);
log.u       = zeros(N+1,1);
log.v       = zeros(N+1,1);
log.r       = zeros(N+1,1);
log.p       = zeros(N+1,1);
log.phi     = zeros(N+1,1);
log.delta   = zeros(N+1,1);
log.pathSeg = ones(N+1,1);
log.u(1)    = USV.U0;
N_end       = N;

fprintf('=== Simulation Start ===\n');

%% =================== MAIN SIMULATION LOOP =======================
for k = 1:N
    t   = (k-1)*dt;
    xk  = state(k,1); yk  = state(k,2);
    psi = state(k,3); phi = state(k,4);
    u   = state(k,5); v   = state(k,6);
    
    %% 1. GOAL CHECK
    dist_goal = norm([xk - goal(1), yk - goal(2)]);
    if dist_goal < 0.8 
        fprintf('  >>> GOAL REACHED t=%.1fs\n', t);
        state(k,5:8) = 0; 
        break;
    end
    
    %% 2. WAYPOINT CHECK
    dist_wp = norm([xk - waypoint(1), yk - waypoint(2)]);
    if ~wpReached && dist_wp < WP_RADIUS
        wpReached  = true;
        wpPauseEnd = t + WP_PAUSE;
        ILOS.beta_hat = 0; 
        fprintf('  >>> WAYPOINT REACHED t=%.1fs \n', t);
    end
    pausing = wpReached && (t < wpPauseEnd);
    
    %% 3. ADVANCE WAYPOINT INDEX 
    dist_to_wp = norm([xk - waypoint(1), yk - waypoint(2)]);
    
    % Antisipasi Tikungan: Curi start jika dekat WP
    if ~wpReached && dist_to_wp < 3.5 
        targetIdx = min(seg1Len + 15, nWP); 
    else
        searchRange = wpIdx : min(wpIdx + 20, nWP);
        dists = sqrt((fullPath(searchRange,1) - xk).^2 + (fullPath(searchRange,2) - yk).^2);
        [~, localIdx] = min(dists);
        wpIdx = wpIdx + localIdx - 1;
        targetIdx = min(wpIdx + 5, nWP); 
    end
    
    log.pathSeg(k) = 1 + (wpIdx > seg1Len);
    
    %% 4. ILOS GUIDANCE (Tight Turns)
    p1 = fullPath(max(1, targetIdx-5), :);
    p2 = fullPath(targetIdx, :);
    alpha = atan2(p2(2)-p1(2), p2(1)-p1(1));
    ye = -sin(alpha)*(xk - p1(1)) + cos(alpha)*(yk - p1(2));
    cte = ye;

    % Delta Adaptif agar tarikan lebih rapat
    Delta_tight = max(1.8, 4.0 * exp(-0.5 * abs(ye))); 
    
    ILOS.beta_hat = ILOS.beta_hat + dt * ILOS.kappa * ...
        (ye / sqrt(Delta_tight^2 + (ye + ILOS.gamma*ILOS.beta_hat)^2));
        
    psi_d = wrapToPi(alpha - atan2(ye + ILOS.gamma*ILOS.beta_hat, Delta_tight));

    %% 5. PD CONTROLLER
    psi_e = wrapToPi(psi_d - psi);
    
    raw_dpsi = (psi_e - psi_e_prev) / dt;
    raw_dpsi = max(-2.0, min(2.0, raw_dpsi)); 
    dpsi_e   = 0.6 * raw_dpsi + 0.4 * dpsi_e;
    psi_e_prev = psi_e;
    
    delta_c = PD.Kp * psi_e + PD.Kd * dpsi_e;
    delta_c = USV.deltaMax * tanh(delta_c / USV.deltaMax);
    
    r_now = state(k,7);
    if abs(r_now) > deg2rad(15) && sign(r_now) == sign(delta_c)
        delta_c = delta_c * 0.25;
    end
    
    if pausing
        delta_c = 0; 
    end
    
    dDelta     = (delta_c - delta(k)) / dt;
    dDelta     = max(-USV.deltaRate, min(USV.deltaRate, dDelta));
    delta(k+1) = delta(k) + dDelta * dt;
    delta(k+1) = max(-USV.deltaMax, min(USV.deltaMax, delta(k+1)));
    
    %% 6. KECEPATAN (Slow-In, Fast-Out)
    if pausing
        USV.U0 = 0.0;
    else
        % Pelan saat mendekati WP atau saat melenceng jauh
        if dist_to_wp < 3.0 && ~wpReached
            USV.U0 = 0.7; 
        elseif abs(ye) > 0.6
            USV.U0 = 0.8;
        else
            USV.U0 = 1.5; 
        end
    end
    USV.T0_active = max(3.0, USV.T0 * (USV.U0 / 1.5));
    
    %% 7. DYNAMICS
    state(k+1,:) = usv4DOF(state(k,:), delta(k+1), USV, dt);
    
    %% 8. LOG
    log.t(k+1)     = t + dt;
    log.psi_d(k+1) = psi_d;
    log.psi_e(k+1) = psi_e;
    log.cte(k+1)   = cte;
    log.u(k+1)     = state(k+1,5);
    log.v(k+1)     = state(k+1,6);
    log.r(k+1)     = state(k+1,7);
    log.p(k+1)     = state(k+1,8);
    log.phi(k+1)   = state(k+1,4);
    log.delta(k+1) = delta(k+1);
    
    N_end = k+1;
end % <--- HANYA ADA SATU END DI SINI UNTUK LOOP FOR
 
Np     = N_end;
t_plot = log.t(1:Np);
segs   = log.pathSeg(1:Np);
fprintf('  Sim done. T=%.1f s  final=(%.2f, %.2f)\n', ...
    t_plot(end), state(Np,1), state(Np,2));

%% =================== SINGLE TABBED FIGURE =======================
c1=[0.00 0.45 0.74]; c2=[0.85 0.33 0.10];
c3=[0.47 0.67 0.19]; c4=[0.49 0.18 0.56];
cObs=[0.8 0.1 0.1];
theta_c = linspace(0,2*pi,60);

hFig = figure('Name','USV 4-DOF | RRT* + G2CBS + ILOS + PD',...
    'Position',[20 20 1350 830]);
tg = uitabgroup(hFig);

%% ======== TAB 1: Path Planning ========
tab1 = uitab(tg,'Title',' Path Planning ');
ax1L = axes('Parent',tab1,'Position',[0.04 0.10 0.53 0.82]);
ax1R = axes('Parent',tab1,'Position',[0.61 0.10 0.36 0.82]);

axes(ax1L); hold on; grid on; axis equal;
xlim([0 xMax]); ylim([0 yMax]);
xlabel('X [m]','FontSize',11); ylabel('Y [m]','FontSize',11);
title('RRT* Raw  vs  G2CBS Smoothed Path','FontSize',12,'FontWeight','bold');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2);
    rc=obstacles(i,3)+displayMargin;
    fill(xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,...
        'FaceAlpha',.18,'EdgeColor',cObs,'LineWidth',1.4);
    fill(xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),...
        cObs,'FaceAlpha',.6,'EdgeColor','none');
    text(xc,yc,sprintf('O%d',i),'HorizontalAlignment','center',...
        'FontSize',9,'FontWeight','bold');
end
plot(rawPath1(:,1),rawPath1(:,2),'--','Color',[c1 .5],'LineWidth',1.3,'DisplayName','Raw Seg.1');
plot(rawPath2(:,1),rawPath2(:,2),'--','Color',[c2 .5],'LineWidth',1.3,'DisplayName','Raw Seg.2');
plot(rawPath1(:,1),rawPath1(:,2),'x','Color',c1,'MarkerSize',4,'HandleVisibility','off');
plot(rawPath2(:,1),rawPath2(:,2),'x','Color',c2,'MarkerSize',4,'HandleVisibility','off');
plot(smoothPath1(:,1),smoothPath1(:,2),'-','Color',c1,'LineWidth',2.8,'DisplayName','G2CBS Seg.1');
plot(smoothPath2(:,1),smoothPath2(:,2),'-','Color',c2,'LineWidth',2.8,'DisplayName','G2CBS Seg.2');
plot(start(1),start(2),'^','MarkerSize',13,'MarkerFaceColor','g','Color','g','DisplayName','Start');
plot(waypoint(1),waypoint(2),'s','MarkerSize',11,'MarkerFaceColor','b','Color','b','DisplayName','Waypoint');
plot(goal(1),goal(2),'p','MarkerSize',16,'MarkerFaceColor','r','Color','r','DisplayName','Goal');
text(start(1)+.5,start(2)+.7,'Start','FontSize',10,'FontWeight','bold','Color','g');
text(waypoint(1)+.5,waypoint(2)+.7,'WP','FontSize',10,'FontWeight','bold','Color','b');
text(goal(1)-3,goal(2)+.7,'Goal','FontSize',10,'FontWeight','bold','Color','r');
legend('Location','northwest','FontSize',9); set(ax1L,'FontSize',10);

axes(ax1R); hold on; grid on;
arcS=computeArcLength(fullPath); arcSnorm=arcS/arcS(end);
arcR=computeArcLength(rawPathAll); arcRnorm=arcR/arcR(end);
kapR=computePathCurvature(rawPathAll);
yyaxis left
plot(arcRnorm,kapR,'-','Color',[c2 .7],'LineWidth',1.5,'DisplayName','Raw \kappa');
plot(arcSnorm,pathCurv,'-','Color',c1,'LineWidth',2.2,'DisplayName','G2CBS \kappa');
ylabel('\kappa [1/m]','FontSize',10);
yyaxis right
psiS=computePathHeading(fullPath);
plot(linspace(0,1,length(psiS)),rad2deg(psiS),'--','Color',c3,'LineWidth',1.8,'DisplayName','\psi_d');
ylabel('Path heading [deg]','FontSize',10);
xlabel('Normalised arc length','FontSize',10);
title('Curvature & Heading Profile','FontSize',11,'FontWeight','bold');
legend('Location','best','FontSize',9);
xline(arcSnorm(seg1Len),'k:','WP','LabelVerticalAlignment','bottom','FontSize',9);
set(ax1R,'FontSize',10);

%% ======== TAB 2: Trajectory ========
tab2 = uitab(tg,'Title',' Trajectory ');
ax2  = axes('Parent',tab2,'Position',[0.05 0.08 0.90 0.86]);
hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
xlim(ax2,[0 xMax]); ylim(ax2,[0 yMax]);
xlabel(ax2,'X [m]','FontSize',12); ylabel(ax2,'Y [m]','FontSize',12);
title(ax2,'USV Actual Trajectory vs G2CBS Reference','FontSize',13,'FontWeight','bold');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2);
    rc=obstacles(i,3)+displayMargin;
    fill(ax2,xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,...
        'FaceAlpha',.15,'EdgeColor',cObs,'LineWidth',1.2);
    fill(ax2,xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),...
        cObs,'FaceAlpha',.6,'EdgeColor','none');
    text(ax2,xc,yc,sprintf('O%d',i),'HorizontalAlignment','center',...
        'FontSize',9,'FontWeight','bold');
end
plot(ax2,rawPathAll(:,1),rawPathAll(:,2),':','Color',[.6 .6 .6],'LineWidth',1.2,'DisplayName','RRT* Raw');
plot(ax2,fullPath(:,1),fullPath(:,2),'--','Color',[.2 .7 .2],'LineWidth',2.0,'DisplayName','G2CBS Ref');
idx1=find(segs==1); idx2=find(segs==2);
if ~isempty(idx1)
    plot(ax2,state(idx1,1),state(idx1,2),'-','Color',c1,'LineWidth',2.8,'DisplayName','Traj Seg.1');
end
if ~isempty(idx2)
    plot(ax2,state(idx2,1),state(idx2,2),'-','Color',c2,'LineWidth',2.8,'DisplayName','Traj Seg.2');
end
aStep=max(1,round(Np/25));
for k=1:aStep:Np
    quiver(ax2,state(k,1),state(k,2),.9*cos(state(k,3)),.9*sin(state(k,3)),0,...
        'Color',[.3 .3 .3],'MaxHeadSize',1.8,'AutoScale','off');
end
plot(ax2,start(1),start(2),'^','MarkerSize',13,'MarkerFaceColor','g','Color','g','DisplayName','Start');
plot(ax2,waypoint(1),waypoint(2),'s','MarkerSize',11,'MarkerFaceColor','b','Color','b','DisplayName','WP');
plot(ax2,goal(1),goal(2),'p','MarkerSize',16,'MarkerFaceColor','r','Color','r','DisplayName','Goal');
legend(ax2,'Location','northwest','FontSize',10); set(ax2,'FontSize',11);

%% ======== TAB 3: State Variables ========
tab3 = uitab(tg,'Title',' States ');
sTit = {'Heading \psi [deg]','Heading Error [deg]','Cross-Track Error [m]',...
        'Surge u [m/s]',     'Sway v [m/s]',      'Yaw Rate r [deg/s]',...
        'Rudder \delta [deg]','Roll \phi [deg]',   'Roll Rate p [deg/s]'};
sCol = {c1,c3,c4, c1,c2,c3, c4,c1,c2};
sY   = {rad2deg(log.psi_d(1:Np)), rad2deg(log.psi_e(1:Np)), log.cte(1:Np),...
        log.u(1:Np),               log.v(1:Np),              rad2deg(log.r(1:Np)),...
        rad2deg(log.delta(1:Np)),  rad2deg(log.phi(1:Np)),   rad2deg(log.p(1:Np))};
for i=1:9
    axS = subplot(3,3,i,'Parent',tab3);
    plot(axS,t_plot,sY{i},'-','Color',sCol{i},'LineWidth',1.8); hold(axS,'on');
    if i==1
        plot(axS,t_plot,rad2deg(state(1:Np,3)),'--','Color',c2,'LineWidth',1.5);
        legend(axS,'\psi_d','\psi_{actual}','FontSize',8,'Location','best');
    end
    if i==7
        yline(axS, rad2deg(USV.deltaMax),'r--','LineWidth',1);
        yline(axS,-rad2deg(USV.deltaMax),'r--','LineWidth',1);
    end
    if i==4, yline(axS,U0_saved,'r--','LineWidth',1.2); end
    yline(axS,0,'k:','LineWidth',0.8);
    xlabel(axS,'t [s]','FontSize',9);
    title(axS,sTit{i},'FontSize',10,'FontWeight','bold');
    grid(axS,'on'); set(axS,'FontSize',9);
end
sgtitle('USV 4-DOF State Variables','FontSize',13,'FontWeight','bold');

%% ======== TAB 4: Performance ========
tab4 = uitab(tg,'Title',' Performance ');

ax41=subplot(2,2,1,'Parent',tab4);
spd=sqrt(log.u(1:Np).^2+log.v(1:Np).^2);
plot(ax41,t_plot,spd,'-','Color',c1,'LineWidth',2); hold(ax41,'on');
yline(ax41,U0_saved,'r--','LineWidth',1.5);
title(ax41,'Total Speed','FontWeight','bold');
xlabel(ax41,'t [s]'); ylabel(ax41,'[m/s]');
legend(ax41,'|V|','U_{des}=1.5m/s','FontSize',9,'Location','best');
grid(ax41,'on'); ylim(ax41,[0 2.5]);

ax42=subplot(2,2,2,'Parent',tab4);
dGoal=sqrt((state(1:Np,1)-goal(1)).^2+(state(1:Np,2)-goal(2)).^2);
plot(ax42,t_plot,dGoal,'-','Color',c2,'LineWidth',2);
yline(ax42,GOAL_TOL,'k--','Tol','FontSize',9,'LineWidth',1.2);
title(ax42,'Distance to Goal','FontWeight','bold');
xlabel(ax42,'t [s]'); ylabel(ax42,'[m]'); grid(ax42,'on');

ax43=subplot(2,2,3,'Parent',tab4);
cteA=abs(log.cte(1:Np));
cteR=zeros(Np,1);
for ii=1:Np, cteR(ii)=sqrt(mean(cteA(1:ii).^2)); end
plot(ax43,t_plot,cteA,'-','Color',[c3 .4],'LineWidth',1); hold(ax43,'on');
plot(ax43,t_plot,cteR,'-','Color',c3,'LineWidth',2.5);
title(ax43,'Cross-Track Error & RMS','FontWeight','bold');
xlabel(ax43,'t [s]'); ylabel(ax43,'[m]');
legend(ax43,'|CTE|','RMS CTE','FontSize',9,'Location','best'); grid(ax43,'on');

ax44=subplot(2,2,4,'Parent',tab4);
yyaxis(ax44,'left');
if ~isempty(idx1)
    area(ax44,t_plot(idx1),ones(length(idx1),1),'FaceColor',c1,'FaceAlpha',.35,'EdgeColor','none');
end
hold(ax44,'on');
if ~isempty(idx2)
    area(ax44,t_plot(idx2),ones(length(idx2),1)*2,'FaceColor',c2,'FaceAlpha',.35,'EdgeColor','none');
end
yticks(ax44,[1 2]); yticklabels(ax44,{'Seg1','Seg2'}); ylim(ax44,[0 3]);
ylabel(ax44,'Segment');
yyaxis(ax44,'right');
plot(ax44,t_plot,rad2deg(log.delta(1:Np)),'-','Color',c4,'LineWidth',1.5);
ylabel(ax44,'\delta [deg]');
title(ax44,'Segment & Rudder','FontWeight','bold');
xlabel(ax44,'t [s]'); grid(ax44,'on'); set(ax44,'FontSize',9);

%% ======== TAB 5: G2CBS Analysis ========
tab5 = uitab(tg,'Title',' G2CBS Analysis ');

ax51=subplot(2,3,1,'Parent',tab5);
kapR2=computePathCurvature(rawPathAll);
plot(ax51,linspace(0,1,length(kapR2)),kapR2,'--','Color',c2,'LineWidth',1.5,'DisplayName','RRT* Raw');
hold(ax51,'on');
plot(ax51,linspace(0,1,length(pathCurv)),pathCurv,'-','Color',c1,'LineWidth',2.2,'DisplayName','G2CBS');
xlabel(ax51,'Norm. arc length'); ylabel(ax51,'\kappa [1/m]');
title(ax51,'Curvature \kappa','FontWeight','bold'); grid(ax51,'on'); legend(ax51,'FontSize',9);

ax52=subplot(2,3,2,'Parent',tab5);
psiR2=computePathHeading(rawPathAll); psiS2=computePathHeading(fullPath);
plot(ax52,linspace(0,1,length(psiR2)),rad2deg(psiR2),'--','Color',c2,'LineWidth',1.5,'DisplayName','RRT* Raw');
hold(ax52,'on');
plot(ax52,linspace(0,1,length(psiS2)),rad2deg(psiS2),'-','Color',c1,'LineWidth',2.2,'DisplayName','G2CBS');
xline(ax52,seg1Len/nWP,'k:','WP','FontSize',9,'LabelVerticalAlignment','bottom');
xlabel(ax52,'Norm. arc length'); ylabel(ax52,'\psi_d [deg]');
title(ax52,'Path Heading','FontWeight','bold'); grid(ax52,'on'); legend(ax52,'FontSize',9);

ax53=subplot(2,3,3,'Parent',tab5);
dkR=abs(diff(kapR2)); dkS=abs(diff(pathCurv));
plot(ax53,linspace(0,1,length(dkR)),dkR,'--','Color',c2,'LineWidth',1.5,'DisplayName','Raw');
hold(ax53,'on');
plot(ax53,linspace(0,1,length(dkS)),dkS,'-','Color',c1,'LineWidth',2.2,'DisplayName','G2CBS');
xlabel(ax53,'Norm. arc length'); ylabel(ax53,'|d\kappa/ds|');
title(ax53,'Curvature Rate of Change','FontWeight','bold'); grid(ax53,'on'); legend(ax53,'FontSize',9);

ax54=subplot(2,3,4,'Parent',tab5); hold(ax54,'on'); grid(ax54,'on'); axis(ax54,'equal');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2);
    rc=obstacles(i,3)+displayMargin;
    fill(ax54,xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,...
        'FaceAlpha',.15,'EdgeColor',cObs,'LineWidth',1,'HandleVisibility','off');
end
plot(ax54,rawPath1(:,1),rawPath1(:,2),'--x','Color',c2,'LineWidth',1.3,'MarkerSize',5,'DisplayName','Raw Seg.1');
plot(ax54,smoothPath1(:,1),smoothPath1(:,2),'-','Color',c1,'LineWidth',2.5,'DisplayName','G2CBS Seg.1');
xlabel(ax54,'X [m]'); ylabel(ax54,'Y [m]');
title(ax54,'Segment 1 Detail','FontWeight','bold'); legend(ax54,'FontSize',9);

ax55=subplot(2,3,5,'Parent',tab5); hold(ax55,'on'); grid(ax55,'on'); axis(ax55,'equal');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2);
    rc=obstacles(i,3)+displayMargin;
    fill(ax55,xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,...
        'FaceAlpha',.15,'EdgeColor',cObs,'LineWidth',1,'HandleVisibility','off');
end
plot(ax55,rawPath2(:,1),rawPath2(:,2),'--x','Color',c2,'LineWidth',1.3,'MarkerSize',5,'DisplayName','Raw Seg.2');
plot(ax55,smoothPath2(:,1),smoothPath2(:,2),'-','Color',c1,'LineWidth',2.5,'DisplayName','G2CBS Seg.2');
xlabel(ax55,'X [m]'); ylabel(ax55,'Y [m]');
title(ax55,'Segment 2 Detail','FontWeight','bold'); legend(ax55,'FontSize',9);

ax56=subplot(2,3,6,'Parent',tab5); axis(ax56,'off');
lenRaw=sum(sqrt(sum(diff(rawPathAll).^2,2)));
lenSmo=sum(sqrt(sum(diff(fullPath).^2,2)));
stats={
    'Metric',              'RRT* Raw',                      'G2CBS';
    'Max |\kappa| [1/m]',  sprintf('%.4f',max(abs(kapR2))), sprintf('%.4f',max(abs(pathCurv)));
    'RMS \kappa [1/m]',    sprintf('%.5f',rms(kapR2)),      sprintf('%.5f',rms(pathCurv));
    'Max d\kappa/ds',      sprintf('%.4f',max(dkR)),         sprintf('%.4f',max(dkS));
    'Path length [m]',     sprintf('%.2f',lenRaw),           sprintf('%.2f',lenSmo);
    'Waypoints',           sprintf('%d',size(rawPathAll,1)), sprintf('%d',nWP);
    'ILOS \Delta [m]',     '-',                              sprintf('%.2f',ILOS.Delta);
    'WP Accept R [m]',     '-',                              sprintf('%.2f',WP_ACCEPT);
    'Cruise Speed [m/s]',  '-',                              sprintf('%.2f',U0_saved);
};
uitable('Parent',tab5,'Data',stats(2:end,:),'ColumnName',stats(1,:),...
    'RowName',{},'Units','normalized','Position',[0.67 0.03 0.31 0.46],...
    'FontSize',9,'ColumnWidth',{120,80,80});
text(ax56,.5,.88,'G2CBS Stats','HorizontalAlignment','center',...
    'FontSize',11,'FontWeight','bold','Units','normalized');

%% ======== TAB 6: Animation ========
tab6 = uitab(tg,'Title',' Animation ');
fprintf('=== Running Animation ===\n');
runAnimation(tab6,state,fullPath,rawPathAll,smoothPath1,smoothPath2,...
    obstacles,USV,log,N_end,dt,xMax,yMax,displayMargin,...
    goal,start,waypoint,ILOS,c1,c2,theta_c,U0_saved);

tg.SelectedTab = tab6;
fprintf('=== All done ===\n');

%% ================================================================
%  LOCAL FUNCTIONS
%% ================================================================

function path = repairPathObstacles(path, obs, margin)
    nObs = size(obs,1);
    for iter = 1:30
        anyFixed = false;
        for i = 2:size(path,1)-1
            for j = 1:nObs
                ox = obs(j,1); oy = obs(j,2);
                excl = obs(j,3) + margin;
                dx = path(i,1) - ox;
                dy = path(i,2) - oy;
                d  = sqrt(dx^2 + dy^2);
                if d < excl
                    if d < 1e-6, dx=1; dy=0; d=1; end
                    path(i,1) = ox + excl * dx/d;
                    path(i,2) = oy + excl * dy/d;
                    anyFixed  = true;
                end
            end
        end
        if ~anyFixed, break; end
    end
end

function path = rrtStar(startPt,goalPt,obs,mapSz,rrt,margin)
    xMax=mapSz(2); yMax=mapSz(1);
    nodes=zeros(rrt.maxIter+2,2); parent=zeros(rrt.maxIter+2,1);
    costArr=zeros(rrt.maxIter+2,1);
    nodes(1,:)=startPt; nNodes=1;
    foundGoal=false; goalIdx=0;
    for i=1:rrt.maxIter
        if rand<rrt.goalBias, sample=goalPt;
        else, sample=[rand*xMax,rand*yMax]; end
        d=sqrt(sum((nodes(1:nNodes,:)-repmat(sample,nNodes,1)).^2,2));
        [~,nI]=min(d);
        dir=sample-nodes(nI,:); dL=norm(dir);
        if dL>rrt.stepSize, newPt=nodes(nI,:)+rrt.stepSize*dir/dL;
        else, newPt=sample; end
        if ~isCF(nodes(nI,:),newPt,obs,margin)||~inBounds(newPt,xMax,yMax), continue; end
        d2=sqrt(sum((nodes(1:nNodes,:)-repmat(newPt,nNodes,1)).^2,2));
        nIdxs=find(d2<=rrt.rewireRad);
        bP=nI; bC=costArr(nI)+norm(newPt-nodes(nI,:));
        for ni=nIdxs'
            c=costArr(ni)+norm(newPt-nodes(ni,:));
            if c<bC&&isCF(nodes(ni,:),newPt,obs,margin), bC=c; bP=ni; end
        end
        nNodes=nNodes+1; nodes(nNodes,:)=newPt; parent(nNodes)=bP; costArr(nNodes)=bC;
        for ni=nIdxs'
            nc=costArr(nNodes)+norm(nodes(ni,:)-newPt);
            if nc<costArr(ni)&&isCF(newPt,nodes(ni,:),obs,margin)
                parent(ni)=nNodes; costArr(ni)=nc;
            end
        end
        if norm(newPt-goalPt)<rrt.goalTol, foundGoal=true; goalIdx=nNodes; end
    end
    if ~foundGoal
        d=sqrt(sum((nodes(1:nNodes,:)-repmat(goalPt,nNodes,1)).^2,2));
        [~,goalIdx]=min(d);
    end
    path=goalPt; idx=goalIdx;
    while idx~=1, path=[nodes(idx,:);path]; idx=parent(idx); end
    path=[startPt;path(2:end,:)];
end

function f=isCF(p1,p2,obs,margin)
    f=true;
    for s=linspace(0,1,20)
        px=p1(1)+s*(p2(1)-p1(1)); py=p1(2)+s*(p2(2)-p1(2));
        for i=1:size(obs,1)
            if sqrt((px-obs(i,1))^2+(py-obs(i,2))^2)<(obs(i,3)+margin)
                f=false; return;
            end
        end
    end
end

function b=inBounds(pt,xMax,yMax)
    b=pt(1)>=0&&pt(1)<=xMax&&pt(2)>=0&&pt(2)<=yMax;
end

function pathS = g2cbsSmooth(waypts, nPts)
    % 1. Cek apakah waypts kosong
    if isempty(waypts)
        pathS = []; return;
    end

    % 2. Hilangkan titik yang duplikat atau terlalu dekat
    minD = 1e-4; 
    keep = true(size(waypts, 1), 1);
    for i = 2:size(waypts, 1)
        if norm(waypts(i,:) - waypts(i-1,:)) < minD
            keep(i) = false; 
        end
    end
    waypts = waypts(keep, :); 
    n = size(waypts, 1);

    % 3. Penanganan jika titik <= 2 (Pemicu Error Sebelumnya)
    if n < 3
        if n == 1
            % Jika hanya ada 1 titik, duplikasi titik tersebut agar bisa diinterpolasi
            waypts = [waypts; waypts + 0.01]; 
            n = 2;
        end
        % Linear interpolation untuk 2 titik
        t0 = linspace(0, 1, n); 
        tO = linspace(0, 1, nPts);
        pathS = [interp1(t0, waypts(:,1), tO, 'linear')', ...
                 interp1(t0, waypts(:,2), tO, 'linear')'];
        pathS(1,:) = waypts(1,:); 
        pathS(end,:) = waypts(end,:); 
        return;
    end

    % 4. Prosedur Normal (Spline Smoothing)
    d = sqrt(sum(diff(waypts).^2, 2)); 
    d(d < 1e-10) = 1e-10;
    t = [0; cumsum(d)]; 
    t = t / t(end);
    
    % Pastikan t strictly increasing
    for i = 2:length(t)
        if t(i) <= t(i-1), t(i) = t(i-1) + 1e-9; end
    end
    t = t / t(end);

    ppX = spline(t, waypts(:,1)); 
    ppY = spline(t, waypts(:,2));
    
    tD = linspace(0, 1, nPts * 10);
    xD = ppval(ppX, tD); 
    yD = ppval(ppY, tD);
    
    ds = sqrt(diff(xD).^2 + diff(yD).^2);
    sA = [0, cumsum(ds)]; 
    sA = sA / sA(end);
    
    sU = linspace(0, 1, nPts);
    xS = interp1(sA, xD, sU, 'linear'); 
    yS = interp1(sA, yD, sU, 'linear');
    
    xS(1) = waypts(1,1); yS(1) = waypts(1,2);
    xS(end) = waypts(end,1); yS(end) = waypts(end,2);
    pathS = [xS(:), yS(:)];
end

function ok=isPathClear(pts,obs,margin)
    ok=true;
    for k=1:size(pts,1)
        for i=1:size(obs,1)
            if norm(pts(k,:)-obs(i,1:2))<(obs(i,3)+margin), ok=false; return; end
        end
    end
end

function s=computeArcLength(path)
    d=sqrt(sum(diff(path).^2,2)); s=[0;cumsum(d)];
end

function kappa=computePathCurvature(path)
    n=size(path,1);
    if n<3, kappa=zeros(n,1); return; end
    x=path(:,1); y=path(:,2);
    d=sqrt(diff(x).^2+diff(y).^2); d(d<1e-12)=1e-12;
    s=[0;cumsum(d)];
    xp=zeros(n,1); yp=zeros(n,1); xpp=zeros(n,1); ypp=zeros(n,1);
    for i=2:n-1
        h1=s(i)-s(i-1); h2=s(i+1)-s(i);
        xp(i)=(x(i+1)*h1^2+x(i)*(h2^2-h1^2)-x(i-1)*h2^2)/(h1*h2*(h1+h2));
        yp(i)=(y(i+1)*h1^2+y(i)*(h2^2-h1^2)-y(i-1)*h2^2)/(h1*h2*(h1+h2));
        xpp(i)=2*(x(i+1)*h1-x(i)*(h1+h2)+x(i-1)*h2)/(h1*h2*(h1+h2));
        ypp(i)=2*(y(i+1)*h1-y(i)*(h1+h2)+y(i-1)*h2)/(h1*h2*(h1+h2));
    end
    xp(1)=(x(2)-x(1))/(s(2)-s(1)); yp(1)=(y(2)-y(1))/(s(2)-s(1));
    xp(n)=(x(n)-x(n-1))/(s(n)-s(n-1)); yp(n)=(y(n)-y(n-1))/(s(n)-s(n-1));
    xpp(1)=xpp(2); ypp(1)=ypp(2); xpp(n)=xpp(n-1); ypp(n)=ypp(n-1);
    den=(xp.^2+yp.^2).^1.5; den(den<1e-9)=1e-9;
    kappa=(xp.*ypp-yp.*xpp)./den;
end

function psi=computePathHeading(path)
    dx=diff(path(:,1)); dy=diff(path(:,2));
    psi=[atan2(dy,dx); atan2(dy(end),dx(end))];
end

function [psi_d, ILOS, ey] = ilosGuidance(x, y, psi, v, u, wpPrev, wpNext, ILOS, dt) %#ok<INUSD>
% Standard ILOS — Fossen 2002, Lekkas 2014
% STABILITAS: Delta >= 2*R_turn = 8.2m
% JANGAN pakai Delta < 5m -> serpentine!
    dx    = wpNext(1) - wpPrev(1);
    dy    = wpNext(2) - wpPrev(2);
    alpha = atan2(dy, dx);

    ye = -sin(alpha)*(x - wpPrev(1)) + cos(alpha)*(y - wpPrev(2));
    ey = ye;

    if abs(ye) > 4.0
        ILOS.beta_hat = 0;
    end

    ILOS.beta_hat = ILOS.beta_hat + dt * ILOS.kappa * ...
        (ye / sqrt(ILOS.Delta^2 + (ye + ILOS.gamma*ILOS.beta_hat)^2));

    psi_d = wrapToPi(alpha - atan2(ye + ILOS.gamma*ILOS.beta_hat, ILOS.Delta));

    psi_d_err = wrapToPi(psi_d - alpha);
    if abs(psi_d_err) > deg2rad(85)
        psi_d = wrapToPi(alpha + sign(psi_d_err)*deg2rad(85));
    end
end

function sNew=usv4DOF(s,delta,USV,dt)
    k1=shipEOM(s,          delta,USV);
    k2=shipEOM(s+.5*dt*k1, delta,USV);
    k3=shipEOM(s+.5*dt*k2, delta,USV);
    k4=shipEOM(s+dt*k3,    delta,USV);
    sNew=s+(dt/6)*(k1+2*k2+2*k3+k4);
    sNew(3)=wrapToPi(sNew(3));
    sNew(4)=wrapToPi(sNew(4));
end

function ds = shipEOM(s, delta, USV)
% 4-DOF USV: [x, y, psi, phi, u, v, r, p]
    psi = s(3); phi = s(4);
    u   = s(5); v   = s(6); r = s(7); p = s(8);
    
    m  = USV.m;  mx = USV.mx; my = USV.my;
    Iz = USV.Iz; Jz = USV.Jz;
    Ix = USV.Ix; Jx = USV.Jx;
    
    % ==============================================================
    % BAGIAN YANG DIGANTI: Membaca nilai T0_active jika tersedia
    % ==============================================================
    if isfield(USV, 'T0_active')
        nom_T = USV.T0_active;
    else
        nom_T = USV.T0;
    end
    
    % Thrust: P-controller untuk menjaga target kecepatan (U0)
    T = nom_T + 5.0 * (USV.U0 - u); % 5.0 adalah gain tambahan untuk respon cepat
    T = max(0, T); % Mencegah thrust bernilai negatif berlebih jika tidak pakai rem
    % ==============================================================

    Ur   = max(sqrt(u^2 + v^2), 0.05);
    FY_r = USV.Ky_r * Ur^2 * sin(delta);
    FN_r = USV.Kn_r * Ur^2 * sin(delta);
    
    X = T + USV.Xuu * u * abs(u) + (m + my) * v * r;
    Y = USV.Yv*v + USV.Yr*r + USV.Yvv*v*abs(v) + FY_r - (m+mx)*u*r;
    N = USV.Nv*v + USV.Nr*r + USV.Nrr*r*abs(r) + FN_r;
    K = USV.Kv_roll*v + USV.Kr_roll*r + USV.Kp_roll*p ...
      + USV.Kpp*p*abs(p) - m*9.81*USV.GMT*sin(phi);
      
    u_dot = X / (m + mx);
    v_dot = Y / (m + my);
    r_dot = N / (Iz + Jz);
    p_dot = K / (Ix + Jx);
    
    x_dot = u * cos(psi) - v * sin(psi);
    y_dot = u * sin(psi) + v * cos(psi);
    
    ds = [x_dot, y_dot, r, p, u_dot, v_dot, r_dot, p_dot];
end

function runAnimation(tab,state,fullPath,rawAll,sP1,sP2,...
    obstacles,USV,log,N_end,dt,xMax,yMax,margin,...
    goal,startPt,wayptPt,ILOS,c1,c2,theta_c,U0_saved)

    axA=axes('Parent',tab,'Position',[0.03 0.17 0.61 0.79]);
    hold(axA,'on'); grid(axA,'on'); axis(axA,'equal');
    axA.Color=[0.07 0.07 0.13]; axA.GridColor=[.4 .4 .5]; axA.GridAlpha=.3;
    axA.XColor=[.9 .9 .9]; axA.YColor=[.9 .9 .9];
    xlim(axA,[0 xMax]); ylim(axA,[0 yMax]);
    xlabel(axA,'X [m]','Color',[.9 .9 .9],'FontSize',11);
    ylabel(axA,'Y [m]','Color',[.9 .9 .9],'FontSize',11);
    title(axA,'USV  RRT* + G2CBS + ILOS + PD',...
        'Color',[.95 .95 1],'FontSize',12,'FontWeight','bold');

    obsC={[1 .3 .3],[1 .5 .2],[1 .7 .2],[.8 .3 1],[.3 .8 1],[.3 1 .5]};
    for i=1:size(obstacles,1)
        xc=obstacles(i,1); yc=obstacles(i,2); rc=obstacles(i,3)+margin;
        oc=obsC{mod(i-1,6)+1};
        fill(axA,xc+rc*cos(theta_c),yc+rc*sin(theta_c),oc,...
            'FaceAlpha',.12,'EdgeColor',oc,'LineWidth',1,'LineStyle','--');
        fill(axA,xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),...
            oc,'FaceAlpha',.7,'EdgeColor','none');
        text(axA,xc,yc,sprintf('O%d',i),'Color','w','FontSize',9,...
            'HorizontalAlignment','center','FontWeight','bold');
    end
    plot(axA,rawAll(:,1),rawAll(:,2),':','Color',[.55 .55 .55],'LineWidth',1.0,'DisplayName','RRT* Raw');
    plot(axA,sP1(:,1),sP1(:,2),'-','Color',[c1 .65],'LineWidth',1.6,'DisplayName','G2CBS Seg.1');
    plot(axA,sP2(:,1),sP2(:,2),'-','Color',[c2 .65],'LineWidth',1.6,'DisplayName','G2CBS Seg.2');
    plot(axA,startPt(1),startPt(2),'^','MarkerSize',13,'MarkerFaceColor','g','Color','g');
    plot(axA,wayptPt(1),wayptPt(2),'s','MarkerSize',11,'MarkerFaceColor','b','Color','b');
    plot(axA,goal(1),goal(2),'p','MarkerSize',16,'MarkerFaceColor','r','Color','r');
    text(axA,startPt(1)+.3,startPt(2)+.8,'START','Color',[.3 1 .4],'FontSize',9,'FontWeight','bold');
    text(axA,wayptPt(1)+.3,wayptPt(2)+.8,'WP','Color',[.4 .7 1],'FontSize',9,'FontWeight','bold');
    text(axA,goal(1)-2.8,goal(2)+.8,'GOAL','Color',[1 .5 .5],'FontSize',9,'FontWeight','bold');

    hTrail=plot(axA,NaN,NaN,'-','Color',[.4 .85 1],'LineWidth',2.2);

    Ls=1.5; Bs=0.5;
    hullX=[.5*Ls,.28*Ls,-.5*Ls,-.5*Ls,.28*Ls];
    hullY=[0,.5*Bs,.4*Bs,-.4*Bs,-.5*Bs];
    houseX=[.18*Ls,-.02*Ls,-.02*Ls,.18*Ls];
    houseY=[.28*Bs,.28*Bs,-.28*Bs,-.28*Bs];
    hShip =fill(axA,NaN,NaN,[.2 .6 1],'FaceAlpha',.88,'EdgeColor','w','LineWidth',1.5);
    hHouse=fill(axA,NaN,NaN,[1 .8 .3],'FaceAlpha',.92,'EdgeColor','w','LineWidth',1);
    hBow  =plot(axA,NaN,NaN,'->','Color',[1 1 0],'MarkerSize',8,'LineWidth',2);
    hInfo =text(axA,.3,yMax*.97,'','Color','w','FontSize',9,...
        'VerticalAlignment','top','BackgroundColor',[0 0 0 .55]);

    bg=[.06 .06 .12];
    axC=axes('Parent',tab,'Position',[0.66 0.68 0.31 0.25]);
    axC.Color=bg; axC.XColor=[.8 .8 .8]; axC.YColor=[.8 .8 .8]; axC.GridColor=[.4 .4 .5];
    grid(axC,'on'); hold(axC,'on');
    xlabel(axC,'t [s]','FontSize',8,'Color',[.8 .8 .8]);
    ylabel(axC,'CTE [m]','FontSize',8,'Color',[.8 .8 .8]);
    title(axC,'Cross-Track Error','FontSize',9,'Color',[.9 .9 1],'FontWeight','bold');
    hCTE=plot(axC,NaN,NaN,'-','Color',[.3 .8 1],'LineWidth',1.5);
    yline(axC,0,'--','Color',[.6 .6 .6],'LineWidth',1);

    axSp=axes('Parent',tab,'Position',[0.66 0.40 0.31 0.24]);
    axSp.Color=bg; axSp.XColor=[.8 .8 .8]; axSp.YColor=[.8 .8 .8]; axSp.GridColor=[.4 .4 .5];
    grid(axSp,'on'); hold(axSp,'on');
    xlabel(axSp,'t [s]','FontSize',8,'Color',[.8 .8 .8]);
    ylabel(axSp,'[m/s]','FontSize',8,'Color',[.8 .8 .8]);
    title(axSp,'Speed  (cruise = 1.5 m/s)','FontSize',9,'Color',[.9 .9 1],'FontWeight','bold');
    hSpd=plot(axSp,NaN,NaN,'-','Color',[1 .65 .2],'LineWidth',1.5,'DisplayName','|V|');
    hU  =plot(axSp,NaN,NaN,'--','Color',[.4 1 .4],'LineWidth',1.2,'DisplayName','u');
    yline(axSp,U0_saved,'--','Color',[1 .4 .4],'LineWidth',1.2,'DisplayName','1.5 m/s');
    legend(axSp,'FontSize',7,'TextColor','w','Color',[0 0 0 0]);

    axDl=axes('Parent',tab,'Position',[0.66 0.12 0.31 0.24]);
    axDl.Color=bg; axDl.XColor=[.8 .8 .8]; axDl.YColor=[.8 .8 .8]; axDl.GridColor=[.4 .4 .5];
    grid(axDl,'on'); hold(axDl,'on');
    xlabel(axDl,'t [s]','FontSize',8,'Color',[.8 .8 .8]);
    ylabel(axDl,'\delta [deg]','FontSize',8,'Color',[.8 .8 .8]);
    title(axDl,'Rudder Angle','FontSize',9,'Color',[.9 .9 1],'FontWeight','bold');
    hDel=plot(axDl,NaN,NaN,'-','Color',[1 .5 .8],'LineWidth',1.5);
    yline(axDl, rad2deg(USV.deltaMax),'--','Color',[1 .3 .3],'LineWidth',1);
    yline(axDl,-rad2deg(USV.deltaMax),'--','Color',[1 .3 .3],'LineWidth',1);

    annotation(tab,'textbox',[0.66 0.91 0.31 0.07],...
        'String','RRT* → G2CBS → ILOS → PD',...
        'FontSize',9,'Color',[.7 1 .7],...
        'BackgroundColor',[0 0 0 .6],'EdgeColor',[.4 .6 .4],...
        'HorizontalAlignment','center');

    skip=max(1,round(0.08/dt));

    for k=1:skip:N_end
        if ~ishandle(tab), break; end
        xk=state(k,1); yk=state(k,2); psik=state(k,3); phik=state(k,4);
        uk=state(k,5); vk=state(k,6); delk=log.delta(k); tk=(k-1)*dt;

        set(hTrail,'XData',state(1:k,1),'YData',state(1:k,2));

        R=[cos(psik) -sin(psik);sin(psik) cos(psik)];
        hw=R*[hullX;hullY]; hsw=R*[houseX;houseY];
        set(hShip, 'XData',xk+hw(1,:), 'YData',yk+hw(2,:));
        set(hHouse,'XData',xk+hsw(1,:),'YData',yk+hsw(2,:));
        set(hBow,  'XData',[xk,xk+.5*Ls*cos(psik)],...
                   'YData',[yk,yk+.5*Ls*sin(psik)]);

        kW=max(1,k-round(60/dt)); tW=(kW-1:k-1)*dt;
        set(hCTE,'XData',tW,'YData',log.cte(kW:k));
        xlim(axC,[tW(1) max(tW(end)+.1,tW(1)+5)]);
        sp=sqrt(log.u(kW:k).^2+log.v(kW:k).^2);
        set(hSpd,'XData',tW,'YData',sp);
        set(hU,  'XData',tW,'YData',log.u(kW:k));
        xlim(axSp,[tW(1) max(tW(end)+.1,tW(1)+5)]);
        set(hDel,'XData',tW,'YData',rad2deg(log.delta(kW:k)));
        xlim(axDl,[tW(1) max(tW(end)+.1,tW(1)+5)]);

        spd=sqrt(uk^2+vk^2);
        sl='Seg.1'; if log.pathSeg(k)==2, sl='Seg.2 G2CBS'; end
        set(hInfo,'String',sprintf(...
            't = %.1f s\nSpeed = %.2f m/s\n\\delta = %.1f°\n\\psi = %.1f°\n\\phi = %.2f°\nCTE = %.2f m\n[%s]',...
            tk,spd,rad2deg(delk),rad2deg(psik),rad2deg(phik),log.cte(k),sl));

        drawnow limitrate; pause(0.001);
    end

    plot(axA,state(1:N_end,1),state(1:N_end,2),'-','Color',[.4 .9 1],...
        'LineWidth',2.5,'DisplayName','USV Trajectory');
    legend(axA,'Location','northwest','FontSize',9,...
        'TextColor','w','Color',[0 0 0 .45],'EdgeColor',[.5 .5 .6]);
    title(axA,'USV Simulation  COMPLETE  ✓',...
        'Color',[.4 1 .4],'FontSize',12,'FontWeight','bold');
end