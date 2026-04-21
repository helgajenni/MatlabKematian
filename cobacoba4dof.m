%% ============================================================
% SIMULASI USV 4-DOF RRT* + G2CBS + ILOS (SCRIPT LAMA + PARAMETER BARU)
%% ============================================================
clear; clc; close all;

%% ===================== SEED SETUP (UNTUK DATA TA) ===============
seed_value = 50; 
rng(seed_value);
fprintf('=== Inisialisasi Simulasi dengan Seed: %d ===\n', seed_value);

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
safetyMargin  = 2.0;   
displayMargin = 0.5;   

%% =================== USV PHYSICAL PARAMETERS ====================
USV.L   = 1.6;
USV.B   = 0.4;
USV.m   = 11.8;
USV.U0  = 1.5;      
USV.T0  = 55.0;     % Dipertahankan 55 agar kuat menembus 1.5 m/s

% --- PARAMETER IDENTIFIKASI TERBARU (DIKUNCI DENGAN ABS) ---
USV.A2  = 1.918600;  
USV.A3  = abs(0.661100);  
USV.A4  = abs(-0.097400); 
USV.A5  = 0.000000;
USV.A37 = 0.021900;  

USV.A7  = abs(0.546000);  
USV.A8  = abs(-0.091900); 
USV.A9  = 0.000000;
USV.A39 = -0.376300; 

USV.A18 = abs(0.010000);  
USV.A19 = 0.000000;  
USV.A20 = 0.000000;
USV.A21 = 0.260530;  
USV.A22 = 8.353146;  
USV.A23 = -0.006395; 

USV.A32 = 0.000000;
USV.A33 = 0.000000;
USV.A34 = abs(-0.561900); 
USV.A35 = abs(-0.832200); 
USV.A36 = 0.000000;
USV.A38 = abs(0.074200);  

USV.Ky_r = 12.0;
USV.deltaMax  = deg2rad(35);
USV.deltaRate = deg2rad(25);

%% =================== RRT* PARAMETERS ============================
rrt.maxIter   = 800;
rrt.stepSize  = 1.0;
rrt.goalBias  = 0.15;
rrt.rewireRad = 3.0;
rrt.goalTol   = 1.0;

%% =================== RUN RRT* ===================================
fprintf('=== RRT* Segment 1: Start -> Waypoint ===\n');
tic; 
rawPath1 = rrtStar(start, waypoint, obstacles, mapSize, rrt, safetyMargin);
waktu_segmen1 = toc; 
if isempty(rawPath1), error('RRT* Seg1 failed'); end

fprintf('=== RRT* Segment 2: Waypoint -> Goal ===\n');
tic; 
rawPath2 = rrtStar(waypoint, goal, obstacles, mapSize, rrt, safetyMargin);
waktu_segmen2 = toc; 
if isempty(rawPath2), error('RRT* Seg2 failed'); end
rawPathAll = [rawPath1; rawPath2(2:end,:)];

%% =================== G2CBS SMOOTHING ===================
fprintf('=== G2CBS Smoothing (Continuous Mode) ===\n');
rawPathCombined = [rawPath1; rawPath2(2:end,:)]; 
nTotalPoints = 400; 
smoothFull = g2cbsSmooth(rawPathCombined, nTotalPoints);
fullPath = repairPathObstacles(smoothFull, obstacles, safetyMargin);
[~, wpJunctionIdx] = min(sqrt(sum((fullPath - waypoint).^2, 2)));
nWP = size(fullPath, 1);
pathCurv = computePathCurvature(fullPath);
seg1Len = wpJunctionIdx;
smoothPath1 = fullPath(1:wpJunctionIdx, :);
smoothPath2 = fullPath(wpJunctionIdx:end, :);

%% =================== CONTROLLER PARAMETERS ======================
ILOS.Delta    = 1.11;    % DIPERPENDEK: Pandangan jarak dekat agar tidak memotong tikungan
ILOS.gamma    = 0.6;   
ILOS.beta_hat = 0;
ILOS.kappa    = 0.04;    

WP_RADIUS  = 1.5;   
WP_PAUSE   = 3.5;   
GOAL_TOL   = 0.5;   

PD.Kp = 4.98;            % HYPER-AGRESSIVE: Error 5cm saja kemudi langsung dibanting untuk kembali ke garis
PD.Kd = 3.67  ;            % HYPER-BRAKE: Rem kemudi yang sangat masif agar kapal TIDAK mengular saat menyentuh garis
U0_saved = USV.U0; 
fprintf('  ILOS Delta=%.1fm | Kp=%.1f Kd=%.1f\n', ILOS.Delta, PD.Kp, PD.Kd);

%% =================== SIMULATION SETUP ===========================
dt   = 0.05;
Tsim = 200;
N    = round(Tsim/dt);
wpIdx = 2;
idxNext_init = min(wpIdx + 12, nWP); % Sesuaikan dengan idxNext di dalam loop (misal 12)
dx_init = fullPath(idxNext_init,1) - fullPath(wpIdx,1);
dy_init = fullPath(idxNext_init,2) - fullPath(wpIdx,2);
psi0 = atan2(dy_init, dx_init); % Kapal otomatis sejajar dengan garis referensi biru muda

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
psi_d_filtered = psi0;
min_dist_to_goal = inf;

fprintf('=== Simulation Start ===\n');

%% =================== MAIN SIMULATION LOOP =======================
for k = 1:N
    t   = (k-1)*dt;
    xk  = state(k,1); yk  = state(k,2);
    psi = state(k,3); phi = state(k,4);
    u   = state(k,5); v   = state(k,6);
    r   = state (k,7); p   = state (k,8);
    
    %% 1. GOAL CHECK (SMART DOCKING)
    dist_goal = norm([xk - goal(1), yk - goal(2)]); 
    
    % Mencatat jarak paling dekat yang pernah dicapai kapal
    if dist_goal < min_dist_to_goal
        min_dist_to_goal = dist_goal;
    end
    
    % LOGIKA SANDAR: Berhenti jika menyentuh radius 0.3m (30 cm)
    % ATAU jika sudah masuk radius 1.5m tapi jaraknya mulai membesar (pertanda bablas)
    if dist_goal < 0.3 || (dist_goal < 1.5 && dist_goal > min_dist_to_goal + 0.02)
        fprintf('  >>> TARGET TERCAPAI pada t = %.1fs\n', t);
        
        % Force stop semua kecepatan
        state(k+1, 5:8) = 0; 
        state(k, 5:8) = 0;
        
        % Snap (kunci) posisi persis ke titik goal secara visual agar rapi
        state(k+1, 1:2) = [goal(1), goal(2)]; 
        break; 
    end
    
    %% 2. WAYPOINT CHECK
    dist_wp = norm([xk - waypoint(1), yk - waypoint(2)]);
    if ~wpReached && dist_wp < WP_RADIUS
        wpReached  = true;
        wpPauseEnd = t + WP_PAUSE;
        ILOS.beta_hat = 0; 
    end
    pausing = false; 
    
  %% 3. ADVANCE WAYPOINT
    searchRange = wpIdx : min(wpIdx + 18, nWP);
    dists = sqrt((fullPath(searchRange,1) - xk).^2 + (fullPath(searchRange,2) - yk).^2);
    [~, localIdx] = min(dists);
    newIdx = searchRange(localIdx);
    if newIdx >= wpIdx
        wpIdx = newIdx;  
    end
    log.pathSeg(k) = 1 + (wpIdx > seg1Len);   
    
    idxNext = min(wpIdx + 13, nWP);  % UBAH MENJADI 8: Menghitung arah tikungan secara lebih presisi
    if idxNext == wpIdx && wpIdx > 1
        dx_path = fullPath(wpIdx,1) - fullPath(wpIdx-1,1);
        dy_path = fullPath(wpIdx,2) - fullPath(wpIdx-1,2);
    else
        dx_path = fullPath(idxNext,1) - fullPath(wpIdx,1);
        dy_path = fullPath(idxNext,2) - fullPath(wpIdx,2);
    end
    alpha = atan2(dy_path, dx_path);
    
    %% 4. MURNI ILOS GUIDANCE
    p1 = fullPath(wpIdx,:);
    ye = -sin(alpha)*(xk - p1(1)) + cos(alpha)*(yk - p1(2));
    cte = ye;
    
    if abs(ye) > 5.0
        ILOS.beta_hat = 0; 
    end
    
    Delta_eff = ILOS.Delta; 
    max_ye_allowed = Delta_eff * 1.0; 
    ye_clamped = max(-max_ye_allowed, min(max_ye_allowed, ye));
    
    ILOS.beta_hat = ILOS.beta_hat + dt * ILOS.kappa * (ye_clamped / Delta_eff);
    ILOS.beta_hat = max(-0.6, min(0.6, ILOS.beta_hat)); % Batasi agar tidak meluap

    psi_d_raw = wrapToPi(alpha - atan2(ye_clamped + ILOS.gamma*ILOS.beta_hat, Delta_eff));
    alpha_lpf = 0.6; 
    psi_d_filtered = psi_d_filtered + alpha_lpf * wrapToPi(psi_d_raw - psi_d_filtered);
    psi_d = wrapToPi(psi_d_filtered);
    
    %% 5. PD CONTROLLER (RUDDER)
    psi_e = wrapToPi(psi_d - psi);
    dpsi_e = -r;
    
    % --- KODE BARU: FADE-IN GAIN ---
    % Mencegah hentakan. Nilainya bergerak dari 0 ke 1 secara halus selama 5 detik pertama.
    fade_in = min(1.0, t / 5.0); 
    
    % Kalikan hasil PD dengan fade_in
    delta_c = (PD.Kp * psi_e + PD.Kd * dpsi_e) * fade_in;
    
    delta_c = USV.deltaMax * tanh(delta_c / USV.deltaMax);
    
    if pausing, delta_c = 0; end
    
    dDelta     = (delta_c - delta(k)) / dt;
    dDelta     = max(-USV.deltaRate, min(USV.deltaRate, dDelta));
    delta(k+1) = delta(k) + dDelta * dt;
    delta(k+1) = max(-USV.deltaMax, min(USV.deltaMax, delta(k+1)));
    
    %% 6. KECEPATAN ADAPTIF DENGAN LOW-PASS FILTER (SMOOTH BRAKING)
    % Cek kelengkungan di 10 titik ke depan
    futureCurvature = max(abs(pathCurv(wpIdx : min(wpIdx + 14, nWP))));
    
    % Tentukan TARGET kecepatan terlebih dahulu
    if dist_goal < 4.0
        target_U0 = max(0.1, 1.5 * (dist_goal / 4.0)); % Kecepatan merayap pelan saat nyaris sampai
    elseif abs(psi_e) > deg2rad(20) || futureCurvature > 0.12
        target_U0 = 1.1; 
    else
        target_U0 = 1.5; 
    end
    
    % --- KUNCI UTAMA: LOW-PASS FILTER ---
    % Kecepatan USV akan mendekati target secara perlahan (halus), tidak lompat instan.
    % Ini akan menghilangkan "tanjakan/spike" tajam pada grafik Rudder Angle.
    alpha_u = 0.15; 
    USV.U0 = USV.U0 + alpha_u * (target_U0 - USV.U0);
    
    USV.T0_active = max(1.5, USV.T0 * (USV.U0 / 1.5));
    if USV.U0 == 0, USV.T0_active = 0; end
    
    %% 7. DYNAMICS (Model 4 DOF Baru)
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
end 
 
Np     = N_end;
t_plot = log.t(1:Np);
segs   = log.pathSeg(1:Np);
fprintf('  Sim done. T=%.1f s  final=(%.2f, %.2f)\n', t_plot(end), state(Np,1), state(Np,2));

%% =================== 6-TAB FIGURE LENGKAP =======================
c1=[0.00 0.45 0.74]; c2=[0.85 0.33 0.10]; c3=[0.47 0.67 0.19]; c4=[0.49 0.18 0.56];
cObs=[0.8 0.1 0.1]; theta_c = linspace(0,2*pi,60);
hFig = figure('Name','USV 4-DOF | RRT* + G2CBS + ILOS + PD','Position',[20 20 1350 830]);
tg = uitabgroup(hFig);

%% ======== TAB 1: Path Planning ========
tab1 = uitab(tg,'Title',' Path Planning ');
ax1L = axes('Parent',tab1,'Position',[0.04 0.10 0.53 0.82]);
ax1R = axes('Parent',tab1,'Position',[0.61 0.10 0.36 0.82]);
axes(ax1L); hold on; grid on; axis equal; xlim([0 xMax]); ylim([0 yMax]);
xlabel('X [m]','FontSize',11); ylabel('Y [m]','FontSize',11);
title('RRT* Raw  vs  G2CBS (Bezier) Smoothed Path','FontSize',12,'FontWeight','bold');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2); rc=obstacles(i,3)+displayMargin;
    fill(xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,'FaceAlpha',.18,'EdgeColor',cObs,'LineWidth',1.4);
    fill(xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),cObs,'FaceAlpha',.6,'EdgeColor','none');
    text(xc,yc,sprintf('O%d',i),'HorizontalAlignment','center','FontSize',9,'FontWeight','bold');
end
plot(rawPath1(:,1),rawPath1(:,2),'--','Color',[c1 .5],'LineWidth',1.3,'DisplayName','Raw Seg.1');
plot(rawPath2(:,1),rawPath2(:,2),'--','Color',[c2 .5],'LineWidth',1.3,'DisplayName','Raw Seg.2');
plot(smoothPath1(:,1),smoothPath1(:,2),'-','Color',c1,'LineWidth',2.8,'DisplayName','G2CBS Seg.1');
plot(smoothPath2(:,1),smoothPath2(:,2),'-','Color',c2,'LineWidth',2.8,'DisplayName','G2CBS Seg.2');
plot(start(1),start(2),'^','MarkerSize',13,'MarkerFaceColor','g','Color','g','DisplayName','Start');
plot(waypoint(1),waypoint(2),'s','MarkerSize',11,'MarkerFaceColor','b','Color','b','DisplayName','Waypoint');
plot(goal(1),goal(2),'p','MarkerSize',16,'MarkerFaceColor','r','Color','r','DisplayName','Goal');
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
ylabel('Path heading [deg]','FontSize',10); xlabel('Normalised arc length','FontSize',10);
title('Curvature & Heading Profile','FontSize',11,'FontWeight','bold'); legend('Location','best','FontSize',9);
set(ax1R,'FontSize',10);

%% ======== TAB 2: Trajectory ========
tab2 = uitab(tg,'Title',' Trajectory ');
ax2  = axes('Parent',tab2,'Position',[0.05 0.08 0.90 0.86]);
hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal'); xlim(ax2,[0 xMax]); ylim(ax2,[0 yMax]);
xlabel(ax2,'X [m]','FontSize',12); ylabel(ax2,'Y [m]','FontSize',12);
title(ax2,'USV Actual Trajectory vs G2CBS Reference','FontSize',13,'FontWeight','bold');
for i=1:size(obstacles,1)
    xc=obstacles(i,1); yc=obstacles(i,2); rc=obstacles(i,3)+displayMargin;
    fill(ax2,xc+rc*cos(theta_c),yc+rc*sin(theta_c),cObs,'FaceAlpha',.15,'EdgeColor',cObs,'LineWidth',1.2);
    fill(ax2,xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),cObs,'FaceAlpha',.6,'EdgeColor','none');
end
plot(ax2,rawPathAll(:,1),rawPathAll(:,2),':','Color',[.6 .6 .6],'LineWidth',1.2,'DisplayName','RRT* Raw');
plot(ax2,fullPath(:,1),fullPath(:,2),'--','Color',[.2 .7 .2],'LineWidth',2.0,'DisplayName','G2CBS Ref');
idx1=find(segs==1); idx2=find(segs==2);
if ~isempty(idx1), plot(ax2,state(idx1,1),state(idx1,2),'-','Color',c1,'LineWidth',2.8,'DisplayName','Traj Seg.1'); end
if ~isempty(idx2), plot(ax2,state(idx2,1),state(idx2,2),'-','Color',c2,'LineWidth',2.8,'DisplayName','Traj Seg.2'); end
% Plot Marker Start, WP, Goal di Tab Trajectory (Bentuk Titik Presisi)
plot(ax2,start(1),start(2),'o','MarkerSize',8,'MarkerFaceColor','g','Color','g','DisplayName','Start');
plot(ax2,waypoint(1),waypoint(2),'o','MarkerSize',8,'MarkerFaceColor','b','Color','b','DisplayName','Waypoint');
plot(ax2,goal(1),goal(2),'o','MarkerSize',8,'MarkerFaceColor','r','Color','r','DisplayName','Goal');
legend(ax2,'Location','northwest','FontSize',10); set(ax2,'FontSize',11);

%% ======== TAB 3: State Variables ========
tab3 = uitab(tg,'Title',' States ');
sTit = {'Heading \psi [deg]','Heading Error [deg]','Cross-Track Error [m]',...
        'Surge u [m/s]','Sway v [m/s]','Yaw Rate r [deg/s]',...
        'Rudder \delta [deg]','Roll \phi [deg]','Roll Rate p [deg/s]'};
sCol = {c1,c3,c4, c1,c2,c3, c4,c1,c2};
sY   = {rad2deg(log.psi_d(1:Np)), rad2deg(log.psi_e(1:Np)), log.cte(1:Np),...
        log.u(1:Np), log.v(1:Np), rad2deg(log.r(1:Np)),...
        rad2deg(log.delta(1:Np)), rad2deg(log.phi(1:Np)), rad2deg(log.p(1:Np))};
for i=1:9
    axS = subplot(3,3,i,'Parent',tab3);
    plot(axS,t_plot,sY{i},'-','Color',sCol{i},'LineWidth',1.8); hold(axS,'on');
    if i==1, plot(axS,t_plot,rad2deg(state(1:Np,3)),'--','Color',c2,'LineWidth',1.5); end
    if i==7, yline(axS, rad2deg(USV.deltaMax),'r--'); yline(axS,-rad2deg(USV.deltaMax),'r--'); end
    if i==4, yline(axS,U0_saved,'r--','LineWidth',1.2); end
    yline(axS,0,'k:','LineWidth',0.8); xlabel(axS,'t [s]','FontSize',9);
    title(axS,sTit{i},'FontSize',10,'FontWeight','bold'); grid(axS,'on'); set(axS,'FontSize',9);
end

%% ======== TAB 4: Performance ========
tab4 = uitab(tg,'Title',' Performance ');
ax41=subplot(2,2,1,'Parent',tab4);
spd=sqrt(log.u(1:Np).^2+log.v(1:Np).^2);
plot(ax41,t_plot,spd,'-','Color',c1,'LineWidth',2); hold(ax41,'on');
yline(ax41,U0_saved,'r--','LineWidth',1.5); title(ax41,'Total Speed','FontWeight','bold');
xlabel(ax41,'t [s]'); ylabel(ax41,'[m/s]'); grid(ax41,'on'); ylim(ax41,[0 2.5]);

ax42=subplot(2,2,2,'Parent',tab4);
dGoal=sqrt((state(1:Np,1)-goal(1)).^2+(state(1:Np,2)-goal(2)).^2);
plot(ax42,t_plot,dGoal,'-','Color',c2,'LineWidth',2); hold on;
yline(ax42,GOAL_TOL,'r--','LineWidth',1.5); 
title(ax42,'Distance to Goal','FontWeight','bold'); xlabel(ax42,'t [s]'); ylabel(ax42,'[m]'); grid(ax42,'on');

ax43=subplot(2,2,3,'Parent',tab4);
cteA=abs(log.cte(1:Np));
plot(ax43,t_plot,cteA,'-','Color',[c3 .4],'LineWidth',1); hold(ax43,'on');
yline(ax43, 1.0, 'r--', 'LineWidth', 1.0); 
title(ax43,'Cross-Track Error (Abs)','FontWeight','bold'); xlabel(ax43,'t [s]'); ylabel(ax43,'[m]'); grid(ax43,'on');

ax44=subplot(2,2,4,'Parent',tab4);
plot(ax44,t_plot,rad2deg(log.delta(1:Np)),'-','Color',c4,'LineWidth',1.5);
ylabel(ax44,'\delta [deg]'); title(ax44,'Rudder Angle','FontWeight','bold');
xlabel(ax44,'t [s]'); grid(ax44,'on'); set(ax44,'FontSize',9);

%% ======== TAB 5: G2CBS Analysis ========
tab5 = uitab(tg,'Title',' G2CBS Analysis ');
ax56=axes('Parent',tab5,'Position',[0.2 0.2 0.6 0.6]); axis(ax56,'off');
lenSmo=sum(sqrt(sum(diff(fullPath).^2,2)));
stats={'Metric','G2CBS';'Max |\kappa|',sprintf('%.4f',max(abs(pathCurv)));
    'Path length [m]',sprintf('%.2f',lenSmo); 'Waypoints',sprintf('%d',nWP)};
uitable('Parent',tab5,'Data',stats(2:end,:),'ColumnName',stats(1,:),...
    'Units','normalized','Position',[0.2 0.2 0.6 0.6],'FontSize',11);
text(ax56,.5,1.05,'G2CBS Stats','HorizontalAlignment','center','FontSize',12,'FontWeight','bold');

%% =================== ANALISIS ERROR CTE =========================
cte_actual = log.cte(1:Np);
MAE_CTE = mean(abs(cte_actual));
RMSE_CTE = sqrt(mean(cte_actual.^2));
MAX_CTE = max(abs(cte_actual));
fprintf('\n=== HASIL ANALISIS ERROR (PATH TRACKING) ===\n');
fprintf('Maximum CTE : %.3f meter\n', MAX_CTE);
fprintf('MAE CTE     : %.3f meter\n', MAE_CTE);
fprintf('RMSE CTE    : %.3f meter\n', RMSE_CTE);
fprintf('============================================\n');

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
function sNew=usv4DOF(s,delta,USV,dt)
    k1=shipEOM(s,          delta,USV); k2=shipEOM(s+.5*dt*k1, delta,USV);
    k3=shipEOM(s+.5*dt*k2, delta,USV); k4=shipEOM(s+dt*k3,    delta,USV);
    sNew=s+(dt/6)*(k1+2*k2+2*k3+k4); sNew(3)=wrapToPi(sNew(3)); sNew(4)=wrapToPi(sNew(4));
end

function ds = shipEOM(s, delta, USV)
    psi = s(3); phi = s(4); u = s(5); v = s(6); r = s(7); p = s(8);
    
    if isfield(USV, 'T0_active'), nom_T = USV.T0_active; else, nom_T = USV.T0; end
    tau_u = nom_T + 45.0 * (USV.U0 - u); 
    tau_u = max(0, tau_u); 
    
    Ur = max(sqrt(u^2 + v^2), 0.05);
    tau_r = USV.Ky_r * Ur^2 * sin(delta); 

    u_dot = USV.A2*v*r - USV.A3*u - USV.A4*abs(u)*u - USV.A5*(abs(u)^2)*u + USV.A37*tau_u;
    v_dot = -(1/USV.A2)*u*r - USV.A7*v - USV.A8*abs(v)*v - USV.A9*(abs(v)^2)*v;
    r_dot = -USV.A34*r - USV.A35*abs(r)*r - USV.A36*(r^3) - USV.A32*v*u + USV.A33*u*v + USV.A38*tau_r;
    p_dot = -USV.A18*p - USV.A19*abs(p)*p - USV.A20*(p^3) + USV.A21*v*r - USV.A22*sin(phi) + USV.A23*tau_r;
      
    x_dot = u * cos(psi) - v * sin(psi); 
    y_dot = u * sin(psi) + v * cos(psi);
    ds = [x_dot, y_dot, r, p, u_dot, v_dot, r_dot, p_dot];
end

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
    nodes=zeros(rrt.maxIter+2,2); parent=zeros(rrt.maxIter+2,1); costArr=zeros(rrt.maxIter+2,1);
    nodes(1,:)=startPt; nNodes=1; foundGoal=false; goalIdx=0;
    for i=1:rrt.maxIter
        if rand<rrt.goalBias, sample=goalPt; else, sample=[rand*xMax,rand*yMax]; end
        d=sqrt(sum((nodes(1:nNodes,:)-repmat(sample,nNodes,1)).^2,2)); [~,nI]=min(d);
        dir=sample-nodes(nI,:); dL=norm(dir);
        if dL>rrt.stepSize, newPt=nodes(nI,:)+rrt.stepSize*dir/dL; else, newPt=sample; end
        if ~isCF(nodes(nI,:),newPt,obs,margin)||~inBounds(newPt,xMax,yMax), continue; end
        d2=sqrt(sum((nodes(1:nNodes,:)-repmat(newPt,nNodes,1)).^2,2)); nIdxs=find(d2<=rrt.rewireRad);
        bP=nI; bC=costArr(nI)+norm(newPt-nodes(nI,:));
        for ni=nIdxs'
            c=costArr(ni)+norm(newPt-nodes(ni,:));
            if c<bC&&isCF(nodes(ni,:),newPt,obs,margin), bC=c; bP=ni; end
        end
        nNodes=nNodes+1; nodes(nNodes,:)=newPt; parent(nNodes)=bP; costArr(nNodes)=bC;
        for ni=nIdxs'
            nc=costArr(nNodes)+norm(nodes(ni,:)-newPt);
            if nc<costArr(ni)&&isCF(newPt,nodes(ni,:),obs,margin), parent(ni)=nNodes; costArr(ni)=nc; end
        end
        if norm(newPt-goalPt)<rrt.goalTol, foundGoal=true; goalIdx=nNodes; end
    end
    if ~foundGoal
        d=sqrt(sum((nodes(1:nNodes,:)-repmat(goalPt,nNodes,1)).^2,2)); [~,goalIdx]=min(d);
    end
    path=goalPt; idx=goalIdx;
    while idx~=1, path=[nodes(idx,:);path]; idx=parent(idx); end
    path=[startPt;path(2:end,:)];
end
function f=isCF(p1,p2,obs,margin)
    f=true;
    for s=linspace(0,1,20)
        px=p1(1)+s*(p2(1)-p1(1)); py=p1(2)+s*(p2(2)-p1(2));
        for i=1:size(obs,1), if sqrt((px-obs(i,1))^2+(py-obs(i,2))^2)<(obs(i,3)+margin), f=false; return; end, end
    end
end
function b=inBounds(pt,xMax,yMax), b=pt(1)>=0&&pt(1)<=xMax&&pt(2)>=0&&pt(2)<=yMax; end
function pathS = g2cbsSmooth(waypts, nPts)
    if isempty(waypts), pathS = []; return; end
    minD = 1e-4; keep = true(size(waypts, 1), 1);
    for i = 2:size(waypts, 1), if norm(waypts(i,:) - waypts(i-1,:)) < minD, keep(i) = false; end, end
    waypts = waypts(keep, :); n = size(waypts, 1);
    if n < 3
        if n == 1, waypts = [waypts; waypts + 0.01]; n = 2; end
        t0 = linspace(0, 1, n); tO = linspace(0, 1, nPts);
        pathS = [interp1(t0, waypts(:,1), tO, 'linear')', interp1(t0, waypts(:,2), tO, 'linear')'];
        pathS(1,:) = waypts(1,:); pathS(end,:) = waypts(end,:); return;
    end
    d = sqrt(sum(diff(waypts).^2, 2)); d(d < 1e-10) = 1e-10; t = [0; cumsum(d)]; t = t / t(end);
    for i = 2:length(t), if t(i) <= t(i-1), t(i) = t(i-1) + 1e-9; end, end
    t = t / t(end);
    ppX = spline(t, waypts(:,1)); ppY = spline(t, waypts(:,2));  
    tD = linspace(0, 1, nPts * 10); xD = ppval(ppX, tD); yD = ppval(ppY, tD);
    ds = sqrt(diff(xD).^2 + diff(yD).^2); sA = [0, cumsum(ds)]; sA = sA / sA(end);
    sU = linspace(0, 1, nPts);
    xS = interp1(sA, xD, sU, 'linear'); yS = interp1(sA, yD, sU, 'linear');
    xS(1) = waypts(1,1); yS(1) = waypts(1,2); xS(end) = waypts(end,1); yS(end) = waypts(end,2);
    pathS = [xS(:), yS(:)];
end
function s=computeArcLength(path), d=sqrt(sum(diff(path).^2,2)); s=[0;cumsum(d)]; end
function kappa=computePathCurvature(path)
    n=size(path,1); if n<3, kappa=zeros(n,1); return; end
    x=path(:,1); y=path(:,2); d=sqrt(diff(x).^2+diff(y).^2); d(d<1e-12)=1e-12; s=[0;cumsum(d)];
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
    den=(xp.^2+yp.^2).^1.5; den(den<1e-9)=1e-9; kappa=(xp.*ypp-yp.*xpp)./den;
end
function psi=computePathHeading(path), dx=diff(path(:,1)); dy=diff(path(:,2)); psi=[atan2(dy,dx); atan2(dy(end),dx(end))]; end
function runAnimation(tab,state,fullPath,rawAll,sP1,sP2,obstacles,USV,log,N_end,dt,xMax,yMax,margin,goal,startPt,wayptPt,ILOS,c1,c2,theta_c,U0_saved)
    axA=axes('Parent',tab,'Position',[0.03 0.17 0.61 0.79]); hold(axA,'on'); grid(axA,'on'); axis(axA,'equal');
    axA.Color=[0.07 0.07 0.13]; axA.GridColor=[.4 .4 .5]; axA.GridAlpha=.3; axA.XColor=[.9 .9 .9]; axA.YColor=[.9 .9 .9];
    xlim(axA,[0 xMax]); ylim(axA,[0 yMax]); xlabel(axA,'X [m]','Color',[.9 .9 .9]); ylabel(axA,'Y [m]','Color',[.9 .9 .9]);
    title(axA,'USV 4-DOF | RRT* + G2CBS + ILOS + PD','Color',[.95 .95 1],'FontSize',12,'FontWeight','bold');
    obsC={[1 .3 .3],[1 .5 .2],[1 .7 .2],[.8 .3 1],[.3 .8 1],[.3 1 .5]};
    for i=1:size(obstacles,1)
        xc=obstacles(i,1); yc=obstacles(i,2); rc=obstacles(i,3)+margin; oc=obsC{mod(i-1,6)+1};
        fill(axA,xc+rc*cos(theta_c),yc+rc*sin(theta_c),oc,'FaceAlpha',.12,'EdgeColor',oc,'LineStyle','--');
        fill(axA,xc+obstacles(i,3)*cos(theta_c),yc+obstacles(i,3)*sin(theta_c),oc,'FaceAlpha',.7,'EdgeColor','none');
        text(axA,xc,yc,sprintf('O%d',i),'Color','w','FontSize',9,'HorizontalAlignment','center','FontWeight','bold');
    end
    plot(axA,rawAll(:,1),rawAll(:,2),':','Color',[.55 .55 .55],'LineWidth',1.0);
    plot(axA,sP1(:,1),sP1(:,2),'-','Color',[c1 .65],'LineWidth',1.6); plot(axA,sP2(:,1),sP2(:,2),'-','Color',[c2 .65],'LineWidth',1.6);
    % PERBAIKAN: Mengubah marker menjadi bentuk titik (dot) yang presisi
plot(axA,startPt(1),startPt(2),'o','MarkerSize',8,'MarkerFaceColor','g','MarkerEdgeColor','w','LineWidth',1);
plot(axA,wayptPt(1),wayptPt(2),'o','MarkerSize',8,'MarkerFaceColor','b','MarkerEdgeColor','w','LineWidth',1);
plot(axA,goal(1),goal(2),'o','MarkerSize',8,'MarkerFaceColor','r','MarkerEdgeColor','w','LineWidth',1);
    hTrail=plot(axA,NaN,NaN,'-','Color',[.4 .85 1],'LineWidth',2.2);
    Ls=1.5; Bs=0.5; hullX=[.5*Ls,.28*Ls,-.5*Ls,-.5*Ls,.28*Ls]; hullY=[0,.5*Bs,.4*Bs,-.4*Bs,-.5*Bs];
    hShip=fill(axA,NaN,NaN,[.2 .6 1],'FaceAlpha',.88,'EdgeColor','w');
    hBow=plot(axA,NaN,NaN,'->','Color',[1 1 0],'MarkerSize',8,'LineWidth',2);
    hInfo=text(axA,.3,yMax*.97,'','Color','w','FontSize',9,'VerticalAlignment','top','BackgroundColor',[0 0 0 .55]);
    
    bg=[.06 .06 .12];
    axC=axes('Parent',tab,'Position',[0.66 0.68 0.31 0.25]); axC.Color=bg; axC.XColor=[.8 .8 .8]; axC.YColor=[.8 .8 .8];
    grid(axC,'on'); hold(axC,'on'); ylabel(axC,'CTE [m]'); title(axC,'Cross-Track Error','Color','w');
    hCTE=plot(axC,NaN,NaN,'-','Color',[.3 .8 1],'LineWidth',1.5); yline(axC,0,'--','Color',[.6 .6 .6]);
    
    axSp=axes('Parent',tab,'Position',[0.66 0.40 0.31 0.24]); axSp.Color=bg; axSp.XColor=[.8 .8 .8]; axSp.YColor=[.8 .8 .8];
    grid(axSp,'on'); hold(axSp,'on'); ylabel(axSp,'[m/s]'); title(axSp,'Speed','Color','w');
    hSpd=plot(axSp,NaN,NaN,'-','Color',[1 .65 .2],'LineWidth',1.5); yline(axSp,U0_saved,'--','Color',[1 .4 .4]);
    
    axDl=axes('Parent',tab,'Position',[0.66 0.12 0.31 0.24]); axDl.Color=bg; axDl.XColor=[.8 .8 .8]; axDl.YColor=[.8 .8 .8];
    grid(axDl,'on'); hold(axDl,'on'); xlabel(axDl,'t [s]'); ylabel(axDl,'\delta [deg]'); title(axDl,'Rudder Angle','Color','w');
    hDel=plot(axDl,NaN,NaN,'-','Color',[1 .5 .8],'LineWidth',1.5);
    yline(axDl, rad2deg(USV.deltaMax),'--','Color',[1 .3 .3]); yline(axDl,-rad2deg(USV.deltaMax),'--','Color',[1 .3 .3]);
    
    skip=max(1,round(0.08/dt));
    for k=1:skip:N_end
        if ~ishandle(tab), break; end
        xk=state(k,1); yk=state(k,2); psik=state(k,3); uk=state(k,5); vk=state(k,6); delk=log.delta(k); tk=(k-1)*dt;
        set(hTrail,'XData',state(1:k,1),'YData',state(1:k,2));
        R=[cos(psik) -sin(psik);sin(psik) cos(psik)]; hw=R*[hullX;hullY];
        set(hShip, 'XData',xk+hw(1,:), 'YData',yk+hw(2,:));
        set(hBow,  'XData',[xk,xk+.5*Ls*cos(psik)],'YData',[yk,yk+.5*Ls*sin(psik)]);
        kW=max(1,k-round(60/dt)); tW=(kW-1:k-1)*dt;
        set(hCTE,'XData',tW,'YData',log.cte(kW:k)); xlim(axC,[tW(1) max(tW(end)+.1,tW(1)+5)]);
        sp=sqrt(log.u(kW:k).^2+log.v(kW:k).^2); set(hSpd,'XData',tW,'YData',sp); xlim(axSp,[tW(1) max(tW(end)+.1,tW(1)+5)]);
        set(hDel,'XData',tW,'YData',rad2deg(log.delta(kW:k))); xlim(axDl,[tW(1) max(tW(end)+.1,tW(1)+5)]);
        set(hInfo,'String',sprintf('t = %.1f s\nSpeed = %.2f m/s\nCTE = %.2f m',tk,sqrt(uk^2+vk^2),log.cte(k)));
        drawnow limitrate; pause(0.001);
    end
end